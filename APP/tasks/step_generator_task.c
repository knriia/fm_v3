#include "step_generator_task.h"

#include "motion/execution_buffer.h"
#include "motion/motion_segment_generator.h"
#include "motion/plan_buffer.h"
#include "motion_axis_hardware.h"
#include "step_stream_axis.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include <stddef.h>

#define STEP_GENERATOR_TASK_FLAG_BLOCK_READY (1UL << 0)
#define STEP_GENERATOR_TASK_FLAG_BUFFER_SPACE (1UL << 1)
#define STEP_GENERATOR_TASK_FLAG_STOP_RESET (1UL << 2)
#define STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED (1UL << 3)
#define STEP_GENERATOR_TASK_FLAGS_ALL                                                                                  \
    (STEP_GENERATOR_TASK_FLAG_BLOCK_READY | STEP_GENERATOR_TASK_FLAG_BUFFER_SPACE |                                    \
        STEP_GENERATOR_TASK_FLAG_STOP_RESET | STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED)

typedef struct {
    MotionSegmentGenerator_t generator;
    /* Event already generated but not yet accepted by the full ExecutionBuffer. */
    MotionExecutionEvent_t pending_event;
    uint8_t block_active;
    uint8_t event_pending;
} StepGeneratorState_t;

static volatile StepGeneratorTaskDiagnostics step_generator_task_diagnostics;
/* Written once at task start and read by MotionTask and the timer ISR to wake the task. */
static osThreadId_t volatile step_generator_task_thread_id;
/* Set by STOP in another task and cleared by this task once its state is reset. */
static volatile uint8_t step_generator_task_halted;

static void step_generator_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void step_generator_task_signal(uint32_t flags) {
    const osThreadId_t thread_id = step_generator_task_thread_id;
    if (thread_id == NULL) {
        return;
    }
    (void)osThreadFlagsSet(thread_id, flags);
}

void step_generator_task_notify_block_ready(void) { step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_BLOCK_READY); }

void step_generator_task_notify_ring_half_released(void) {
    step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED);
}

void step_generator_task_halt(void) { step_generator_task_halted = 1U; }

void step_generator_task_resume_after_stop(void) { step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_STOP_RESET); }

/* Runs in the timer ISR when ExecutionBuffer drains to its low watermark. */
static void step_generator_task_buffer_space_available(void) {
    step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_BUFFER_SPACE);
}

/* Fills ExecutionBuffer until it is full or no planned block is left. */
static void step_generator_task_fill(StepGeneratorState_t *state) {
    for (;;) {
        if (state->event_pending != 0U) {
            if (execution_buffer_try_push(&state->pending_event) != EXECUTION_BUFFER_STATUS_OK) {
                step_generator_counter_increment(&step_generator_task_diagnostics.buffer_full_waits);
                return;
            }
            state->event_pending = 0U;
            step_generator_counter_increment(&step_generator_task_diagnostics.events_generated);
        }

        if (state->block_active == 0U) {
            MotionPlanBlock_t block;
            if (plan_buffer_try_pop(&block) != PLAN_BUFFER_STATUS_OK) {
                return;
            }
            if (motion_segment_generator_init(&state->generator, &block) != MOTION_SEGMENT_GENERATOR_STATUS_OK) {
                /* The planner validated the block, so this is a defect; the block is skipped and counted. */
                step_generator_counter_increment(&step_generator_task_diagnostics.generator_errors);
                continue;
            }
            state->block_active = 1U;
            step_generator_counter_increment(&step_generator_task_diagnostics.blocks_started);
        }

        const MotionSegmentGeneratorStatus_t status =
            motion_segment_generator_next(&state->generator, &state->pending_event);
        if (status == MOTION_SEGMENT_GENERATOR_STATUS_OK) {
            state->event_pending = 1U;
            continue;
        }
        state->block_active = 0U;
        if (status == MOTION_SEGMENT_GENERATOR_STATUS_DONE) {
            step_generator_counter_increment(&step_generator_task_diagnostics.blocks_completed);
        } else {
            step_generator_counter_increment(&step_generator_task_diagnostics.generator_errors);
        }
    }
}

/* Starts the idle X axis when events are waiting; X is the only axis on the planned stream for now. A running axis
 * picks new events up by itself, and one that stopped or faulted is left alone until MotionTask recovers it. */
static void step_generator_task_start_axis(void) {
    if (step_generator_task_halted != 0U || execution_buffer_count() == 0U) {
        return;
    }

    /* MotionTask also starts this axis (homing, X_JOG). With task switching paused, the status check and the
     * start cannot interleave with its start; interrupts keep running. */
    (void)osKernelLock();
    const MotionAxisHardwareStatus_t status = motion_axis_hardware_get_status(MOTION_AXIS_X);
    if (status == MOTION_AXIS_HARDWARE_STATUS_READY || status == MOTION_AXIS_HARDWARE_STATUS_COMPLETE) {
        if (motion_axis_hardware_start(MOTION_AXIS_X) == MOTION_AXIS_HARDWARE_STATUS_STARTING) {
            step_generator_counter_increment(&step_generator_task_diagnostics.axis_starts);
        } else {
            step_generator_counter_increment(&step_generator_task_diagnostics.axis_start_errors);
        }
    }
    (void)osKernelUnlock();
}

/* DMA interrupt: the stream axis has stopped itself after a ring fault, so stop feeding until STOP resets us. */
static void step_generator_task_stream_fault(StepStreamAxisFault_t fault) {
    (void)fault;
    step_generator_task_halt();
}

void StepGeneratorTask(void *argument) {
    (void)argument;
    StepGeneratorState_t state = {0};
    step_generator_task_thread_id = osThreadGetId();
    execution_buffer_set_space_listener(step_generator_task_buffer_space_available);
    static const StepStreamAxisListener_t stream_listener = {
        step_generator_task_notify_ring_half_released,
        step_generator_task_stream_fault,
    };
    (void)step_stream_axis_set_listener(&stream_listener);

    uint32_t flags = 0U;
    for (;;) {
        if ((flags & STEP_GENERATOR_TASK_FLAG_STOP_RESET) != 0U) {
            /* The axis has stopped and this task is not filling, so both ends of ExecutionBuffer are idle. */
            state = (StepGeneratorState_t){0};
            execution_buffer_clear();
            step_generator_task_halted = 0U;
            step_generator_counter_increment(&step_generator_task_diagnostics.stop_resets);
        }
        if (step_generator_task_halted == 0U) {
            step_generator_task_fill(&state);
            step_generator_task_start_axis();
        }
        /* Flags set while filling stay pending, so a block or free space that arrives meanwhile is not missed. */
        flags = osThreadFlagsWait(STEP_GENERATOR_TASK_FLAGS_ALL, osFlagsWaitAny, osWaitForever);
        configASSERT((flags & osFlagsError) == 0U);
    }
}

void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    diagnostics->blocks_started = step_generator_task_diagnostics.blocks_started;
    diagnostics->blocks_completed = step_generator_task_diagnostics.blocks_completed;
    diagnostics->events_generated = step_generator_task_diagnostics.events_generated;
    diagnostics->generator_errors = step_generator_task_diagnostics.generator_errors;
    diagnostics->buffer_full_waits = step_generator_task_diagnostics.buffer_full_waits;
    diagnostics->axis_starts = step_generator_task_diagnostics.axis_starts;
    diagnostics->axis_start_errors = step_generator_task_diagnostics.axis_start_errors;
    diagnostics->stop_resets = step_generator_task_diagnostics.stop_resets;
}

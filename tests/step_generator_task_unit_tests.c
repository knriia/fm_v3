#include "motion/execution_buffer.h"
#include "motion/motion_segment_generator.h"
#include "motion/plan_buffer.h"
#include "motion_axis_hardware.h"
#include "step_generator_task.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_THREAD_ID ((osThreadId_t)(uintptr_t)0x5678U)
#define TEST_WAIT_SCRIPT_LENGTH 4U

static uint32_t test_failures;
static uint32_t test_error_handler_calls;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint8_t test_mutex_object;
static uint32_t test_flags_set_calls;
static uint32_t test_flags_set_value;
static osThreadId_t test_flags_set_target;
static uint32_t test_wait_calls;
static uint32_t test_wait_results[TEST_WAIT_SCRIPT_LENGTH];
static uint8_t test_wait_pushes_block[TEST_WAIT_SCRIPT_LENGTH];
static uint32_t test_wait_script_count;
static uint32_t test_wait_last_timeout;
static uint8_t test_force_next_error;
static MotionAxisHardwareStatus_t test_axis_status;
static MotionAxisHardwareStatus_t test_axis_start_result;
static uint32_t test_axis_start_calls;
static uint32_t test_kernel_lock_depth;
static uint32_t test_kernel_lock_calls;
static uint32_t test_axis_started_unlocked;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

static void expect_true(uint8_t condition, const char *message) {
    if (condition == 0U) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
    }
}

static MotionPlanBlock_t make_x_block(uint32_t sequence, uint64_t steps) {
    return (MotionPlanBlock_t){
        .sequence = sequence,
        .step_delta = {(int64_t)steps, 0, 0},
        .path_length_um = steps,
        .master_step_count = steps,
        .step_event_rate_numerator = 1000U,
        .step_event_rate_denominator = 1U,
        .feedrate_um_per_s = 1000U,
        .moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK,
        .positive_direction_mask = MOTION_PLANNER_AXIS_X_MASK,
    };
}

static void push_block(uint32_t sequence, uint64_t steps) {
    const MotionPlanBlock_t block = make_x_block(sequence, steps);
    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "test block is queued");
}

void Error_Handler(void) {
    ++test_error_handler_calls;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
}

osMutexId_t osMutexNew(const osMutexAttr_t *attr) {
    (void)attr;
    return &test_mutex_object;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout) {
    (void)mutex_id;
    (void)timeout;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id) {
    (void)mutex_id;
    return osOK;
}

osThreadId_t osThreadGetId(void) { return TEST_THREAD_ID; }

int32_t osKernelLock(void) {
    ++test_kernel_lock_calls;
    ++test_kernel_lock_depth;
    return 0;
}

int32_t osKernelUnlock(void) {
    --test_kernel_lock_depth;
    return 1;
}

/* Only X runs the planned stream; a call for any other axis is counted as a defect. */
static uint32_t test_other_axis_calls;

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) {
    if (axis != MOTION_AXIS_X) {
        ++test_other_axis_calls;
    }
    return test_axis_status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_start(MotionAxis_t axis) {
    if (axis != MOTION_AXIS_X) {
        ++test_other_axis_calls;
    }
    ++test_axis_start_calls;
    if (test_kernel_lock_depth == 0U) {
        ++test_axis_started_unlocked;
    }
    test_axis_status = test_axis_start_result;
    return test_axis_start_result;
}

uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags) {
    ++test_flags_set_calls;
    test_flags_set_target = thread_id;
    test_flags_set_value |= flags;
    return test_flags_set_value;
}

/* Scripted wakeups; a scripted entry can queue a block first, like MotionTask does. The script end stops the task. */
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout) {
    (void)flags;
    (void)options;
    test_wait_last_timeout = timeout;
    const uint32_t index = test_wait_calls;
    ++test_wait_calls;
    if (index >= test_wait_script_count) {
        longjmp(test_jump_buffer, 2);
    }
    if (test_wait_pushes_block[index] != 0U) {
        push_block(200U + index, 2U);
    }
    return test_wait_results[index];
}

static MotionSegmentGeneratorStatus_t
test_motion_segment_generator_next(MotionSegmentGenerator_t *generator, MotionExecutionEvent_t *event);

#define motion_segment_generator_next test_motion_segment_generator_next
#define static
#include "../APP/tasks/step_generator_task.c"
#undef static
#undef motion_segment_generator_next

static MotionSegmentGeneratorStatus_t
test_motion_segment_generator_next(MotionSegmentGenerator_t *generator, MotionExecutionEvent_t *event) {
    if (test_force_next_error != 0U) {
        test_force_next_error = 0U;
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
    }
    return motion_segment_generator_next(generator, event);
}

static void reset_fixture(void) {
    plan_buffer_test_reset();
    expect_u32(plan_buffer_init(), PLAN_BUFFER_STATUS_OK, "plan buffer initializes");
    execution_buffer_init();
    execution_buffer_set_space_listener(NULL);
    step_generator_task_diagnostics = (StepGeneratorTaskDiagnostics){0};
    step_generator_task_thread_id = NULL;
    step_generator_task_halted = 0U;
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_flags_set_calls = 0U;
    test_flags_set_value = 0U;
    test_flags_set_target = NULL;
    test_wait_calls = 0U;
    test_wait_script_count = 0U;
    test_wait_last_timeout = 0U;
    test_force_next_error = 0U;
    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    test_axis_start_result = MOTION_AXIS_HARDWARE_STATUS_STARTING;
    test_axis_start_calls = 0U;
    test_kernel_lock_depth = 0U;
    test_kernel_lock_calls = 0U;
    test_axis_started_unlocked = 0U;
    for (uint32_t index = 0U; index < TEST_WAIT_SCRIPT_LENGTH; ++index) {
        test_wait_results[index] = 0U;
        test_wait_pushes_block[index] = 0U;
    }
}

static int run_task_until_jump(void) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        StepGeneratorTask(NULL);
    }
    test_jump_active = 0U;
    return jumped;
}

static void drain_execution_buffer(uint32_t *events, uint32_t *last_events) {
    MotionExecutionEvent_t event = {0};
    *events = 0U;
    *last_events = 0U;
    while (execution_buffer_try_pop(&event) == EXECUTION_BUFFER_STATUS_OK) {
        ++(*events);
        *last_events += event.last_in_block;
    }
}

static void test_task_fills_buffer_and_waits_for_work(void) {
    reset_fixture();
    push_block(1U, 3U);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    test_wait_pushes_block[0] = 1U;
    expect_u32((uint32_t)run_task_until_jump(), 2U, "task ends at the scripted wait");
    expect_true(step_generator_task_thread_id == TEST_THREAD_ID, "task stores its id for wakeups");
    expect_u32(test_wait_calls, 2U, "task waits again after each fill");
    expect_u32(test_wait_last_timeout, osWaitForever, "task waits for work without polling");

    uint32_t events = 0U;
    uint32_t last_events = 0U;
    drain_execution_buffer(&events, &last_events);
    expect_u32(events, 5U, "both blocks are turned into step events");
    expect_u32(last_events, 2U, "each block ends with its last event marked");

    StepGeneratorTaskDiagnostics diagnostics = {0};
    step_generator_task_get_diagnostics(&diagnostics);
    step_generator_task_get_diagnostics(NULL);
    expect_u32(diagnostics.blocks_started, 2U, "started blocks are counted");
    expect_u32(diagnostics.blocks_completed, 2U, "completed blocks are counted");
    expect_u32(diagnostics.events_generated, 5U, "queued events are counted");
    expect_u32(diagnostics.generator_errors, 0U, "valid blocks produce no errors");
    expect_u32(diagnostics.buffer_full_waits, 0U, "small blocks fit into the buffer");

    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = osFlagsErrorParameter;
    expect_u32((uint32_t)run_task_until_jump(), 1U, "wait error stops the task");
    expect_u32(test_error_handler_calls, 1U, "wait error is fatal");
}

static void test_full_buffer_keeps_pending_event_and_wakes_on_space(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    push_block(2U, EXECUTION_BUFFER_CAPACITY + 10U);
    expect_u32((uint32_t)run_task_until_jump(), 2U, "task fills until the buffer is full");
    expect_u32(execution_buffer_count(), EXECUTION_BUFFER_CAPACITY, "buffer is filled completely");
    expect_u32(step_generator_task_diagnostics.buffer_full_waits, 1U, "full buffer stops generation");

    MotionExecutionEvent_t event = {0};
    test_flags_set_calls = 0U;
    while (execution_buffer_count() > EXECUTION_BUFFER_LOW_WATERMARK) {
        expect_u32(execution_buffer_try_pop(&event), EXECUTION_BUFFER_STATUS_OK, "consumer takes an event");
    }
    expect_u32(test_flags_set_calls, 1U, "draining to the low watermark wakes the generator once");
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_BUFFER_SPACE, "drain sets the buffer space flag");

    StepGeneratorState_t state = {0};
    state.generator = (MotionSegmentGenerator_t){0};
    reset_fixture();
    push_block(3U, EXECUTION_BUFFER_CAPACITY + 1U);
    step_generator_task_fill(&state);
    expect_u32(state.event_pending, 1U, "event that does not fit is kept");
    expect_u32(execution_buffer_try_pop(&event), EXECUTION_BUFFER_STATUS_OK, "consumer frees one slot");
    step_generator_task_fill(&state);
    expect_u32(state.event_pending, 0U, "kept event is queued once space appears");
    expect_u32(state.block_active, 0U, "block finishes after its last event");
    uint32_t events = 0U;
    uint32_t last_events = 0U;
    drain_execution_buffer(&events, &last_events);
    expect_u32(events, EXECUTION_BUFFER_CAPACITY, "buffer holds the rest of the block");
    expect_u32(last_events, 1U, "last event of the long block is marked");
    expect_u32(
        step_generator_task_diagnostics.events_generated,
        EXECUTION_BUFFER_CAPACITY + 1U,
        "every event of the long block is queued"
    );
}

static void test_invalid_blocks_are_skipped(void) {
    reset_fixture();
    StepGeneratorState_t state = {0};
    MotionPlanBlock_t invalid = make_x_block(4U, 3U);
    invalid.master_step_count = 0U;
    expect_u32(plan_buffer_try_push(&invalid), PLAN_BUFFER_STATUS_OK, "invalid block is queued");
    push_block(5U, 2U);
    step_generator_task_fill(&state);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "invalid block is counted");
    expect_u32(step_generator_task_diagnostics.blocks_completed, 1U, "next valid block still runs");
    expect_u32(execution_buffer_count(), 2U, "only the valid block produces events");

    reset_fixture();
    state = (StepGeneratorState_t){0};
    push_block(6U, 3U);
    test_force_next_error = 1U;
    step_generator_task_fill(&state);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "generator failure mid-block is counted");
    expect_u32(step_generator_task_diagnostics.blocks_completed, 0U, "failed block is not reported as completed");
    expect_u32(state.block_active, 0U, "failed block is dropped");
}

static void test_axis_start(void) {
    reset_fixture();
    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    step_generator_task_start_axis();
    expect_u32(test_axis_start_calls, 0U, "axis is not started without events");

    push_block(7U, 2U);
    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    expect_u32((uint32_t)run_task_until_jump(), 2U, "task fills and starts the idle axis");
    expect_u32(test_axis_start_calls, 1U, "idle axis is started once events are queued");
    expect_u32(test_axis_started_unlocked, 0U, "axis is started with task switching paused");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed after the start");
    expect_u32(step_generator_task_diagnostics.axis_starts, 1U, "axis start is counted");

    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    step_generator_task_start_axis();
    expect_u32(test_axis_start_calls, 2U, "axis that finished earlier is restarted for new events");

    const MotionAxisHardwareStatus_t busy_or_halted[] = {
        MOTION_AXIS_HARDWARE_STATUS_STARTING,
        MOTION_AXIS_HARDWARE_STATUS_RUNNING,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT,
    };
    for (uint32_t index = 0U; index < sizeof(busy_or_halted) / sizeof(busy_or_halted[0]); ++index) {
        test_axis_status = busy_or_halted[index];
        step_generator_task_start_axis();
    }
    expect_u32(test_axis_start_calls, 2U, "running, stopped or faulted axis is not started");
    expect_u32(test_kernel_lock_depth, 0U, "every pause of task switching is undone");

    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    test_axis_start_result = MOTION_AXIS_HARDWARE_STATUS_FAULT_UNDERRUN;
    step_generator_task_start_axis();
    expect_u32(step_generator_task_diagnostics.axis_start_errors, 1U, "failed axis start is counted");
}

static void test_stop_halts_and_resets(void) {
    reset_fixture();
    push_block(8U, 3U);
    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    step_generator_task_halt();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(), 2U, "halted task only waits");
    expect_u32(execution_buffer_count(), 0U, "halted task generates nothing");
    expect_u32(test_axis_start_calls, 0U, "halted task does not start the axis");

    step_generator_task_start_axis();
    expect_u32(test_axis_start_calls, 0U, "axis start is refused while halted");

    /* Events left from the stopped move; MotionTask has already emptied the plan buffer at this point. */
    reset_fixture();
    const MotionExecutionEvent_t leftover = {.interval_ns = 1000U, .step_mask = MOTION_PLANNER_AXIS_X_MASK};
    expect_u32(execution_buffer_try_push(&leftover), EXECUTION_BUFFER_STATUS_OK, "leftover event is queued");
    test_axis_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    step_generator_task_halt();
    expect_u32((uint32_t)run_task_until_jump(), 2U, "task resets on the STOP reset flag");
    expect_u32(step_generator_task_halted, 0U, "reset lifts the halt");
    expect_u32(step_generator_task_diagnostics.stop_resets, 1U, "reset is counted");
    expect_u32(execution_buffer_count(), 0U, "reset drops the leftover events");
    expect_u32(test_axis_start_calls, 0U, "nothing is left to start after the reset");

    reset_fixture();
    step_generator_task_resume_after_stop();
    expect_u32(test_flags_set_calls, 0U, "resume before the task starts is ignored");
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_task_resume_after_stop();
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_STOP_RESET, "resume sets the reset flag");
}

static void test_notifications(void) {
    reset_fixture();
    step_generator_task_notify_block_ready();
    step_generator_task_buffer_space_available();
    expect_u32(test_flags_set_calls, 0U, "wakeups before the task starts are ignored");

    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_task_notify_block_ready();
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_BLOCK_READY, "new block sets its flag");
    expect_true(test_flags_set_target == TEST_THREAD_ID, "flag goes to the generator task");
    test_flags_set_value = 0U;
    step_generator_task_buffer_space_available();
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_BUFFER_SPACE, "free space sets its flag");

    volatile uint32_t counter = UINT32_MAX;
    step_generator_counter_increment(&counter);
    expect_u32(counter, UINT32_MAX, "diagnostic counter saturates");
}

int main(void) {
    test_task_fills_buffer_and_waits_for_work();
    test_full_buffer_keeps_pending_event_and_wakes_on_space();
    test_invalid_blocks_are_skipped();
    test_axis_start();
    test_stop_halts_and_resets();
    test_notifications();
    expect_u32(test_other_axis_calls, 0U, "the generator drives only X");

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Step generator task unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Step generator task unit tests passed.\n");
    return 0;
}

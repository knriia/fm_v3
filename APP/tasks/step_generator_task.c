#include "step_generator_task.h"

#include "motion/plan_buffer.h"
#include "motion/step_stream_feeder.h"
#include "motion_axis_hardware.h"
#include "step_stream_axis.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include <stddef.h>

/* X is the only axis with a planned stream so far (TIM4 CH1 and DMA1 stream 0 in step_stream_axis). */
#define STEP_GENERATOR_TASK_AXIS MOTION_AXIS_X
#define STEP_GENERATOR_TASK_PLANNER_AXIS_INDEX 0U
/* The timer idles this long before the DMA delivers the first record of a stream. */
#define STEP_GENERATOR_TASK_STARTUP_PERIOD_NS STEP_STREAM_AXIS_MIN_STARTUP_NS
/* A record is never shorter than this: the DMA must finish a burst before the next timer update. */
#define STEP_GENERATOR_TASK_MIN_RECORD_NS STEP_STREAM_AXIS_MIN_STARTUP_NS
/* Filler record of an empty stretch. A half of them takes about 14 ms, which is how long a new block can wait behind
 * an idle half; shorter records would only load the DMA interrupt more. */
#define STEP_GENERATOR_TASK_IDLE_RECORD_NS 10000U
/* The braking ramp starts at this step rate: the fast seek of homing. A faster axis is brought down to it at once. */
#define STEP_GENERATOR_TASK_BRAKE_MAX_RATE_HZ 50000U
/* DIR setup before the first STEP is at least this long; the shortest RTOS delay covers it. */
#define STEP_GENERATOR_TASK_DIRECTION_SETTLE_MS 1U

#define STEP_GENERATOR_TASK_FLAG_BLOCK_READY (1UL << 0)
#define STEP_GENERATOR_TASK_FLAG_STOP_RESET (1UL << 1)
#define STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED (1UL << 2)
#define STEP_GENERATOR_TASK_FLAGS_ALL                                                                                  \
    (STEP_GENERATOR_TASK_FLAG_BLOCK_READY | STEP_GENERATOR_TASK_FLAG_STOP_RESET |                                      \
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED)

static volatile StepGeneratorTaskDiagnostics step_generator_task_diagnostics;
/* Owned by the task: only StepGeneratorTask touches it. */
static StepStreamFeeder_t step_generator_task_feeder;
/* Written once at task start and read by MotionTask and the DMA interrupt to wake the task. */
static osThreadId_t volatile step_generator_task_thread_id;
/* From the moment a block leaves the plan buffer until the stream has ended; STOP resets it. */
static volatile uint8_t step_generator_task_stream_busy;
/* Set by STOP in another task or by a ring fault in the DMA interrupt; cleared by this task once it is reset. */
static volatile uint8_t step_generator_task_halted;
/* Set once at start when the axis refused the braking ramp; counted into generator_errors with the feeder's errors. */
static volatile uint8_t step_generator_task_brake_setup_failed;

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

void step_generator_task_halt(void) {
    step_generator_task_halted = 1U;
    /* The axis stops itself even if the task is in the middle of a fill: the start port checks the flag with task
     * switching paused, so a stream cannot begin after this stop. */
    step_stream_axis_stop();
}

uint8_t step_generator_task_is_busy(void) {
    PlanBufferDiagnostics_t plan = {0};
    /* Only a null pointer makes this fail, and the zeroed struct then reads as an empty plan. */
    (void)plan_buffer_get_diagnostics(&plan);
    return (uint8_t)(step_generator_task_stream_busy != 0U || plan.queued != 0U);
}

uint8_t step_generator_task_is_halted(void) { return step_generator_task_halted; }

void step_generator_task_resume_after_stop(void) { step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_STOP_RESET); }

/* The ports below adapt the feeder to the stream axis and the plan buffer. */

static StepStreamFeederAxisResult_t step_generator_task_port_result(StepStreamAxisStatus_t status) {
    return status == STEP_STREAM_AXIS_STATUS_OK ? STEP_STREAM_FEEDER_AXIS_OK : STEP_STREAM_FEEDER_AXIS_ERROR;
}

static StepStreamFeederAxisResult_t step_generator_task_port_prepare(void) {
    return step_generator_task_port_result(step_stream_axis_prepare_stream());
}

static StepStreamFeederAxisResult_t step_generator_task_port_acquire_half(uint8_t *half, uint16_t **records) {
    const StepStreamAxisStatus_t status = step_stream_axis_acquire_half(half, records);
    if (status == STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST) {
        return STEP_STREAM_FEEDER_AXIS_NO_REQUEST;
    }
    return step_generator_task_port_result(status);
}

static StepStreamFeederAxisResult_t step_generator_task_port_commit_half(uint8_t half) {
    return step_generator_task_port_result(step_stream_axis_commit_half(half));
}

static StepStreamFeederAxisResult_t step_generator_task_port_set_direction(uint8_t positive) {
    /* With task switching paused, STOP and MotionTask cannot touch the axis between the flag check and the write. */
    MotionAxisHardwareStatus_t status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    (void)osKernelLock();
    if (step_generator_task_halted == 0U) {
        status = motion_axis_hardware_set_stream_outputs(STEP_GENERATOR_TASK_AXIS, positive);
    }
    (void)osKernelUnlock();
    /* A refusal returns the status of a busy or faulted axis; a write leaves a status the axis can start from. */
    if (status != MOTION_AXIS_HARDWARE_STATUS_READY && status != MOTION_AXIS_HARDWARE_STATUS_COMPLETE &&
        status != MOTION_AXIS_HARDWARE_STATUS_STOPPED) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    (void)osDelay(STEP_GENERATOR_TASK_DIRECTION_SETTLE_MS);
    return STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t step_generator_task_port_start(uint32_t startup_period_ns) {
    StepStreamAxisStatus_t status = STEP_STREAM_AXIS_STATUS_BUSY;
    (void)osKernelLock();
    if (step_generator_task_halted == 0U) {
        status = step_stream_axis_start_stream(startup_period_ns);
    }
    (void)osKernelUnlock();
    return step_generator_task_port_result(status);
}

static void step_generator_task_port_stop(void) { step_stream_axis_stop(); }

static uint8_t step_generator_task_port_pop_block(MotionPlanBlock_t *block) {
    if (plan_buffer_try_pop(block) != PLAN_BUFFER_STATUS_OK) {
        return 0U;
    }
    /* Set before the block is out of sight, so HOME never sees an empty plan buffer and an idle stream in between. */
    step_generator_task_stream_busy = 1U;
    return 1U;
}

/* DMA interrupt: the stream axis has stopped itself after a ring fault, so stop feeding until STOP resets us. */
static void step_generator_task_stream_fault(StepStreamAxisFault_t fault) {
    (void)fault;
    step_generator_task_halted = 1U;
}

/* DMA interrupt: the axis brakes on a limit switch and refills its ring itself, so the feeder must not write. */
static void step_generator_task_stop_feeding_for_brake(void) { step_generator_task_halted = 1U; }

static void step_generator_task_publish_diagnostics(void) {
    StepStreamFeederDiagnostics_t feeder_diagnostics;
    step_stream_feeder_get_diagnostics(&step_generator_task_feeder, &feeder_diagnostics);
    step_generator_task_diagnostics.blocks_started = feeder_diagnostics.blocks_started;
    step_generator_task_diagnostics.blocks_completed = feeder_diagnostics.blocks_completed;
    step_generator_task_diagnostics.halves_filled = feeder_diagnostics.halves_filled;
    step_generator_task_diagnostics.generator_errors =
        step_generator_task_brake_setup_failed != 0U && feeder_diagnostics.generator_errors != UINT32_MAX
            ? feeder_diagnostics.generator_errors + 1U
            : feeder_diagnostics.generator_errors;
    step_generator_task_diagnostics.axis_errors = feeder_diagnostics.axis_errors;
    step_generator_task_diagnostics.streams_started = feeder_diagnostics.streams_started;
    step_generator_task_diagnostics.streams_stopped = feeder_diagnostics.streams_stopped;
}

/* Builds the braking ramp of the axis from the same timing as the feeder. Without a deceleration setting, or with
 * one the ramp cannot be built from, the axis keeps no ramp and stops at once on a limit switch; the latter is counted
 * in generator_errors. */
static void step_generator_task_setup_brake(
    const StepGeneratorTaskConfig_t *task_config,
    const StepStreamFeederConfig_t *feeder_config
) {
    if (task_config->limit_deceleration_steps_per_s2 == 0U) {
        return;
    }
    const uint64_t pulse_and_low = (uint64_t)feeder_config->generator.pulse_ticks + feeder_config->generator.low_ticks;
    const StepStreamBrakeConfig_t brake = {
        .timer_clock_hz = feeder_config->generator.timer_clock_hz,
        .pulse_ticks = feeder_config->generator.pulse_ticks,
        .min_period_ticks = feeder_config->generator.min_record_ticks > pulse_and_low
                                ? feeder_config->generator.min_record_ticks
                                : (uint32_t)pulse_and_low,
        .idle_record_ticks = feeder_config->idle_record_ticks,
        .deceleration_steps_per_s2 = task_config->limit_deceleration_steps_per_s2,
        .max_step_rate_hz = STEP_GENERATOR_TASK_BRAKE_MAX_RATE_HZ,
    };
    if (step_stream_axis_configure_brake(&brake) != STEP_STREAM_AXIS_STATUS_OK) {
        step_generator_task_brake_setup_failed = 1U;
    }
}

/* Builds the feeder from the driver timing. Returns 0 when the clock or the timing cannot be used. */
static uint8_t step_generator_task_setup_feeder(const StepGeneratorTaskConfig_t *task_config) {
    if (task_config == NULL) {
        return 0U;
    }
    StepStreamFeederConfig_t config;
    if (step_stream_feeder_make_config(
            step_stream_axis_get_timer_clock_hz(),
            task_config->step_pulse_width_ns,
            task_config->step_low_time_ns,
            STEP_GENERATOR_TASK_MIN_RECORD_NS,
            STEP_GENERATOR_TASK_IDLE_RECORD_NS,
            STEP_GENERATOR_TASK_STARTUP_PERIOD_NS,
            STEP_STREAM_AXIS_HALF_RECORD_COUNT,
            STEP_GENERATOR_TASK_PLANNER_AXIS_INDEX,
            &config
        ) != STEP_STREAM_FEEDER_STATUS_OK) {
        return 0U;
    }
    static const StepStreamFeederPorts_t ports = {
        step_generator_task_port_prepare,
        step_generator_task_port_acquire_half,
        step_generator_task_port_commit_half,
        step_generator_task_port_set_direction,
        step_generator_task_port_start,
        step_generator_task_port_stop,
        step_generator_task_port_pop_block,
    };
    /* The ramp depends on the config only, so it is set up whether or not the feeder takes it. */
    step_generator_task_setup_brake(task_config, &config);
    return (uint8_t)(step_stream_feeder_init(&step_generator_task_feeder, &config, &ports) ==
                     STEP_STREAM_FEEDER_STATUS_OK);
}

void StepGeneratorTask(void *argument) {
    step_generator_task_thread_id = osThreadGetId();
    const uint8_t feeder_ready = step_generator_task_setup_feeder(argument);
    if (feeder_ready == 0U) {
        /* No usable timing: the task cannot feed the axis, so it refuses all work instead of guessing. */
        step_generator_counter_increment(&step_generator_task_diagnostics.generator_errors);
        step_generator_task_halted = 1U;
    }
    static const StepStreamAxisListener_t stream_listener = {
        step_generator_task_notify_ring_half_released,
        step_generator_task_stream_fault,
        step_generator_task_stop_feeding_for_brake,
    };
    (void)step_stream_axis_set_listener(&stream_listener);

    uint32_t flags = 0U;
    for (;;) {
        if (feeder_ready != 0U && (flags & STEP_GENERATOR_TASK_FLAG_STOP_RESET) != 0U) {
            /* The axis has stopped and this task is not filling, so the feeder owns no running stream. */
            /* Lift the halt first: a STOP that arrives during the reset then stays in force. */
            step_generator_task_halted = 0U;
            step_stream_feeder_reset(&step_generator_task_feeder);
            step_generator_task_stream_busy = 0U;
            step_generator_counter_increment(&step_generator_task_diagnostics.stop_resets);
        }
        if (feeder_ready != 0U && step_generator_task_halted == 0U) {
            if (step_stream_feeder_service(&step_generator_task_feeder) != STEP_STREAM_FEEDER_STATUS_OK) {
                /* The feeder has stopped the axis; the plan is untrustworthy until STOP. */
                step_generator_task_halted = 1U;
            }
            step_generator_task_publish_diagnostics();
            if (step_stream_feeder_get_phase(&step_generator_task_feeder) == STEP_STREAM_FEEDER_PHASE_IDLE) {
                step_generator_task_stream_busy = 0U;
            }
        }
        /* Flags set while filling stay pending, so a block or a released half that arrives meanwhile is not missed. */
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
    diagnostics->halves_filled = step_generator_task_diagnostics.halves_filled;
    diagnostics->generator_errors = step_generator_task_diagnostics.generator_errors;
    diagnostics->axis_errors = step_generator_task_diagnostics.axis_errors;
    diagnostics->streams_started = step_generator_task_diagnostics.streams_started;
    diagnostics->streams_stopped = step_generator_task_diagnostics.streams_stopped;
    diagnostics->stop_resets = step_generator_task_diagnostics.stop_resets;
}

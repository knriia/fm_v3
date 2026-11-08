#include "motion/plan_buffer.h"
#include "motion_axis_hardware.h"
#include "step_generator_task.h"
#include "step_stream_axis.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_THREAD_ID ((osThreadId_t)(uintptr_t)0x5678U)
#define TEST_WAIT_SCRIPT_LENGTH 4U
#define TEST_TIMER_CLOCK_HZ 275000000U
#define TEST_HALF_WORDS (STEP_STREAM_AXIS_HALF_RECORD_COUNT * 3U)

typedef enum {
    TEST_ACTION_NONE = 0,
    TEST_ACTION_RELEASE_HALF_0,
    TEST_ACTION_RELEASE_HALF_1,
    TEST_ACTION_PUSH_BLOCK
} TestWaitAction_t;

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
static TestWaitAction_t test_wait_actions[TEST_WAIT_SCRIPT_LENGTH];
static uint32_t test_wait_script_count;
static uint32_t test_wait_last_timeout;
static uint32_t test_kernel_lock_depth;
static uint32_t test_delay_calls;

/* Fake X stream axis: two halves that are EMPTY after prepare. */
static uint16_t test_halves[2][TEST_HALF_WORDS];
static uint8_t test_half_empty[2];
static uint32_t test_prepare_calls;
static uint32_t test_start_calls;
static uint32_t test_last_startup_ns;
static uint32_t test_stop_calls;
static uint32_t test_listener_calls;
static StepStreamAxisListener_t test_stream_listener;
static uint32_t test_timer_clock_hz;
static uint32_t test_outputs_calls;
static uint8_t test_outputs_positive;
static MotionAxisHardwareStatus_t test_outputs_result;
static uint8_t test_fail_prepare;
static uint8_t test_fail_acquire;
static uint8_t test_fail_commit;
static uint8_t test_fail_start;
static uint32_t test_other_axis_calls;
static uint32_t test_configure_brake_calls;
static StepStreamBrakeConfig_t test_brake_config;
static StepStreamAxisStatus_t test_configure_brake_result;

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

/* 100 kHz step rate on X: one 10 us step, 2750 ticks at the 275 MHz test clock. */
static MotionPlanBlock_t make_x_block(uint32_t sequence, int64_t steps) {
    const uint64_t magnitude = (uint64_t)(steps < 0 ? -steps : steps);
    return (MotionPlanBlock_t){
        .sequence = sequence,
        .step_delta = {steps, 0, 0},
        .path_length_um = magnitude,
        .master_step_count = magnitude,
        .step_event_rate_numerator = 100000U,
        .step_event_rate_denominator = 1U,
        .feedrate_um_per_s = 1000U,
        .moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK,
        .positive_direction_mask = steps > 0 ? MOTION_PLANNER_AXIS_X_MASK : 0U,
    };
}

static void push_block(uint32_t sequence, int64_t steps) {
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
    ++test_kernel_lock_depth;
    return 0;
}

int32_t osKernelUnlock(void) {
    --test_kernel_lock_depth;
    return 1;
}

uint32_t osDelay(uint32_t milliseconds) {
    (void)milliseconds;
    ++test_delay_calls;
    return osOK;
}

uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags) {
    ++test_flags_set_calls;
    test_flags_set_target = thread_id;
    test_flags_set_value |= flags;
    return test_flags_set_value;
}

uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout) {
    (void)flags;
    (void)options;
    test_wait_last_timeout = timeout;
    const uint32_t index = test_wait_calls;
    ++test_wait_calls;
    if (index >= test_wait_script_count) {
        longjmp(test_jump_buffer, 2);
    }
    switch (test_wait_actions[index]) {
    case TEST_ACTION_RELEASE_HALF_0:
        test_half_empty[0] = 1U;
        break;
    case TEST_ACTION_RELEASE_HALF_1:
        test_half_empty[1] = 1U;
        break;
    case TEST_ACTION_PUSH_BLOCK:
        push_block(300U + index, 2);
        break;
    case TEST_ACTION_NONE:
    default:
        break;
    }
    return test_wait_results[index];
}

StepStreamAxisStatus_t step_stream_axis_set_listener(const StepStreamAxisListener_t *listener) {
    ++test_listener_calls;
    test_stream_listener = *listener;
    return STEP_STREAM_AXIS_STATUS_OK;
}

uint32_t step_stream_axis_get_timer_clock_hz(void) { return test_timer_clock_hz; }

StepStreamAxisStatus_t step_stream_axis_prepare_stream(void) {
    ++test_prepare_calls;
    if (test_fail_prepare != 0U) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    test_half_empty[0] = 1U;
    test_half_empty[1] = 1U;
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_acquire_half(uint8_t *half, uint16_t **records) {
    if (test_fail_acquire != 0U) {
        return STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
    }
    for (uint8_t index = 0U; index < 2U; ++index) {
        if (test_half_empty[index] != 0U) {
            *half = index;
            *records = test_halves[index];
            return STEP_STREAM_AXIS_STATUS_OK;
        }
    }
    return STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST;
}

StepStreamAxisStatus_t step_stream_axis_commit_half(uint8_t half) {
    if (test_fail_commit != 0U) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    test_half_empty[half] = 0U;
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_start_stream(uint32_t startup_period_ns) {
    ++test_start_calls;
    test_last_startup_ns = startup_period_ns;
    return test_fail_start != 0U ? STEP_STREAM_AXIS_STATUS_AXIS_BUSY : STEP_STREAM_AXIS_STATUS_OK;
}

void step_stream_axis_stop(void) { ++test_stop_calls; }

StepStreamAxisStatus_t step_stream_axis_configure_brake(const StepStreamBrakeConfig_t *config) {
    ++test_configure_brake_calls;
    test_brake_config = *config;
    return test_configure_brake_result;
}

MotionAxisHardwareStatus_t motion_axis_hardware_set_stream_outputs(MotionAxis_t axis, uint8_t positive_direction) {
    if (axis != MOTION_AXIS_X) {
        ++test_other_axis_calls;
    }
    ++test_outputs_calls;
    test_outputs_positive = positive_direction;
    return test_outputs_result;
}

#define static
#include "../APP/tasks/step_generator_task.c"
#undef static

static StepGeneratorTaskConfig_t test_task_config;

static void reset_fixture(void) {
    plan_buffer_test_reset();
    expect_u32(plan_buffer_init(), PLAN_BUFFER_STATUS_OK, "plan buffer initializes");
    step_generator_task_diagnostics = (StepGeneratorTaskDiagnostics){0};
    memset(&step_generator_task_feeder, 0, sizeof(step_generator_task_feeder));
    step_generator_task_thread_id = NULL;
    step_generator_task_halted = 0U;
    step_generator_task_brake_setup_failed = 0U;
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_flags_set_calls = 0U;
    test_flags_set_value = 0U;
    test_flags_set_target = NULL;
    test_wait_calls = 0U;
    test_wait_script_count = 0U;
    test_wait_last_timeout = 0U;
    test_kernel_lock_depth = 0U;
    test_delay_calls = 0U;
    step_generator_task_stream_busy = 0U;
    memset(test_halves, 0, sizeof(test_halves));
    memset(test_half_empty, 0, sizeof(test_half_empty));
    test_prepare_calls = 0U;
    test_start_calls = 0U;
    test_last_startup_ns = 0U;
    test_stop_calls = 0U;
    test_listener_calls = 0U;
    test_timer_clock_hz = TEST_TIMER_CLOCK_HZ;
    test_outputs_calls = 0U;
    test_outputs_positive = 0xFFU;
    test_outputs_result = MOTION_AXIS_HARDWARE_STATUS_READY;
    test_fail_prepare = 0U;
    test_fail_acquire = 0U;
    test_fail_commit = 0U;
    test_fail_start = 0U;
    test_configure_brake_calls = 0U;
    test_brake_config = (StepStreamBrakeConfig_t){0};
    test_configure_brake_result = STEP_STREAM_AXIS_STATUS_OK;
    test_task_config = (StepGeneratorTaskConfig_t){
        .step_pulse_width_ns = 5000U,
        .step_low_time_ns = 2500U,
        .limit_deceleration_steps_per_s2 = 600000U,
    };
    for (uint32_t index = 0U; index < TEST_WAIT_SCRIPT_LENGTH; ++index) {
        test_wait_results[index] = 0U;
        test_wait_actions[index] = TEST_ACTION_NONE;
    }
}

static int run_task_until_jump(const StepGeneratorTaskConfig_t *config) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        StepGeneratorTask((void *)config);
    }
    test_jump_active = 0U;
    return jumped;
}

static void test_task_streams_a_block(void) {
    reset_fixture();
    push_block(1U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_true(step_generator_task_thread_id == TEST_THREAD_ID, "task stores its id for wakeups");
    expect_u32(test_wait_calls, 2U, "task waits again after each service");
    expect_u32(test_wait_last_timeout, osWaitForever, "task waits for work without polling");
    expect_u32(test_listener_calls, 1U, "task registers its stream listener once");

    expect_u32(test_prepare_calls, 1U, "stream is prepared");
    expect_u32(test_outputs_calls, 1U, "driver outputs are set once");
    expect_u32(test_outputs_positive, 1U, "positive block sets the positive direction");
    expect_u32(test_delay_calls, 1U, "DIR setup time is waited");
    expect_u32(test_start_calls, 1U, "stream starts once both halves are filled");
    expect_u32(test_last_startup_ns, STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed after the start");
    expect_u32(test_half_empty[0] + test_half_empty[1], 0U, "both halves are committed");
    expect_u32(test_halves[0][0], 2749U, "step interval is 10 us in ticks");
    expect_u32(test_halves[0][2], 1375U, "pulse is 5 us in ticks");

    StepGeneratorTaskDiagnostics diagnostics = {0};
    step_generator_task_get_diagnostics(&diagnostics);
    step_generator_task_get_diagnostics(NULL);
    expect_u32(diagnostics.blocks_started, 1U, "started blocks are counted");
    expect_u32(diagnostics.blocks_completed, 1U, "completed blocks are counted");
    expect_u32(diagnostics.halves_filled, 2U, "both halves are counted");
    expect_u32(diagnostics.generator_errors, 0U, "valid block produces no error");
    expect_u32(diagnostics.axis_errors, 0U, "axis reports no error");
    expect_u32(diagnostics.streams_started, 1U, "stream start is counted");
    expect_u32(diagnostics.streams_stopped, 0U, "stream is still running");
    expect_u32(diagnostics.stop_resets, 0U, "no STOP yet");

    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = osFlagsErrorParameter;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 1U, "wait error stops the task");
    expect_u32(test_error_handler_calls, 1U, "wait error is fatal");
}

static void test_idle_stream_stops(void) {
    reset_fixture();
    push_block(2U, 3);
    test_wait_script_count = 2U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[0] = TEST_ACTION_RELEASE_HALF_0;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[1] = TEST_ACTION_RELEASE_HALF_1;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(test_stop_calls, 1U, "an idle stream is stopped by the task");
    expect_u32(step_generator_task_diagnostics.streams_stopped, 1U, "stream stop is counted");
    expect_u32(step_generator_task_halted, 0U, "a normal end does not halt the task");

    /* A block that arrives later opens a new stream. */
    reset_fixture();
    push_block(3U, 3);
    test_wait_script_count = 4U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[0] = TEST_ACTION_RELEASE_HALF_0;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[1] = TEST_ACTION_RELEASE_HALF_1;
    test_wait_results[2] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    test_wait_actions[2] = TEST_ACTION_PUSH_BLOCK;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(test_start_calls, 2U, "the next block starts a second stream");
}

static void test_unusable_timing_disables_feeding(void) {
    reset_fixture();
    push_block(4U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task without a config only waits");
    expect_u32(test_prepare_calls, 0U, "no stream without a config");
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "missing config is counted");
    expect_u32(step_generator_task_halted, 1U, "task is halted without a config");
    PlanBufferDiagnostics_t plan_diagnostics;
    (void)plan_buffer_get_diagnostics(&plan_diagnostics);
    expect_u32(plan_diagnostics.queued, 1U, "the block stays in the plan buffer");

    /* A STOP reset does not revive a task that has no feeder. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_diagnostics.stop_resets, 0U, "no reset without a feeder");
    expect_u32(step_generator_task_halted, 1U, "still halted");

    reset_fixture();
    test_timer_clock_hz = 0U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "unusable clock is counted");

    reset_fixture();
    test_task_config.step_pulse_width_ns = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "zero pulse width is counted");

    /* Timing whose tick count does not fit 32 bits is refused when the feeder config is built. */
    reset_fixture();
    test_task_config.step_low_time_ns = UINT32_MAX;
    test_timer_clock_hz = UINT32_MAX;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "overflowing timing is counted");
}

static void test_stop_halts_and_resets(void) {
    reset_fixture();
    push_block(5U, 3);
    expect_u32(step_generator_task_is_halted(), 0U, "a fresh task is not halted");
    step_generator_task_halt();
    expect_u32(test_stop_calls, 1U, "STOP stops the stream axis at once");
    expect_u32(step_generator_task_halted, 1U, "STOP halts feeding");
    expect_u32(step_generator_task_is_halted(), 1U, "the halt is visible to MotionTask");
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "halted task only waits");
    expect_u32(test_prepare_calls, 0U, "halted task prepares nothing");
    expect_u32(test_start_calls, 0U, "halted task starts nothing");

    reset_fixture();
    push_block(6U, 3);
    step_generator_task_halt();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task resets on the STOP reset flag");
    expect_u32(step_generator_task_halted, 0U, "reset lifts the halt");
    expect_u32(step_generator_task_diagnostics.stop_resets, 1U, "reset is counted");
    expect_u32(test_start_calls, 1U, "feeding goes on after the reset");

    reset_fixture();
    step_generator_task_resume_after_stop();
    expect_u32(test_flags_set_calls, 0U, "resume before the task starts is ignored");
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_task_resume_after_stop();
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_STOP_RESET, "resume sets the reset flag");
}

static void test_start_is_refused_while_halted(void) {
    reset_fixture();
    step_generator_task_halted = 1U;
    expect_u32(
        step_generator_task_port_start(2000U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "a stream cannot start after STOP"
    );
    expect_u32(test_start_calls, 0U, "the axis is not started while halted");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed");
}

static void test_direction_is_refused_while_halted(void) {
    reset_fixture();
    step_generator_task_halted = 1U;
    expect_u32(
        step_generator_task_port_set_direction(1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver is not touched after STOP"
    );
    expect_u32(test_outputs_calls, 0U, "no DIR or ENA write while halted");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed");
}

static void test_busy_covers_queue_and_stream(void) {
    reset_fixture();
    expect_u32(step_generator_task_is_busy(), 0U, "idle task with an empty plan is not busy");

    push_block(20U, 3);
    expect_u32(step_generator_task_is_busy(), 1U, "a queued block makes the task busy");

    MotionPlanBlock_t popped;
    expect_u32(step_generator_task_port_pop_block(&popped), 1U, "block is taken");
    expect_u32(step_generator_task_is_busy(), 1U, "a block taken from the plan keeps the task busy");
    expect_u32(step_generator_task_port_pop_block(&popped), 0U, "empty plan gives no block");

    /* A finished stream clears the flag; a fault keeps it until STOP resets the task. */
    reset_fixture();
    push_block(21U, 3);
    test_wait_script_count = 2U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[0] = TEST_ACTION_RELEASE_HALF_0;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[1] = TEST_ACTION_RELEASE_HALF_1;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_is_busy(), 0U, "an ended stream is not busy");

    reset_fixture();
    push_block(22U, 3);
    test_fail_start = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_is_busy(), 1U, "a faulted stream stays busy until STOP");

    reset_fixture();
    step_generator_task_stream_busy = 1U;
    step_generator_task_halted = 1U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_is_busy(), 0U, "STOP reset clears the busy flag");
}

static void test_faults_halt_the_task(void) {
    reset_fixture();
    push_block(7U, 3);
    test_fail_start = 1U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_halted, 1U, "a refused start halts feeding");
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "axis error is counted");
    expect_u32(step_generator_task_diagnostics.streams_started, 0U, "no stream started");
    expect_true(test_stop_calls >= 1U, "the feeder stops the axis on a fault");

    reset_fixture();
    push_block(8U, 3);
    test_outputs_result = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "a busy driver refuses the direction");
    expect_u32(test_delay_calls, 0U, "no settle wait after a refusal");

    const MotionAxisHardwareStatus_t usable[] = {
        MOTION_AXIS_HARDWARE_STATUS_COMPLETE,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
    };
    for (uint32_t index = 0U; index < sizeof(usable) / sizeof(usable[0]); ++index) {
        reset_fixture();
        push_block(9U, -3);
        test_outputs_result = usable[index];
        (void)run_task_until_jump(&test_task_config);
        expect_u32(test_start_calls, 1U, "a completed or stopped axis can start a stream");
        expect_u32(test_outputs_positive, 0U, "negative block sets the negative direction");
    }

    reset_fixture();
    push_block(10U, 3);
    test_fail_prepare = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "prepare failure is an axis error");

    reset_fixture();
    push_block(11U, 3);
    test_fail_acquire = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "acquire failure is an axis error");

    reset_fixture();
    push_block(12U, 3);
    test_fail_commit = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "commit failure is an axis error");

    reset_fixture();
    MotionPlanBlock_t invalid = make_x_block(13U, 3);
    invalid.master_step_count = 0U;
    expect_u32(plan_buffer_try_push(&invalid), PLAN_BUFFER_STATUS_OK, "invalid block is queued");
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "a rejected block is a generator error");
    expect_u32(step_generator_task_halted, 1U, "a rejected block halts feeding");
    expect_u32(test_start_calls, 0U, "nothing starts after a rejected block");
}

static void test_listener_and_notifications(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);

    test_flags_set_value = 0U;
    test_stream_listener.on_half_released();
    expect_u32(
        test_flags_set_value,
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED,
        "a released ring half wakes the task"
    );
    expect_u32(step_generator_task_halted, 0U, "a released half does not halt the task");
    test_stream_listener.on_fault(STEP_STREAM_AXIS_FAULT_UNDERRUN);
    expect_u32(step_generator_task_halted, 1U, "a stream fault halts feeding");

    step_generator_task_halted = 0U;
    test_stream_listener.on_brake();
    expect_u32(step_generator_task_halted, 1U, "braking on a limit halts feeding");

    reset_fixture();
    step_generator_task_notify_block_ready();
    step_generator_task_notify_ring_half_released();
    expect_u32(test_flags_set_calls, 0U, "wakeups before the task starts are ignored");

    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_task_notify_block_ready();
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_BLOCK_READY, "new block sets its flag");
    expect_true(test_flags_set_target == TEST_THREAD_ID, "flag goes to the generator task");

    volatile uint32_t counter = UINT32_MAX;
    step_generator_counter_increment(&counter);
    expect_u32(counter, UINT32_MAX, "diagnostic counter saturates");
}

static void test_brake_ramp_is_built_from_the_driver_timing(void) {
    reset_fixture();
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_configure_brake_calls, 1U, "the braking ramp is configured once");
    expect_u32(test_brake_config.timer_clock_hz, TEST_TIMER_CLOCK_HZ, "ramp uses the timer clock");
    expect_u32(test_brake_config.pulse_ticks, 1375U, "ramp uses the STEP pulse of the driver");
    expect_u32(test_brake_config.min_period_ticks, 2063U, "ramp keeps pulse and low time");
    expect_u32(test_brake_config.idle_record_ticks, 2750U, "ramp uses the filler record of the feeder");
    expect_u32(test_brake_config.deceleration_steps_per_s2, 600000U, "ramp uses the limit deceleration");
    expect_u32(test_brake_config.max_step_rate_hz, STEP_GENERATOR_TASK_BRAKE_MAX_RATE_HZ, "ramp starts at the seek rate");
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "a good ramp is no error");

    /* A longer shortest record than pulse plus low time wins. */
    reset_fixture();
    test_task_config.step_pulse_width_ns = 400U;
    test_task_config.step_low_time_ns = 400U;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_brake_config.min_period_ticks, 550U, "the shortest record sets the ramp period");
    expect_u32(test_brake_config.pulse_ticks, 110U, "ramp pulse follows the driver");
}

static void test_brake_is_skipped_or_counted(void) {
    reset_fixture();
    test_task_config.limit_deceleration_steps_per_s2 = 0U;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_configure_brake_calls, 0U, "no deceleration setting, no ramp");
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "a missing ramp is not an error here");

    reset_fixture();
    test_configure_brake_result = STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "a ramp the axis refuses is counted");
    expect_u32(step_generator_task_halted, 0U, "feeding goes on without a ramp");

    /* The error count adds to the feeder's own and saturates with it. */
    step_generator_task_feeder.diagnostics.generator_errors = 4U;
    step_generator_task_publish_diagnostics();
    expect_u32(step_generator_task_diagnostics.generator_errors, 5U, "ramp error adds to the feeder errors");
    step_generator_task_feeder.diagnostics.generator_errors = UINT32_MAX;
    step_generator_task_publish_diagnostics();
    expect_u32(step_generator_task_diagnostics.generator_errors, UINT32_MAX, "the sum saturates");

    /* An unusable timing builds no feeder, so no ramp either. */
    reset_fixture();
    test_timer_clock_hz = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_configure_brake_calls, 0U, "no feeder, no ramp");
}

int main(void) {
    test_task_streams_a_block();
    test_idle_stream_stops();
    test_unusable_timing_disables_feeding();
    test_stop_halts_and_resets();
    test_start_is_refused_while_halted();
    test_direction_is_refused_while_halted();
    test_busy_covers_queue_and_stream();
    test_faults_halt_the_task();
    test_listener_and_notifications();
    test_brake_ramp_is_built_from_the_driver_timing();
    test_brake_is_skipped_or_counted();
    expect_u32(test_other_axis_calls, 0U, "the generator drives only X");

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Step generator task unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Step generator task unit tests passed.\n");
    return 0;
}

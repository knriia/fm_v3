#include "motion/plan_buffer.h"
#include "motion/step_stream_group.h"
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
/* The generator drives three stream channels: channel 0 is X, channel 1 is Y, channel 2 is Z. */
#define TEST_CHANNELS 3U
#define TEST_CH_X 0U
#define TEST_CH_Y 1U
#define TEST_CH_Z 2U

/* Defined by the task source included below, where `static` is switched off for the tests. */
extern StepStreamGroup_t step_generator_task_group;

typedef enum {
    TEST_ACTION_NONE = 0,
    TEST_ACTION_RELEASE_HALF_0,
    TEST_ACTION_RELEASE_HALF_1,
    TEST_ACTION_PUSH_BLOCK,
    /* The group runs a stream that then ends by itself. */
    TEST_ACTION_RUN_GROUP_THEN_END,
    /* The channel of Y still runs when the task is asked to reset. */
    TEST_ACTION_Y_STREAMING,
    /* The channel of Z still runs when the task is asked to reset. */
    TEST_ACTION_Z_STREAMING,
    /* The channel of Y fails while the group runs, then STOP comes; the fault must survive the STOP. */
    TEST_ACTION_FAULT_THEN_STOP
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

/* Fake stream channels of X and Y: two halves each, EMPTY after prepare. A channel is told apart by its marker. */
static int test_axis_markers[TEST_CHANNELS];
static uint16_t test_halves[TEST_CHANNELS][2][TEST_HALF_WORDS];
static uint8_t test_half_empty[TEST_CHANNELS][2];
static uint32_t test_prepare_calls[TEST_CHANNELS];
static uint32_t test_start_calls[TEST_CHANNELS];
static uint32_t test_last_startup_ns[TEST_CHANNELS];
static uint32_t test_stop_calls[TEST_CHANNELS];
static uint32_t test_listener_calls[TEST_CHANNELS];
static StepStreamAxisListener_t test_stream_listener[TEST_CHANNELS];
static uint32_t test_timer_clock_hz[TEST_CHANNELS];
static uint8_t test_axis_streaming[TEST_CHANNELS];
static uint32_t test_brake_requests[TEST_CHANNELS];
static uint32_t test_configure_brake_calls[TEST_CHANNELS];
static uint32_t test_trigger_calls;
static uint32_t test_trigger_axes;
/* Driver outputs, by motion axis. */
static uint32_t test_outputs_calls[MOTION_AXIS_HARDWARE_COUNT];
static uint8_t test_outputs_positive[MOTION_AXIS_HARDWARE_COUNT];
static MotionAxisHardwareStatus_t test_outputs_result;
static uint8_t test_fail_prepare;
static uint8_t test_fail_acquire;
static uint8_t test_fail_commit;
/* Channel whose arming fails, or TEST_CHANNELS for none. */
static uint8_t test_fail_start_channel;
static MotionAxisHardwareStreamLimitListener_t test_limit_listener;
static void *test_limit_context;
static uint32_t test_limit_listener_calls;
/* A call that carries any other pointer than a channel marker is counted. */
static uint32_t test_wrong_axis_calls;
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

/* One plan block: X, Y and Z step counts (signed), at 100 kHz on the leading axis: one 10 us step, 2750 ticks at the
 * 275 MHz test clock. */
static MotionPlanBlock_t make_block_xyz(uint32_t sequence, int64_t x_steps, int64_t y_steps, int64_t z_steps) {
    const uint64_t x_magnitude = (uint64_t)(x_steps < 0 ? -x_steps : x_steps);
    const uint64_t y_magnitude = (uint64_t)(y_steps < 0 ? -y_steps : y_steps);
    const uint64_t z_magnitude = (uint64_t)(z_steps < 0 ? -z_steps : z_steps);
    uint64_t master = x_magnitude > y_magnitude ? x_magnitude : y_magnitude;
    master = master > z_magnitude ? master : z_magnitude;
    uint8_t moving = 0U;
    uint8_t positive = 0U;
    if (x_steps != 0) {
        moving |= MOTION_PLANNER_AXIS_X_MASK;
        positive |= x_steps > 0 ? MOTION_PLANNER_AXIS_X_MASK : 0U;
    }
    if (y_steps != 0) {
        moving |= MOTION_PLANNER_AXIS_Y_MASK;
        positive |= y_steps > 0 ? MOTION_PLANNER_AXIS_Y_MASK : 0U;
    }
    if (z_steps != 0) {
        moving |= MOTION_PLANNER_AXIS_Z_MASK;
        positive |= z_steps > 0 ? MOTION_PLANNER_AXIS_Z_MASK : 0U;
    }
    return (MotionPlanBlock_t){
        .sequence = sequence,
        .step_delta = {x_steps, y_steps, z_steps},
        .path_length_um = master,
        .master_step_count = master,
        .step_event_rate_numerator = 100000U,
        .step_event_rate_denominator = 1U,
        .feedrate_um_per_s = 1000U,
        .moving_axes_mask = moving,
        .positive_direction_mask = positive,
    };
}

static MotionPlanBlock_t make_block(uint32_t sequence, int64_t x_steps, int64_t y_steps) {
    return make_block_xyz(sequence, x_steps, y_steps, 0);
}

static void push_block_xyz(uint32_t sequence, int64_t x_steps, int64_t y_steps, int64_t z_steps) {
    const MotionPlanBlock_t block = make_block_xyz(sequence, x_steps, y_steps, z_steps);
    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "test block is queued");
}

static void push_block_xy(uint32_t sequence, int64_t x_steps, int64_t y_steps) {
    push_block_xyz(sequence, x_steps, y_steps, 0);
}

static void push_block(uint32_t sequence, int64_t x_steps) { push_block_xy(sequence, x_steps, 0); }

static uint32_t channel_index(const StepStreamAxis_t *axis) {
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        if (axis == (const StepStreamAxis_t *)&test_axis_markers[channel]) {
            return channel;
        }
    }
    ++test_wrong_axis_calls;
    return 0U;
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

static void release_half(uint8_t half) {
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        test_half_empty[channel][half] = 1U;
    }
}

static void run_group_prepared_and_started(void) {
    expect_u32(step_stream_group_prepare(&step_generator_task_group), STEP_STREAM_GROUP_STATUS_OK, "group prepares");
    expect_u32(step_stream_group_start(&step_generator_task_group, 2500U), STEP_STREAM_GROUP_STATUS_OK, "group starts");
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
        release_half(0U);
        break;
    case TEST_ACTION_RELEASE_HALF_1:
        release_half(1U);
        break;
    case TEST_ACTION_PUSH_BLOCK:
        push_block(300U + index, 2);
        break;
    case TEST_ACTION_RUN_GROUP_THEN_END:
        run_group_prepared_and_started();
        test_axis_streaming[TEST_CH_X] = 0U;
        test_axis_streaming[TEST_CH_Y] = 0U;
        test_axis_streaming[TEST_CH_Z] = 0U;
        break;
    case TEST_ACTION_Y_STREAMING:
        test_axis_streaming[TEST_CH_Y] = 1U;
        break;
    case TEST_ACTION_Z_STREAMING:
        test_axis_streaming[TEST_CH_Z] = 1U;
        break;
    case TEST_ACTION_FAULT_THEN_STOP:
        run_group_prepared_and_started();
        test_stream_listener[TEST_CH_Y].on_fault(test_stream_listener[TEST_CH_Y].context, STEP_STREAM_AXIS_FAULT_SEQUENCE);
        test_axis_streaming[TEST_CH_X] = 0U;
        test_axis_streaming[TEST_CH_Y] = 0U;
        test_axis_streaming[TEST_CH_Z] = 0U;
        step_generator_task_halt();
        expect_u32(
            step_stream_group_get_state(&step_generator_task_group),
            STEP_STREAM_GROUP_STATE_FAULT,
            "STOP does not clear a fault"
        );
        break;
    case TEST_ACTION_NONE:
    default:
        break;
    }
    return test_wait_results[index];
}

StepStreamAxis_t *step_stream_axis_get(MotionAxis_t axis) {
    if (axis == MOTION_AXIS_X) {
        return (StepStreamAxis_t *)&test_axis_markers[TEST_CH_X];
    }
    if (axis == MOTION_AXIS_Y) {
        return (StepStreamAxis_t *)&test_axis_markers[TEST_CH_Y];
    }
    return axis == MOTION_AXIS_Z ? (StepStreamAxis_t *)&test_axis_markers[TEST_CH_Z] : NULL;
}

StepStreamAxisStatus_t step_stream_axis_set_listener(StepStreamAxis_t *axis, const StepStreamAxisListener_t *listener) {
    const uint32_t channel = channel_index(axis);
    ++test_listener_calls[channel];
    test_stream_listener[channel] = *listener;
    return STEP_STREAM_AXIS_STATUS_OK;
}

uint32_t step_stream_axis_get_timer_clock_hz(const StepStreamAxis_t *axis) {
    return test_timer_clock_hz[channel_index(axis)];
}

/* The axes that the limit interrupt watches; the stream may not start before they are narrowed to the moving ones. */
static uint8_t test_watched_axes = 0xFFU;
static uint32_t test_watched_axes_calls;
static uint8_t test_watched_at_prepare[TEST_CHANNELS];

StepStreamAxisStatus_t step_stream_axis_prepare_stream(StepStreamAxis_t *axis) {
    const uint32_t channel = channel_index(axis);
    ++test_prepare_calls[channel];
    test_watched_at_prepare[channel] = test_watched_axes;
    if (test_fail_prepare != 0U) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    test_half_empty[channel][0] = 1U;
    test_half_empty[channel][1] = 1U;
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_acquire_half(StepStreamAxis_t *axis, uint8_t *half, uint16_t **records) {
    const uint32_t channel = channel_index(axis);
    if (test_fail_acquire != 0U) {
        return STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
    }
    for (uint8_t index = 0U; index < 2U; ++index) {
        if (test_half_empty[channel][index] != 0U) {
            *half = index;
            *records = test_halves[channel][index];
            return STEP_STREAM_AXIS_STATUS_OK;
        }
    }
    return STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST;
}

StepStreamAxisStatus_t step_stream_axis_commit_half(StepStreamAxis_t *axis, uint8_t half) {
    const uint32_t channel = channel_index(axis);
    if (test_fail_commit != 0U) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    test_half_empty[channel][half] = 0U;
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_arm_stream(StepStreamAxis_t *axis, uint32_t startup_period_ns) {
    const uint32_t channel = channel_index(axis);
    ++test_start_calls[channel];
    test_last_startup_ns[channel] = startup_period_ns;
    if (test_fail_start_channel == channel) {
        return STEP_STREAM_AXIS_STATUS_AXIS_BUSY;
    }
    test_axis_streaming[channel] = 1U;
    return STEP_STREAM_AXIS_STATUS_OK;
}

void step_stream_axis_trigger(StepStreamAxis_t *const *axes, uint32_t count) {
    for (uint32_t index = 0U; index < count; ++index) {
        (void)channel_index(axes[index]);
    }
    test_trigger_axes = count;
    ++test_trigger_calls;
}

void step_stream_axis_stop(StepStreamAxis_t *axis) {
    const uint32_t channel = channel_index(axis);
    ++test_stop_calls[channel];
    test_axis_streaming[channel] = 0U;
}

void step_stream_axis_request_brake(StepStreamAxis_t *axis) { ++test_brake_requests[channel_index(axis)]; }

StepStreamAxisState_t step_stream_axis_get_state(const StepStreamAxis_t *axis) {
    return test_axis_streaming[channel_index(axis)] != 0U ? STEP_STREAM_AXIS_STATE_STREAMING
                                                          : STEP_STREAM_AXIS_STATE_IDLE;
}

static StepStreamAxisFault_t test_axis_fault[TEST_CHANNELS];
static MotionAxisHardwareStatus_t test_hardware_status[TEST_CHANNELS];
static MotionAxisHardwareStopReason_t test_hardware_stop_reason[TEST_CHANNELS];

StepStreamAxisFault_t step_stream_axis_get_fault(const StepStreamAxis_t *axis) {
    return test_axis_fault[channel_index(axis)];
}

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) { return test_hardware_status[axis]; }

MotionAxisHardwareStopReason_t motion_axis_hardware_get_last_stop_reason(MotionAxis_t axis) {
    return test_hardware_stop_reason[axis];
}

/* The host has no interrupts to mask; the sections of the group are tested with the fake ports of its own tests. */
uint32_t step_stream_axis_enter_critical(void) { return 0U; }

void step_stream_axis_exit_critical(uint32_t primask) { (void)primask; }

void motion_axis_hardware_set_stream_limit_listener(MotionAxisHardwareStreamLimitListener_t listener, void *context) {
    ++test_limit_listener_calls;
    test_limit_listener = listener;
    test_limit_context = context;
}

void motion_axis_hardware_set_stream_watched_axes(uint8_t axes_mask) {
    test_watched_axes = axes_mask;
    ++test_watched_axes_calls;
}

StepStreamAxisStatus_t step_stream_axis_configure_brake(StepStreamAxis_t *axis, const StepStreamBrakeConfig_t *config) {
    ++test_configure_brake_calls[channel_index(axis)];
    test_brake_config = *config;
    return test_configure_brake_result;
}

MotionAxisHardwareStatus_t motion_axis_hardware_set_stream_outputs(MotionAxis_t axis, uint8_t positive_direction) {
    ++test_outputs_calls[axis];
    test_outputs_positive[axis] = positive_direction;
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
    step_generator_task_halt_cause = STEP_GENERATOR_TASK_HALT_CAUSE_NONE;
    step_generator_task_halt_member = STEP_GENERATOR_TASK_NO_MEMBER;
    step_generator_task_halt_fault = 0U;
    step_generator_task_halts_total = 0U;
    step_generator_task_feeder_status = 0U;
    step_generator_task_reset_failures = 0U;
    memset((void *)step_generator_task_limit_trips, 0, sizeof(step_generator_task_limit_trips));
    test_watched_axes = 0xFFU;
    test_watched_axes_calls = 0U;
    memset(test_watched_at_prepare, 0, sizeof(test_watched_at_prepare));
    memset(test_axis_fault, 0, sizeof(test_axis_fault));
    memset(test_hardware_status, 0, sizeof(test_hardware_status));
    memset(test_hardware_stop_reason, 0, sizeof(test_hardware_stop_reason));
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
    memset(test_prepare_calls, 0, sizeof(test_prepare_calls));
    memset(test_start_calls, 0, sizeof(test_start_calls));
    memset(test_last_startup_ns, 0, sizeof(test_last_startup_ns));
    memset(test_stop_calls, 0, sizeof(test_stop_calls));
    memset(test_listener_calls, 0, sizeof(test_listener_calls));
    memset(test_stream_listener, 0, sizeof(test_stream_listener));
    memset(test_axis_streaming, 0, sizeof(test_axis_streaming));
    memset(test_brake_requests, 0, sizeof(test_brake_requests));
    memset(test_configure_brake_calls, 0, sizeof(test_configure_brake_calls));
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = TEST_TIMER_CLOCK_HZ;
    }
    memset(test_outputs_calls, 0, sizeof(test_outputs_calls));
    memset(test_outputs_positive, 0xFF, sizeof(test_outputs_positive));
    test_outputs_result = MOTION_AXIS_HARDWARE_STATUS_READY;
    test_fail_prepare = 0U;
    test_fail_acquire = 0U;
    test_fail_commit = 0U;
    test_fail_start_channel = TEST_CHANNELS;
    test_wrong_axis_calls = 0U;
    test_trigger_calls = 0U;
    test_trigger_axes = 0U;
    test_limit_listener = NULL;
    test_limit_context = NULL;
    test_limit_listener_calls = 0U;
    step_generator_task_group = (StepStreamGroup_t)STEP_GENERATOR_TASK_GROUP_INITIAL;
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
    expect_u32(test_listener_calls[TEST_CH_X], 1U, "task registers the stream listener of X once");
    expect_u32(test_listener_calls[TEST_CH_Y], 1U, "task registers the stream listener of Y once");
    expect_u32(test_listener_calls[TEST_CH_Z], 1U, "task registers the stream listener of Z once");
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_X, "only X is watched while only X moves: Y and Z may stand on their switches");

    expect_u32(test_prepare_calls[TEST_CH_X], 1U, "the stream of X is prepared");
    expect_u32(test_prepare_calls[TEST_CH_Y], 1U, "the stream of Y is prepared with it");
    expect_u32(test_prepare_calls[TEST_CH_Z], 1U, "the stream of Z is prepared with it");
    expect_u32(test_trigger_calls, 1U, "the group triggers the channels once, after arming them");
    expect_u32(test_trigger_axes, 3U, "one trigger covers all three channels of the group");
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_RUNNING,
        "the group runs the stream"
    );
    expect_u32(test_wrong_axis_calls, 0U, "every stream call goes to the channel of X, Y or Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "the driver outputs of X are set once");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 1U, "positive block sets the positive direction");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 0U, "a block that does not move Y leaves its driver alone");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "a block that does not move Z leaves its driver alone");
    expect_u32(test_delay_calls, 1U, "DIR setup time is waited");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "the channel of X starts once both halves are filled");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the channel of Y starts with it");
    expect_u32(test_start_calls[TEST_CH_Z], 1U, "the channel of Z starts with it");
    expect_u32(test_last_startup_ns[TEST_CH_X], STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed to X");
    expect_u32(test_last_startup_ns[TEST_CH_Y], STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed to Y");
    expect_u32(test_last_startup_ns[TEST_CH_Z], STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed to Z");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed after the start");
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_u32(test_half_empty[channel][0] + test_half_empty[channel][1], 0U, "both halves are committed");
    }
    expect_u32(test_halves[TEST_CH_X][0][0], 2749U, "X step interval is 10 us in ticks");
    expect_u32(test_halves[TEST_CH_X][0][2], 1375U, "X pulse is 5 us in ticks");
    /* Y and Z wait out the whole block as one idle record: 3 steps of 10 us are 30 us, 8250 ticks at 275 MHz, and the
     * record holds the period minus one. */
    expect_u32(test_halves[TEST_CH_Y][0][0], 8249U, "Y waits out the block in one idle record of its length");
    expect_u32(test_halves[TEST_CH_Y][0][2], 0U, "Y has no pulse in a block that does not move it");
    expect_u32(test_halves[TEST_CH_Z][0][0], 8249U, "Z waits out the block in one idle record of its length");
    expect_u32(test_halves[TEST_CH_Z][0][2], 0U, "Z has no pulse in a block that does not move it");

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

static void test_y_and_xy_blocks_reach_their_channels(void) {
    reset_fixture();
    push_block_xy(31U, 0, -3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "a block of Y sets the driver outputs of Y");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 0U, "negative block sets the negative direction of Y");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 0U, "a block that does not move X leaves its driver alone");
    expect_u32(test_halves[TEST_CH_Y][0][2], 1375U, "Y pulse is 5 us in ticks");
    expect_u32(test_halves[TEST_CH_X][0][2], 0U, "X has no pulse in a block that does not move it");
    expect_u32(test_watched_axes_calls, 1U, "the watched axes are set once for the stream");
    expect_u32(test_watched_at_prepare[TEST_CH_X], 1U << MOTION_AXIS_Y, "the axes are narrowed before X is prepared");
    expect_u32(test_watched_at_prepare[TEST_CH_Y], 1U << MOTION_AXIS_Y, "the axes are narrowed before Y is prepared");
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Y, "only Y is watched while only Y moves: X may stand on its switch");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the stream of Y starts");
    expect_u32(step_generator_task_diagnostics.axis_errors, 0U, "a block of Y is no axis error");
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "a block of Y is no generator error");

    /* A diagonal moves both channels, each in its own direction. */
    reset_fixture();
    push_block_xy(32U, 3, -3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "a diagonal sets the outputs of X");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "a diagonal sets the outputs of Y");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 1U, "X goes the positive way");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 0U, "Y goes the negative way");
    expect_u32(test_halves[TEST_CH_X][0][2], 1375U, "the diagonal has a pulse on X");
    expect_u32(test_halves[TEST_CH_Y][0][2], 1375U, "the diagonal has a pulse on Y");
    expect_u32(test_trigger_axes, 3U, "all three channels start together");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "a diagonal of X and Y leaves the driver of Z alone");
    expect_u32(test_halves[TEST_CH_Z][0][2], 0U, "Z has no pulse in a diagonal of X and Y");
    expect_u32(test_watched_axes, (1U << MOTION_AXIS_X) | (1U << MOTION_AXIS_Y), "a diagonal watches both axes");
}

/* The channel of Z is member 2 of the group: its block reaches the driver of Z and its own ring. */
static void test_z_and_xyz_blocks_reach_their_channels(void) {
    reset_fixture();
    push_block_xyz(33U, 0, 0, -3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "a block of Z sets the driver outputs of Z once");
    expect_u32(test_outputs_positive[MOTION_AXIS_Z], 0U, "negative block sets the negative direction of Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 0U, "a block of Z leaves the driver of X alone");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 0U, "a block of Z leaves the driver of Y alone");
    expect_u32(test_halves[TEST_CH_Z][0][0], 2749U, "Z step interval is 10 us in ticks");
    expect_u32(test_halves[TEST_CH_Z][0][2], 1375U, "Z pulse is 5 us in ticks");
    expect_u32(test_halves[TEST_CH_X][0][2], 0U, "X has no pulse in a block of Z");
    expect_u32(test_halves[TEST_CH_Y][0][2], 0U, "Y has no pulse in a block of Z");
    expect_u32(test_watched_at_prepare[TEST_CH_Z], 1U << MOTION_AXIS_Z, "the axes are narrowed to Z before Z is prepared");
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Z, "only Z is watched while only Z moves");
    expect_u32(test_start_calls[TEST_CH_Z], 1U, "the stream of Z starts");
    expect_u32(test_trigger_axes, 3U, "Z starts in the same trigger as X and Y");
    expect_u32(test_wrong_axis_calls, 0U, "every call goes to a known channel");
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "a block of Z is no generator error");
    expect_u32(step_generator_task_diagnostics.axis_errors, 0U, "a block of Z is no axis error");
    expect_u32(step_generator_task_halted, 0U, "a block of Z does not halt feeding");

    /* All three axes in one block: each channel gets its direction and its pulse. */
    reset_fixture();
    push_block_xyz(34U, 3, -3, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "X driver is set for XYZ");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "Y driver is set for XYZ");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "Z driver is set for XYZ");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 1U, "X goes the positive way in XYZ");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 0U, "Y goes the negative way in XYZ");
    expect_u32(test_outputs_positive[MOTION_AXIS_Z], 1U, "Z goes the positive way in XYZ");
    expect_u32(test_halves[TEST_CH_X][0][2], 1375U, "XYZ has a pulse on X");
    expect_u32(test_halves[TEST_CH_Y][0][2], 1375U, "XYZ has a pulse on Y");
    expect_u32(test_halves[TEST_CH_Z][0][2], 1375U, "XYZ has a pulse on Z");
    expect_u32(test_delay_calls, 3U, "DIR setup time is waited once per moving axis");
    expect_u32(
        test_watched_axes,
        (1U << MOTION_AXIS_X) | (1U << MOTION_AXIS_Y) | (1U << MOTION_AXIS_Z),
        "XYZ watches all three axes"
    );
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "XYZ is no generator error");
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
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "an idle stream is stopped on the channel of X");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "and on the channel of Y");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "and on the channel of Z");
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
    expect_u32(test_start_calls[TEST_CH_X], 2U, "the next block starts a second stream on X");
    expect_u32(test_start_calls[TEST_CH_Y], 2U, "and on Y");
    expect_u32(test_start_calls[TEST_CH_Z], 2U, "and on Z");
}

static void test_unusable_timing_disables_feeding(void) {
    reset_fixture();
    push_block(4U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task without a config only waits");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream without a config");
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
    test_timer_clock_hz[TEST_CH_X] = 0U;
    test_timer_clock_hz[TEST_CH_Y] = 0U;
    test_timer_clock_hz[TEST_CH_Z] = 0U;
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
    test_timer_clock_hz[TEST_CH_X] = UINT32_MAX;
    test_timer_clock_hz[TEST_CH_Y] = UINT32_MAX;
    test_timer_clock_hz[TEST_CH_Z] = UINT32_MAX;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "overflowing timing is counted");
}

/* The feeder counts every record in the ticks of one clock, so channels on different kernel clocks cannot share it. */
static void test_channels_on_different_clocks_disable_feeding(void) {
    reset_fixture();
    push_block_xy(40U, 3, 3);
    test_timer_clock_hz[TEST_CH_Y] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "channels on different clocks are counted");
    expect_u32(step_generator_task_halted, 1U, "no channel is fed on different clocks");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream is prepared on different clocks");
    expect_u32(test_configure_brake_calls[TEST_CH_X], 0U, "no feeder, no ramp");

    /* The channel of Z is checked as well: a clock that differs only there also disables feeding. */
    reset_fixture();
    push_block_xyz(41U, 3, 3, 3);
    test_timer_clock_hz[TEST_CH_Z] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "a different clock of Z is counted");
    expect_u32(step_generator_task_halted, 1U, "no channel is fed when only Z has a different clock");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream is prepared when Z has a different clock");
    expect_u32(test_configure_brake_calls[TEST_CH_Z], 0U, "no feeder, no ramp for Z");
}

static void test_stop_halts_and_resets(void) {
    reset_fixture();
    push_block(5U, 3);
    expect_u32(step_generator_task_is_halted(), 0U, "a fresh task is not halted");
    step_generator_task_halt();
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "STOP stops the channel of X at once");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "STOP stops the channel of Y at once");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "STOP stops the channel of Z at once");
    expect_u32(test_wrong_axis_calls, 0U, "STOP reaches the channels of X, Y and Z only");
    expect_u32(step_generator_task_halted, 1U, "STOP halts feeding");
    expect_u32(step_generator_task_is_halted(), 1U, "the halt is visible to MotionTask");
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "halted task only waits");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "halted task prepares nothing");
    expect_u32(test_start_calls[TEST_CH_X], 0U, "halted task starts nothing");

    reset_fixture();
    push_block(6U, 3);
    step_generator_task_halt();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task resets on the STOP reset flag");
    expect_u32(step_generator_task_halted, 0U, "reset lifts the halt");
    expect_u32(step_generator_task_diagnostics.stop_resets, 1U, "reset is counted");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "feeding goes on after the reset");

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
    expect_u32(
        test_start_calls[TEST_CH_X] + test_start_calls[TEST_CH_Y] + test_start_calls[TEST_CH_Z],
        0U,
        "no axis is started while halted"
    );
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed");
}

static void test_direction_is_refused_while_halted(void) {
    reset_fixture();
    step_generator_task_halted = 1U;
    expect_u32(
        step_generator_task_port_set_direction(0U, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver is not touched after STOP"
    );
    expect_u32(
        step_generator_task_port_set_direction(1U, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver of Y is not touched after STOP either"
    );
    expect_u32(
        step_generator_task_port_set_direction(2U, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver of Z is not touched after STOP either"
    );
    expect_u32(
        test_outputs_calls[MOTION_AXIS_X] + test_outputs_calls[MOTION_AXIS_Y] + test_outputs_calls[MOTION_AXIS_Z],
        0U,
        "no DIR or ENA write while halted"
    );
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed");
}

static void test_direction_goes_to_the_axis_of_the_lane(void) {
    reset_fixture();
    expect_u32(
        step_generator_task_port_set_direction(1U, 1U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 1 is set"
    );
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "lane 1 is the driver of Y");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 0U, "lane 1 does not touch X");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 1U, "the direction reaches Y");
    expect_u32(
        step_generator_task_port_set_direction(0U, 0U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 0 is set"
    );
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "lane 0 is the driver of X");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 0U, "the direction reaches X");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "lanes 0 and 1 do not touch Z");
    const uint32_t x_calls_before_lane_2 = test_outputs_calls[MOTION_AXIS_X];
    const uint32_t y_calls_before_lane_2 = test_outputs_calls[MOTION_AXIS_Y];
    expect_u32(
        step_generator_task_port_set_direction(2U, 1U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 2 is set"
    );
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "lane 2 is the driver of Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], x_calls_before_lane_2, "lane 2 does not touch X");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], y_calls_before_lane_2, "lane 2 does not touch Y");
    expect_u32(test_outputs_positive[MOTION_AXIS_Z], 1U, "the direction reaches Z");
}

static void test_busy_covers_queue_and_stream(void) {
    reset_fixture();
    expect_u32(step_generator_task_is_busy(), 0U, "idle task with an empty plan is not busy");

    push_block(20U, 3);
    expect_u32(step_generator_task_is_busy(), 1U, "a queued block makes the task busy");

    /* Looking at the block takes nothing; giving it back is what the busy flag has to survive. */
    const uint32_t epoch = step_generator_task_port_plan_epoch();
    MotionPlanBlock_t seen;
    expect_u32(step_generator_task_port_peek_block(epoch, 0U, &seen), STEP_STREAM_FEEDER_PLAN_OK, "block is seen");
    expect_u32(seen.sequence, 20U, "the block that was queued");
    expect_u32(step_generator_task_port_peek_block(epoch, 1U, &seen), STEP_STREAM_FEEDER_PLAN_NONE, "no second block");
    expect_u32(step_generator_task_stream_busy, 0U, "looking does not mark a stream");
    expect_u32(
        step_generator_task_port_discard_blocks(epoch, 1U),
        STEP_STREAM_FEEDER_PLAN_OK,
        "the block is given back"
    );
    expect_u32(step_generator_task_stream_busy, 1U, "giving a block back marks the stream before the plan is empty");
    expect_u32(step_generator_task_is_busy(), 1U, "a block taken from the plan keeps the task busy");
    expect_u32(
        step_generator_task_port_peek_block(epoch, 0U, &seen),
        STEP_STREAM_FEEDER_PLAN_NONE,
        "the plan holds nothing now"
    );
    expect_u32(
        step_generator_task_port_discard_blocks(epoch, 1U),
        STEP_STREAM_FEEDER_PLAN_ERROR,
        "more blocks than queued cannot be given back"
    );
    (void)plan_buffer_clear();
    expect_u32(
        step_generator_task_port_peek_block(epoch, 0U, &seen),
        STEP_STREAM_FEEDER_PLAN_STALE,
        "a clear makes the old epoch stale"
    );
    expect_u32(
        step_generator_task_port_discard_blocks(epoch, 0U),
        STEP_STREAM_FEEDER_PLAN_STALE,
        "stale for a discard too"
    );
    /* The plan buffer checks the output before the epoch, so a missing output is an error even for a stale epoch. */
    expect_u32(
        step_generator_task_port_peek_block(epoch, 0U, NULL),
        STEP_STREAM_FEEDER_PLAN_ERROR,
        "a missing output is an error"
    );

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
    test_fail_start_channel = TEST_CH_X;
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
    test_fail_start_channel = TEST_CH_X;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_halted, 1U, "a refused start halts feeding");
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "axis error is counted");
    expect_u32(step_generator_task_diagnostics.streams_started, 0U, "no stream started");
    expect_true(test_stop_calls[TEST_CH_X] >= 1U, "the feeder stops the channel of X on a fault");
    expect_u32(test_wrong_axis_calls, 0U, "the feeder stops and starts the channels of X and Y only");

    /* The channel of Y refuses after X was armed: X must not be left waiting for a trigger. */
    reset_fixture();
    push_block(14U, 3);
    test_fail_start_channel = TEST_CH_Y;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_halted, 1U, "a refusal of the channel of Y halts feeding");
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "a refusal of Y is an axis error");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "X was armed before Y refused");
    expect_u32(test_axis_streaming[TEST_CH_X], 0U, "the channel of X that was armed is not left running");
    expect_u32(test_trigger_calls, 0U, "no trigger after a refused arming");

    /* The channel of Z refuses last: X and Y were armed and must not be left waiting for a trigger. */
    reset_fixture();
    push_block(16U, 3);
    test_fail_start_channel = TEST_CH_Z;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_halted, 1U, "a refusal of the channel of Z halts feeding");
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "a refusal of Z is an axis error");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "X was armed before Z refused");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "Y was armed before Z refused");
    expect_u32(test_axis_streaming[TEST_CH_X], 0U, "the channel of X that was armed is not left running");
    expect_u32(test_axis_streaming[TEST_CH_Y], 0U, "the channel of Y that was armed is not left running");
    expect_u32(test_trigger_calls, 0U, "no trigger after Z refused arming");

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
        expect_u32(test_start_calls[TEST_CH_X], 1U, "a completed or stopped axis can start a stream");
        expect_u32(test_outputs_positive[MOTION_AXIS_X], 0U, "negative block sets the negative direction");
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
    MotionPlanBlock_t invalid = make_block(13U, 3, 0);
    invalid.master_step_count = 0U;
    expect_u32(plan_buffer_try_push(&invalid), PLAN_BUFFER_STATUS_OK, "invalid block is queued");
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "a rejected block is a generator error");
    expect_u32(step_generator_task_halted, 1U, "a rejected block halts feeding");
    expect_u32(test_start_calls[TEST_CH_X], 0U, "nothing starts after a rejected block");

    /* A block of X and Z is played on both lanes: Z has a channel, so it is no longer refused. */
    reset_fixture();
    push_block_xyz(15U, 3, 0, 3);
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "a block of X and Z is no generator error");
    expect_u32(step_generator_task_halted, 0U, "a block of X and Z does not halt feeding");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "a block of X and Z starts the stream");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "a block of X and Z sets the driver of Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 0U, "a block of X and Z leaves Y alone");
}

static void test_listener_and_notifications(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);

    test_flags_set_value = 0U;
    test_stream_listener[TEST_CH_X].on_half_released(test_stream_listener[TEST_CH_X].context);
    expect_u32(
        test_flags_set_value,
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED,
        "a released ring half of X wakes the task"
    );
    test_flags_set_value = 0U;
    test_stream_listener[TEST_CH_Y].on_half_released(test_stream_listener[TEST_CH_Y].context);
    expect_u32(
        test_flags_set_value,
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED,
        "a released ring half of Y wakes the task"
    );
    expect_u32(step_generator_task_halted, 0U, "a released half does not halt the task");
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Y].on_fault(test_stream_listener[TEST_CH_Y].context, STEP_STREAM_AXIS_FAULT_UNDERRUN);
    expect_u32(step_generator_task_halted, 1U, "a stream fault halts feeding");
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_FAULT,
        "a channel fault puts the group in FAULT"
    );
    expect_u32(
        step_stream_group_get_fault(&step_generator_task_group),
        (uint32_t)STEP_STREAM_AXIS_FAULT_UNDERRUN,
        "the group keeps the fault of the channel"
    );
    expect_u32(step_stream_group_get_fault_member(&step_generator_task_group), 1U, "the fault is that of member 1, Y");
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the channel of X is stopped for the fault of Y");
    expect_u32(test_stop_calls[TEST_CH_Y], 0U, "the failed channel of Y stopped itself");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "the channel of Z is stopped for the fault of Y");
    expect_u32(test_wrong_axis_calls, 0U, "the group stops and starts the channels of X, Y and Z only");

    step_generator_task_halted = 0U;
    test_stream_listener[TEST_CH_X].on_brake(test_stream_listener[TEST_CH_X].context);
    expect_u32(step_generator_task_halted, 1U, "braking of X on a limit halts feeding");
    step_generator_task_halted = 0U;
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    expect_u32(step_generator_task_halted, 1U, "braking of Y on a limit halts feeding");
    step_generator_task_halted = 0U;
    test_stream_listener[TEST_CH_Z].on_brake(test_stream_listener[TEST_CH_Z].context);
    expect_u32(step_generator_task_halted, 1U, "braking of Z on a limit halts feeding");
    expect_u32(step_generator_task_halt_member, 2U, "braking of Z is blamed on member 2");
    expect_u32(step_generator_task_halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "braking is named as a limit");

    /* A fault of the channel of Z is member 2 and the rest of the group is stopped. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_stream_listener[TEST_CH_Z].on_half_released(test_stream_listener[TEST_CH_Z].context);
    expect_u32(
        test_flags_set_value,
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED,
        "a released ring half of Z wakes the task"
    );
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Z].on_fault(test_stream_listener[TEST_CH_Z].context, STEP_STREAM_AXIS_FAULT_DMA);
    expect_u32(step_generator_task_halted, 1U, "a fault of Z halts feeding");
    expect_u32(step_stream_group_get_fault_member(&step_generator_task_group), 2U, "the fault is that of member 2, Z");
    expect_u32(
        step_stream_group_get_fault(&step_generator_task_group),
        (uint32_t)STEP_STREAM_AXIS_FAULT_DMA,
        "the group keeps the fault of Z"
    );
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the channel of X is stopped for the fault of Z");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "the channel of Y is stopped for the fault of Z");
    expect_u32(test_stop_calls[TEST_CH_Z], 0U, "the failed channel of Z stopped itself");

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

/* The diagnostics must tell who halted the stream: the next failure is found by this, not by reading the code. */
static void test_stream_diagnostics_name_the_cause_of_a_halt(void) {
    StepGeneratorTaskStreamDiagnostics_t stream = {0};
    step_generator_task_get_stream_diagnostics(NULL);

    reset_fixture();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halted, 0U, "a fresh task is not halted");
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_NONE, "no halt, no cause");
    expect_u32(stream.halt_member, STEP_GENERATOR_TASK_NO_MEMBER, "no halt, no member");
    expect_u32(stream.halts_total, 0U, "no halt is counted");

    step_generator_task_halt();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halted, 1U, "STOP halts");
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "STOP is named as the cause");
    expect_u32(stream.halt_member, STEP_GENERATOR_TASK_NO_MEMBER, "STOP blames no channel");
    expect_u32(stream.halts_total, 1U, "STOP is counted");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    run_group_prepared_and_started();
    test_limit_listener(test_limit_context, MOTION_AXIS_Y);
    /* The group halts the feeding once a channel reports that it brakes, not at the switch itself. */
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "a switch is named as the cause");
    expect_u32(stream.halt_member, 1U, "the switch of Y is member 1");
    expect_u32(stream.members[0].limit_trips, 0U, "X reported no switch");
    expect_u32(stream.members[1].limit_trips, 1U, "Y reported one switch");
    expect_u32(stream.members[2].limit_trips, 0U, "Z reported no switch");

    /* MotionTask ends the move after the switch stopped it: that second halt must not hide the switch. */
    step_generator_task_halt();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "a later halt keeps the first cause");
    expect_u32(stream.halt_member, 1U, "a later halt keeps the first member");
    expect_u32(stream.halts_total, 2U, "a later halt is still counted");

    /* After the reset the next halt records its own cause. */
    step_generator_task_halted = 0U;
    step_generator_task_halt();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "the first halt after a reset sets the cause");
    expect_u32(stream.halt_member, STEP_GENERATOR_TASK_NO_MEMBER, "and its member");
    expect_u32(stream.halts_total, 3U, "three halts in all");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_X].on_fault(test_stream_listener[TEST_CH_X].context, STEP_STREAM_AXIS_FAULT_DMA);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "a ring fault is named as the cause");
    expect_u32(stream.halt_member, 0U, "the fault is that of X");
    expect_u32(stream.halt_fault, (uint32_t)STEP_STREAM_AXIS_FAULT_DMA, "the fault code is kept");

    reset_fixture();
    push_block(70U, 3);
    test_fail_prepare = 1U;
    (void)run_task_until_jump(&test_task_config);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR, "a feeder error is named as the cause");
    expect_u32(stream.feeder_status, (uint32_t)STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "the feeder status is kept");
    expect_u32(stream.feeder_phase, (uint32_t)STEP_STREAM_FEEDER_PHASE_FAULT, "the feeder phase is reported");

    reset_fixture();
    (void)run_task_until_jump(NULL);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED, "a missing feeder is named as the cause");

    /* A channel that still runs after the reset keeps the task halted. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_wait_actions[0] = TEST_ACTION_Y_STREAMING;
    (void)run_task_until_jump(&test_task_config);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED, "a reset that fails is named as the cause");

    /* A STOP reset watches every axis again; a stream that moves only Z watches only Z. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_watched_axes = 0x01U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_watched_axes, STEP_GENERATOR_TASK_ALL_AXES_WATCHED, "the reset watches every axis again");
    step_generator_task_feeder.direction_known_mask = MOTION_PLANNER_AXIS_Z_MASK;
    step_generator_task_watch_moving_axes();
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Z, "a stream that moves only Z watches only Z");

    /* A limit of Z is member 2: it is counted for Z only and named as the cause. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    run_group_prepared_and_started();
    test_limit_listener(test_limit_context, MOTION_AXIS_Z);
    test_stream_listener[TEST_CH_Z].on_brake(test_stream_listener[TEST_CH_Z].context);
    test_limit_listener(test_limit_context, MOTION_AXIS_Z);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "a switch of Z is named as the cause");
    expect_u32(stream.halt_member, 2U, "the switch of Z is member 2");
    expect_u32(stream.members[0].limit_trips, 0U, "X reported no switch of Z");
    expect_u32(stream.members[1].limit_trips, 0U, "Y reported no switch of Z");
    expect_u32(stream.members[2].limit_trips, 2U, "Z reported two switches");

    /* The state of every channel comes from its own channel and its own axis. */
    reset_fixture();
    test_axis_streaming[TEST_CH_Y] = 1U;
    test_axis_fault[TEST_CH_X] = STEP_STREAM_AXIS_FAULT_UNDERRUN;
    test_axis_fault[TEST_CH_Y] = STEP_STREAM_AXIS_FAULT_SEQUENCE;
    test_axis_fault[TEST_CH_Z] = STEP_STREAM_AXIS_FAULT_DMA;
    test_axis_streaming[TEST_CH_Z] = 0U;
    test_hardware_status[MOTION_AXIS_X] = MOTION_AXIS_HARDWARE_STATUS_READY;
    test_hardware_status[MOTION_AXIS_Y] = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    test_hardware_status[MOTION_AXIS_Z] = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    test_hardware_stop_reason[MOTION_AXIS_X] = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    test_hardware_stop_reason[MOTION_AXIS_Y] = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    test_hardware_stop_reason[MOTION_AXIS_Z] = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.members[2].channel_state, (uint32_t)STEP_STREAM_AXIS_STATE_IDLE, "Z channel is idle");
    expect_u32(stream.members[2].channel_fault, (uint32_t)STEP_STREAM_AXIS_FAULT_DMA, "Z channel fault");
    expect_u32(stream.members[2].hardware_status, (uint32_t)MOTION_AXIS_HARDWARE_STATUS_COMPLETE, "Z axis status");
    expect_u32(
        stream.members[2].hardware_stop_reason, (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, "Z stop reason"
    );
    test_axis_streaming[TEST_CH_Z] = 1U;
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.members[2].channel_state, (uint32_t)STEP_STREAM_AXIS_STATE_STREAMING, "Z channel streams");
    expect_u32(stream.members[0].channel_state, (uint32_t)STEP_STREAM_AXIS_STATE_IDLE, "X stays idle while Z streams");
    test_axis_streaming[TEST_CH_Z] = 0U;
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.members[0].channel_state, (uint32_t)STEP_STREAM_AXIS_STATE_IDLE, "X channel is idle");
    expect_u32(stream.members[1].channel_state, (uint32_t)STEP_STREAM_AXIS_STATE_STREAMING, "Y channel streams");
    expect_u32(stream.members[0].channel_fault, (uint32_t)STEP_STREAM_AXIS_FAULT_UNDERRUN, "X channel fault");
    expect_u32(stream.members[1].channel_fault, (uint32_t)STEP_STREAM_AXIS_FAULT_SEQUENCE, "Y channel fault");
    expect_u32(stream.members[0].hardware_status, (uint32_t)MOTION_AXIS_HARDWARE_STATUS_READY, "X axis status");
    expect_u32(stream.members[1].hardware_status, (uint32_t)MOTION_AXIS_HARDWARE_STATUS_STOPPED, "Y axis status");
    expect_u32(
        stream.members[0].hardware_stop_reason, (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, "X stop reason"
    );
    expect_u32(
        stream.members[1].hardware_stop_reason, (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, "Y stop reason"
    );
}

static void test_limit_switch_brakes_through_the_group(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_limit_listener_calls, 1U, "the task registers its limit listener once");
    expect_true(test_limit_listener != NULL, "the limit listener is set");

    run_group_prepared_and_started();
    test_limit_listener(test_limit_context, MOTION_AXIS_HARDWARE_COUNT);
    expect_u32(
        test_brake_requests[TEST_CH_X] + test_brake_requests[TEST_CH_Y] + test_brake_requests[TEST_CH_Z],
        0U,
        "a switch of an axis without a channel brakes nothing"
    );
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_RUNNING,
        "a switch of an axis without a channel does not stop the group"
    );
    StepGeneratorTaskStreamDiagnostics_t no_channel = {0};
    step_generator_task_get_stream_diagnostics(&no_channel);
    expect_u32(
        no_channel.members[0].limit_trips + no_channel.members[1].limit_trips + no_channel.members[2].limit_trips,
        0U,
        "a switch of an axis without a channel is counted for no member"
    );

    /* The group moves its axes together, so the switch of any axis brakes all channels. */
    test_limit_listener(test_limit_context, MOTION_AXIS_X);
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of X brakes the channel of X");
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the switch of X brakes the channel of Y too");
    expect_u32(test_brake_requests[TEST_CH_Z], 1U, "the switch of X brakes the channel of Z too");
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group stops when a switch is reached"
    );
    expect_u32(test_wrong_axis_calls, 0U, "the brake goes to the channels of X, Y and Z");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    run_group_prepared_and_started();
    test_limit_listener(test_limit_context, MOTION_AXIS_Y);
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of Y brakes the channel of X");
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the switch of Y brakes the channel of Y");
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group stops when the switch of Y is reached"
    );
    expect_u32(test_brake_requests[TEST_CH_Z], 1U, "the switch of Y brakes the channel of Z");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    run_group_prepared_and_started();
    test_limit_listener(test_limit_context, MOTION_AXIS_Z);
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of Z brakes the channel of X");
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the switch of Z brakes the channel of Y");
    expect_u32(test_brake_requests[TEST_CH_Z], 1U, "the switch of Z brakes the channel of Z");
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group stops when the switch of Z is reached"
    );
}

static void test_group_ends_with_its_stream(void) {
    reset_fixture();
    test_wait_script_count = 2U;
    test_wait_actions[0] = TEST_ACTION_RUN_GROUP_THEN_END;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_IDLE,
        "the task sees that no channel runs and the group is IDLE again"
    );
}

static void test_stop_reset_clears_a_group_fault(void) {
    reset_fixture();
    test_wait_script_count = 2U;
    test_wait_actions[0] = TEST_ACTION_FAULT_THEN_STOP;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_stream_group_get_state(&step_generator_task_group),
        STEP_STREAM_GROUP_STATE_IDLE,
        "the reset after STOP clears the fault of the group"
    );
    expect_u32(step_stream_group_get_fault(&step_generator_task_group), 0U, "the reset forgets the fault");
    expect_u32(step_generator_task_halted, 0U, "feeding goes on after the reset");
    expect_u32(step_generator_task_diagnostics.axis_errors, 0U, "a clean reset is no error");
}

static void test_reset_is_refused_while_a_channel_runs(void) {
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_actions[0] = TEST_ACTION_Y_STREAMING;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "a channel of Y that still runs is an axis error");
    expect_u32(step_generator_task_halted, 1U, "nothing is fed while a channel runs after STOP");

    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_actions[0] = TEST_ACTION_Z_STREAMING;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.axis_errors, 1U, "a channel of Z that still runs is an axis error");
    expect_u32(step_generator_task_halted, 1U, "nothing is fed while the channel of Z runs after STOP");
}

static void test_brake_ramp_is_built_from_the_driver_timing(void) {
    reset_fixture();
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_configure_brake_calls[TEST_CH_X], 1U, "the braking ramp of X is configured once");
    expect_u32(test_configure_brake_calls[TEST_CH_Y], 1U, "the braking ramp of Y is configured once");
    expect_u32(test_configure_brake_calls[TEST_CH_Z], 1U, "the braking ramp of Z is configured once");
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
    expect_u32(
        test_configure_brake_calls[TEST_CH_X] + test_configure_brake_calls[TEST_CH_Y] +
            test_configure_brake_calls[TEST_CH_Z],
        0U,
        "no deceleration setting, no ramp"
    );
    expect_u32(step_generator_task_diagnostics.generator_errors, 0U, "a missing ramp is not an error here");

    reset_fixture();
    test_configure_brake_result = STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_diagnostics.generator_errors, 1U, "a ramp the axes refuse is counted once");
    expect_u32(step_generator_task_halted, 0U, "feeding goes on without a ramp");

    /* The error count adds to the feeder's own and saturates with it. */
    step_generator_task_feeder.diagnostics.generator_errors = 4U;
    step_generator_task_publish_diagnostics();
    expect_u32(step_generator_task_diagnostics.generator_errors, 5U, "ramp error adds to the feeder errors");
    step_generator_task_feeder.diagnostics.generator_errors = UINT32_MAX;
    step_generator_task_publish_diagnostics();
    expect_u32(step_generator_task_diagnostics.generator_errors, UINT32_MAX, "the sum saturates");

    /* Resets that left a channel running add to the axis errors of the feeder and saturate with them. */
    step_generator_task_feeder.diagnostics.axis_errors = 6U;
    step_generator_task_reset_failures = 2U;
    step_generator_task_publish_diagnostics();
    expect_u32(step_generator_task_diagnostics.axis_errors, 8U, "reset failures add to the feeder axis errors");
    step_generator_task_feeder.diagnostics.axis_errors = UINT32_MAX;
    step_generator_task_publish_diagnostics();
    expect_u32(step_generator_task_diagnostics.axis_errors, UINT32_MAX, "the axis error sum saturates");

    /* An unusable timing builds no feeder, so no ramp either. */
    reset_fixture();
    test_timer_clock_hz[TEST_CH_X] = 0U;
    test_timer_clock_hz[TEST_CH_Y] = 0U;
    test_timer_clock_hz[TEST_CH_Z] = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        test_configure_brake_calls[TEST_CH_X] + test_configure_brake_calls[TEST_CH_Y] +
            test_configure_brake_calls[TEST_CH_Z],
        0U,
        "no feeder, no ramp"
    );
}

int main(void) {
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        test_axis_markers[channel] = (int)channel;
    }
    test_task_streams_a_block();
    test_y_and_xy_blocks_reach_their_channels();
    test_z_and_xyz_blocks_reach_their_channels();
    test_idle_stream_stops();
    test_unusable_timing_disables_feeding();
    test_channels_on_different_clocks_disable_feeding();
    test_stop_halts_and_resets();
    test_stream_diagnostics_name_the_cause_of_a_halt();
    test_start_is_refused_while_halted();
    test_direction_is_refused_while_halted();
    test_direction_goes_to_the_axis_of_the_lane();
    test_busy_covers_queue_and_stream();
    test_faults_halt_the_task();
    test_listener_and_notifications();
    test_limit_switch_brakes_through_the_group();
    test_group_ends_with_its_stream();
    test_stop_reset_clears_a_group_fault();
    test_reset_is_refused_while_a_channel_runs();
    test_brake_ramp_is_built_from_the_driver_timing();
    test_brake_is_skipped_or_counted();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Step generator task unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Step generator task unit tests passed.\n");
    return 0;
}

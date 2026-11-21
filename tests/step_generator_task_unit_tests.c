#include "motion/plan_buffer.h"
#include "motion/step_stream_group.h"
#include "motion_axis_hardware.h"
#include "step_generator_task.h"
#include "step_stream_axis.h"
#include "syringe_driver.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* The queue of the stream under test. */
static PlanBuffer_t test_plan_buffer;
static MotionPlanBlock_t test_plan_buffer_entries[PLAN_BUFFER_CAPACITY];

static void reset_test_plan_buffer(void) {
    (void)memset(&test_plan_buffer, 0, sizeof(test_plan_buffer));
    (void)memset(test_plan_buffer_entries, 0, sizeof(test_plan_buffer_entries));
}

#define TEST_THREAD_ID ((osThreadId_t)(uintptr_t)0x5678U)
#define TEST_WAIT_SCRIPT_LENGTH 4U
#define TEST_TIMER_CLOCK_HZ 275000000U
#define TEST_HALF_WORDS (STEP_STREAM_AXIS_HALF_RECORD_COUNT * 3U)
/* The channels of the axes: channel 0 is X, channel 1 is Y, channel 2 is Z. The per-axis tables (limit switches, driver
 * outputs) and the loops over them keep to these three. */
#define TEST_CHANNELS 3U
#define TEST_CH_X 0U
#define TEST_CH_Y 1U
#define TEST_CH_Z 2U
/* The channel of the syringe follows those of the axes: it is the fourth member of the printing stream. The per-channel
 * state below has a slot for it. */
#define TEST_CH_SYRINGE 3U
#define TEST_ALL_CHANNELS 4U
/* The member of the syringe in the printing instance, and the number of members that its diagnostic frame lists. */
#define TEST_MEMBER_SYRINGE 3U

/* Defined by the task source included below, where `static` is switched off for the tests. */
/* Defined after the include: the group of the one instance. */
static StepStreamGroup_t *test_group(void);
/* Same value as STEP_GENERATOR_TASK_FLAG_STOP_RESET, which the file under test defines after the mocks. */
#define TEST_FLAG_STOP_RESET (1UL << 1)
/* Asks every instance to reset, through the real resume function; an instance that is not set up ignores it. */
static void test_request_reset(void);
/* The print instance takes its channels by hand, for the tests that fire the listeners without a stream. */
static void test_claim_print_channels(void);
/* The context that a channel or a switch of the axis instance of X reports with, and the print instance's by member. */
static const void *test_axis_context(MotionAxis_t axis);
static const void *test_print_context(uint32_t member);
static void test_check_print_still_holds(void);
/* The snapshot of an instance, filled with a marker first so that a field that is not written shows. */
static StepGeneratorProgress_t read_progress(const StepGeneratorInstance_t *instance);
/* Gives the one-lane instance of the axis a queue of its own in the config. */
static void init_axis_plan_buffer(MotionAxis_t axis);

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
    TEST_ACTION_FAULT_THEN_STOP,
    /* The channels no longer stream: the ramp that kept them busy has been played. Checks that the print instance
     * went on holding them until now. */
    TEST_ACTION_FREE_STUCK_CHANNELS,
    /* STOP of the instance of X in the middle of its stream; the channel of X is at rest afterwards. */
    TEST_ACTION_HALT_X_AXIS_INSTANCE,
    /* STOP of the instance of X after which the channel of X is still running. */
    TEST_ACTION_HALT_X_AXIS_INSTANCE_CHANNEL_RUNS,
    /* A block of X and Y arrives in the print queue while another instance already streams on X. */
    TEST_ACTION_PUSH_PRINT_BLOCK_XY,
    /* Records what MotionTask can read of the print instance (ready, halted, reset count) before the reset of this wait. */
    TEST_ACTION_SAMPLE_PRINT,
    /* The channel of the syringe still runs when the task is asked to reset. */
    TEST_ACTION_SYRINGE_STREAMING
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
/* What TEST_ACTION_SAMPLE_PRINT read in each wait, through the public functions of the print instance. */
static uint32_t test_sampled_ready[TEST_WAIT_SCRIPT_LENGTH];
static uint32_t test_sampled_halted[TEST_WAIT_SCRIPT_LENGTH];
static uint32_t test_sampled_reset_count[TEST_WAIT_SCRIPT_LENGTH];
static uint32_t test_kernel_lock_depth;
static uint32_t test_delay_calls;
/* The argument of the latest osDelay. */
static uint32_t test_delay_last_ticks;

/* Fake stream channels of X, Y, Z and the syringe: two halves each, EMPTY after prepare. A channel is told apart by its
 * marker. */
static int test_axis_markers[TEST_ALL_CHANNELS];
static uint16_t test_halves[TEST_ALL_CHANNELS][2][TEST_HALF_WORDS];
static uint8_t test_half_empty[TEST_ALL_CHANNELS][2];
static uint32_t test_prepare_calls[TEST_ALL_CHANNELS];
static uint32_t test_start_calls[TEST_ALL_CHANNELS];
static uint32_t test_last_startup_ns[TEST_ALL_CHANNELS];
static uint32_t test_stop_calls[TEST_ALL_CHANNELS];
/* Calls of step_stream_axis_set_listener, accepted or not. */
static uint32_t test_listener_calls[TEST_ALL_CHANNELS];
/* The context of the owner of each channel (the listener's context), NULL while the channel is free; the callbacks of the
 * owner are in test_stream_listener. */
static const void *test_stream_owner[TEST_ALL_CHANNELS];
static StepStreamAxisListener_t test_stream_listener[TEST_ALL_CHANNELS];
/* The channel goes on streaming after a stop, as one that plays its braking ramp does. */
static uint8_t test_axis_stuck[TEST_ALL_CHANNELS];
static uint32_t test_timer_clock_hz[TEST_ALL_CHANNELS];
static uint8_t test_axis_streaming[TEST_ALL_CHANNELS];
static uint32_t test_brake_requests[TEST_ALL_CHANNELS];
static uint32_t test_configure_brake_calls[TEST_ALL_CHANNELS];
static uint32_t test_trigger_calls;
static uint32_t test_trigger_axes;
/* Driver outputs, by motion axis. */
static uint32_t test_outputs_calls[MOTION_AXIS_HARDWARE_COUNT];
static uint8_t test_outputs_positive[MOTION_AXIS_HARDWARE_COUNT];
static MotionAxisHardwareStatus_t test_outputs_result;
static uint8_t test_fail_prepare;
static uint8_t test_fail_acquire;
static uint8_t test_fail_commit;
/* Channel whose arming fails, or TEST_ALL_CHANNELS for none. */
static uint8_t test_fail_start_channel;
/* The switch listener of each axis and who owns it; NULL while the axis is free. */
static MotionAxisHardwareStreamLimitListener_t test_limit_listeners[MOTION_AXIS_HARDWARE_COUNT];
static void *test_limit_contexts[MOTION_AXIS_HARDWARE_COUNT];
/* Calls that offer a listener, and calls that let go (listener NULL), accepted or not. */
static uint32_t test_limit_listener_calls[MOTION_AXIS_HARDWARE_COUNT];
static uint32_t test_limit_releases[MOTION_AXIS_HARDWARE_COUNT];
/* Calls of step_stream_axis_release_listener, whoever the owner is. */
static uint32_t test_listener_releases[TEST_ALL_CHANNELS];
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
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "test block is queued");
}

static void push_block_xy(uint32_t sequence, int64_t x_steps, int64_t y_steps) {
    push_block_xyz(sequence, x_steps, y_steps, 0);
}

static void push_block(uint32_t sequence, int64_t x_steps) { push_block_xy(sequence, x_steps, 0); }

/* A block of the syringe alone: `steps` steps (signed) at 100 kHz, one 10 us step each. */
static MotionPlanBlock_t make_syringe_block(uint32_t sequence, int64_t steps) {
    MotionPlanBlock_t block = {
        .sequence = sequence,
        .master_step_count = (uint64_t)(steps < 0 ? -steps : steps),
        .step_event_rate_numerator = 100000U,
        .step_event_rate_denominator = 1U,
        .moving_axes_mask = MOTION_PLAN_CHANNEL_SYRINGE_MASK,
        .positive_direction_mask = steps > 0 ? MOTION_PLAN_CHANNEL_SYRINGE_MASK : 0U,
    };
    block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE] = steps;
    return block;
}

static void push_syringe_block(uint32_t sequence, int64_t steps) {
    const MotionPlanBlock_t block = make_syringe_block(sequence, steps);
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "syringe block is queued");
}

static uint32_t channel_index(const StepStreamAxis_t *axis) {
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
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
    test_delay_last_ticks = milliseconds;
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
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_half_empty[channel][half] = 1U;
    }
}

static void run_group_prepared_and_started(void) {
    expect_u32(step_stream_group_prepare(test_group()), STEP_STREAM_GROUP_STATUS_OK, "group prepares");
    expect_u32(step_stream_group_start(test_group(), 2500U), STEP_STREAM_GROUP_STATUS_OK, "group starts");
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
        test_axis_streaming[TEST_CH_SYRINGE] = 0U;
        break;
    case TEST_ACTION_Y_STREAMING:
        test_axis_streaming[TEST_CH_Y] = 1U;
        break;
    case TEST_ACTION_Z_STREAMING:
        test_axis_streaming[TEST_CH_Z] = 1U;
        break;
    case TEST_ACTION_SYRINGE_STREAMING:
        test_axis_streaming[TEST_CH_SYRINGE] = 1U;
        break;
    case TEST_ACTION_PUSH_PRINT_BLOCK_XY:
        push_block_xy(400U + index, 3, 3);
        break;
    case TEST_ACTION_FREE_STUCK_CHANNELS:
        test_check_print_still_holds();
        (void)memset(test_axis_stuck, 0, sizeof(test_axis_stuck));
        (void)memset(test_axis_streaming, 0, sizeof(test_axis_streaming));
        break;
    case TEST_ACTION_HALT_X_AXIS_INSTANCE:
    case TEST_ACTION_HALT_X_AXIS_INSTANCE_CHANNEL_RUNS:
        expect_true(
            test_stream_owner[TEST_CH_X] == test_axis_context(MOTION_AXIS_X),
            "the instance of X owns its channel while it streams"
        );
        step_generator_instance_halt(step_generator_task_get_axis_instance(MOTION_AXIS_X));
        if (test_wait_actions[index] == TEST_ACTION_HALT_X_AXIS_INSTANCE_CHANNEL_RUNS) {
            test_axis_streaming[TEST_CH_X] = 1U;
        }
        break;
    case TEST_ACTION_FAULT_THEN_STOP:
        /* The fault is reported by a channel, so the print instance has to own its channels. */
        test_claim_print_channels();
        run_group_prepared_and_started();
        test_stream_listener[TEST_CH_Y].on_fault(
            test_stream_listener[TEST_CH_Y].context,
            STEP_STREAM_AXIS_FAULT_SEQUENCE
        );
        test_axis_streaming[TEST_CH_X] = 0U;
        test_axis_streaming[TEST_CH_Y] = 0U;
        test_axis_streaming[TEST_CH_Z] = 0U;
        test_axis_streaming[TEST_CH_SYRINGE] = 0U;
        step_generator_task_halt_all();
        expect_u32(
            step_stream_group_get_state(test_group()),
            STEP_STREAM_GROUP_STATE_FAULT,
            "STOP does not clear a fault"
        );
        break;
    case TEST_ACTION_SAMPLE_PRINT:
        test_sampled_ready[index] = step_generator_instance_is_ready(step_generator_task_get_print_instance());
        test_sampled_halted[index] = step_generator_instance_is_halted(step_generator_task_get_print_instance());
        test_sampled_reset_count[index] = step_generator_instance_reset_count(step_generator_task_get_print_instance());
        break;
    case TEST_ACTION_NONE:
    default:
        break;
    }
    /* The reset flag a wait returns is the one the real resume sets, so the request goes through the real function. */
    if ((test_wait_results[index] & TEST_FLAG_STOP_RESET) != 0U) {
        test_request_reset();
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

/* The channel of the syringe is the fourth fake channel, with the state of the others. */
StepStreamAxis_t *step_stream_axis_get_syringe(void) { return (StepStreamAxis_t *)&test_axis_markers[TEST_CH_SYRINGE]; }

/* The DIR/ENA outputs of the syringe: calls, the last direction, and what the driver answers (1 written, 0 refused). */
static uint32_t test_syringe_outputs_calls;
static uint8_t test_syringe_outputs_positive;
static uint8_t test_syringe_outputs_result;

uint8_t syringe_driver_set_stream_outputs(uint8_t positive_direction) {
    ++test_syringe_outputs_calls;
    test_syringe_outputs_positive = positive_direction;
    return test_syringe_outputs_result;
}

/* As the channel does: BUSY while it streams and while another owner (another context) holds it; the owner may repeat. */
StepStreamAxisStatus_t step_stream_axis_set_listener(StepStreamAxis_t *axis, const StepStreamAxisListener_t *listener) {
    const uint32_t channel = channel_index(axis);
    ++test_listener_calls[channel];
    const uint8_t owned_by_other =
        (uint8_t)(listener != NULL && test_stream_owner[channel] != NULL && test_stream_owner[channel] != listener->context);
    if (test_axis_streaming[channel] != 0U || owned_by_other != 0U) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    if (listener == NULL) {
        test_stream_owner[channel] = NULL;
        memset(&test_stream_listener[channel], 0, sizeof(test_stream_listener[channel]));
        return STEP_STREAM_AXIS_STATUS_OK;
    }
    test_stream_listener[channel] = *listener;
    test_stream_owner[channel] = listener->context;
    return STEP_STREAM_AXIS_STATUS_OK;
}

/* As the channel does: lets go only of the listener whose context is `context`; BUSY while the owner's channel streams. */
StepStreamAxisStatus_t step_stream_axis_release_listener(StepStreamAxis_t *axis, const void *context) {
    const uint32_t channel = channel_index(axis);
    ++test_listener_releases[channel];
    if (context == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    if (test_stream_owner[channel] != context) {
        return STEP_STREAM_AXIS_STATUS_OK;
    }
    if (test_axis_streaming[channel] != 0U) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    test_stream_owner[channel] = NULL;
    memset(&test_stream_listener[channel], 0, sizeof(test_stream_listener[channel]));
    return STEP_STREAM_AXIS_STATUS_OK;
}

uint32_t step_stream_axis_get_timer_clock_hz(const StepStreamAxis_t *axis) {
    return test_timer_clock_hz[channel_index(axis)];
}

/* The axes that the limit interrupt watches; the stream may not start before they are narrowed to the moving ones. */
/* Bit = MotionAxis_t of X, Y and Z: every axis of the instance is watched until a stream narrows it. */
#define TEST_ALL_AXES_WATCHED 0x07U
static uint8_t test_watched_axes = TEST_ALL_AXES_WATCHED;
static uint32_t test_watched_axes_calls;
static uint8_t test_watched_at_prepare[TEST_ALL_CHANNELS];

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
    if (test_axis_stuck[channel] == 0U) {
        test_axis_streaming[channel] = 0U;
    }
}

void step_stream_axis_request_brake(StepStreamAxis_t *axis) { ++test_brake_requests[channel_index(axis)]; }

StepStreamAxisState_t step_stream_axis_get_state(const StepStreamAxis_t *axis) {
    return test_axis_streaming[channel_index(axis)] != 0U ? STEP_STREAM_AXIS_STATE_STREAMING
                                                          : STEP_STREAM_AXIS_STATE_IDLE;
}

/* Records of the plan that each channel reports as played, and how often the task asked for them. */
static uint32_t test_records_played[TEST_ALL_CHANNELS];
static uint32_t test_records_played_calls[TEST_ALL_CHANNELS];

uint32_t step_stream_axis_plan_records_played(const StepStreamAxis_t *axis) {
    const uint32_t channel = channel_index(axis);
    ++test_records_played_calls[channel];
    return test_records_played[channel];
}

static StepStreamAxisFault_t test_axis_fault[TEST_ALL_CHANNELS];
/* Indexed by MotionAxis_t: the syringe has no hardware axis. */
static MotionAxisHardwareStatus_t test_hardware_status[MOTION_AXIS_HARDWARE_COUNT];
static MotionAxisHardwareStopReason_t test_hardware_stop_reason[MOTION_AXIS_HARDWARE_COUNT];

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

uint8_t motion_axis_hardware_set_stream_limit_listener(
    MotionAxis_t axis,
    MotionAxisHardwareStreamLimitListener_t listener,
    void *context
) {
    /* As the axis does: a release has nothing to refuse when the caller owns nothing (also while the channel streams),
     * refuses while the owner's channel streams, and lets go otherwise; a listener goes into a free slot or over the
     * same owner, and only while the channel is IDLE. */
    const uint8_t idle = test_axis_streaming[axis] == 0U ? 1U : 0U;
    if (listener == NULL) {
        ++test_limit_releases[axis];
        if (test_limit_listeners[axis] != NULL && test_limit_contexts[axis] == context) {
            if (idle == 0U) {
                return 0U;
            }
            test_limit_listeners[axis] = NULL;
            test_limit_contexts[axis] = NULL;
        }
        return 1U;
    }
    ++test_limit_listener_calls[axis];
    if (idle == 0U) {
        return 0U;
    }
    if (test_limit_listeners[axis] != NULL && test_limit_contexts[axis] != context) {
        return 0U;
    }
    test_limit_listeners[axis] = listener;
    test_limit_contexts[axis] = context;
    return 1U;
}

/* The tests read the watched state as a mask, bit = MotionAxis_t, as the old interface passed it. */
void motion_axis_hardware_set_stream_axis_watched(MotionAxis_t axis, uint8_t watched) {
    test_watched_axes = watched != 0U ? (uint8_t)(test_watched_axes | (1U << (uint32_t)axis))
                                      : (uint8_t)(test_watched_axes & ~(1U << (uint32_t)axis));
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

_Static_assert(TEST_FLAG_STOP_RESET == STEP_GENERATOR_TASK_FLAG_STOP_RESET, "the mock returns the reset flag of the task");

static void test_request_reset(void) { step_generator_task_resume_all_after_stop(); }
#undef static

static StepStreamGroup_t *test_group(void) { return &step_generator_task_instance.group; }

static const void *test_axis_context(MotionAxis_t axis) {
    return &step_generator_task_axis_instances[axis].member_contexts[0];
}

static const void *test_print_context(uint32_t member) { return &step_generator_task_instance.member_contexts[member]; }

static void test_claim_print_channels(void) {
    expect_u32(
        step_generator_task_claim_channels(&step_generator_task_instance),
        1U,
        "the print instance takes its channels"
    );
}

/* The print instance tried to let go of its channels while they still stream: it holds them and its listeners stay. */
static void test_check_print_still_holds(void) {
    expect_u32(step_generator_task_instance.channels_held, 1U, "an instance that could not let go still holds its channels");
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        expect_true(
            test_stream_owner[member] == test_print_context(member),
            "the print instance stays the owner of a channel that still streams"
        );
    }
    /* The syringe has no switch: only the three axes have a switch owner. */
    for (uint32_t axis = 0U; axis < TEST_CHANNELS; ++axis) {
        expect_true(
            test_limit_contexts[axis] == &step_generator_task_instance,
            "the print instance stays the owner of the switch of a channel that still streams"
        );
    }
    /* One attempt when the feeder stopped the stream, one when the pass ended with an idle feeder. */
    expect_u32(test_listener_releases[TEST_CH_X], 2U, "the release was tried twice while the channel streamed");
    expect_u32(test_listener_releases[TEST_CH_SYRINGE], 2U, "the release of the syringe was tried twice as well");
}

/* A second instance with the tables of the axes X, Y and Z as its three members, standing for a stream of the axes
 * only: every link of it points to itself. It has the policy of the single-axis instances. */
static void make_other_instance(StepGeneratorInstance_t *other) {
    *other = (StepGeneratorInstance_t)STEP_GENERATOR_TASK_INSTANCE_INITIAL(
        *other,
        step_generator_channels_axis_members,
        step_generator_channels_axis_planner_channels,
        STEP_GENERATOR_CHANNELS_AXIS_COUNT,
        0U
    );
}

static StepGeneratorTaskConfig_t test_task_config;
/* The queues of the one-lane instances, indexed by MotionAxis_t; a test hands one to the config with init_axis_plan_buffer. */
static PlanBuffer_t test_axis_plan_buffer[MOTION_AXIS_HARDWARE_COUNT];
static MotionPlanBlock_t test_axis_plan_entries[MOTION_AXIS_HARDWARE_COUNT][PLAN_BUFFER_CAPACITY];

static void reset_instances(void) {
    step_generator_task_instance =
        (StepGeneratorInstance_t)STEP_GENERATOR_TASK_PRINT_INSTANCE_INITIAL(step_generator_task_instance);
    step_generator_task_axis_instances[MOTION_AXIS_X] =
        (StepGeneratorInstance_t)STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(MOTION_AXIS_X);
    step_generator_task_axis_instances[MOTION_AXIS_Y] =
        (StepGeneratorInstance_t)STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(MOTION_AXIS_Y);
    step_generator_task_axis_instances[MOTION_AXIS_Z] =
        (StepGeneratorInstance_t)STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(MOTION_AXIS_Z);
}

static void reset_fixture(void) {
    reset_test_plan_buffer();
    expect_u32(
        plan_buffer_init(&test_plan_buffer, test_plan_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "plan buffer initializes"
    );
    reset_instances();
    step_generator_task_instance.plan_buffer = &test_plan_buffer;
    (void)memset(test_axis_plan_buffer, 0, sizeof(test_axis_plan_buffer));
    (void)memset(test_axis_plan_entries, 0, sizeof(test_axis_plan_entries));
    step_generator_task_thread_id = NULL;
    test_watched_axes = TEST_ALL_AXES_WATCHED;
    test_watched_axes_calls = 0U;
    memset(test_watched_at_prepare, 0, sizeof(test_watched_at_prepare));
    memset(test_axis_fault, 0, sizeof(test_axis_fault));
    memset(test_hardware_status, 0, sizeof(test_hardware_status));
    memset(test_hardware_stop_reason, 0, sizeof(test_hardware_stop_reason));
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_flags_set_calls = 0U;
    test_flags_set_value = 0U;
    test_flags_set_target = NULL;
    test_wait_calls = 0U;
    test_wait_script_count = 0U;
    test_wait_last_timeout = 0U;
    memset(test_sampled_ready, 0xFF, sizeof(test_sampled_ready));
    memset(test_sampled_halted, 0xFF, sizeof(test_sampled_halted));
    memset(test_sampled_reset_count, 0xFF, sizeof(test_sampled_reset_count));
    test_kernel_lock_depth = 0U;
    test_delay_calls = 0U;
    test_delay_last_ticks = 0U;
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
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = TEST_TIMER_CLOCK_HZ;
    }
    memset(test_outputs_calls, 0, sizeof(test_outputs_calls));
    memset(test_outputs_positive, 0xFF, sizeof(test_outputs_positive));
    test_outputs_result = MOTION_AXIS_HARDWARE_STATUS_READY;
    test_fail_prepare = 0U;
    test_fail_acquire = 0U;
    test_fail_commit = 0U;
    test_fail_start_channel = TEST_ALL_CHANNELS;
    test_wrong_axis_calls = 0U;
    test_syringe_outputs_calls = 0U;
    test_syringe_outputs_positive = 0xFFU;
    test_syringe_outputs_result = 1U;
    test_trigger_calls = 0U;
    test_trigger_axes = 0U;
    memset(test_limit_listener_calls, 0, sizeof(test_limit_listener_calls));
    memset(test_limit_releases, 0, sizeof(test_limit_releases));
    memset(test_limit_listeners, 0, sizeof(test_limit_listeners));
    memset(test_limit_contexts, 0, sizeof(test_limit_contexts));
    memset(test_listener_releases, 0, sizeof(test_listener_releases));
    memset(test_stream_owner, 0, sizeof(test_stream_owner));
    memset(test_axis_stuck, 0, sizeof(test_axis_stuck));
    memset(test_records_played, 0, sizeof(test_records_played));
    memset(test_records_played_calls, 0, sizeof(test_records_played_calls));
    test_brake_config = (StepStreamBrakeConfig_t){0};
    test_configure_brake_result = STEP_STREAM_AXIS_STATUS_OK;
    test_task_config = (StepGeneratorTaskConfig_t){
        .step_pulse_width_ns = 5000U,
        .step_low_time_ns = 2500U,
        .limit_deceleration_steps_per_s2 = 600000U,
        .plan_buffer = &test_plan_buffer,
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
    expect_u32(test_listener_calls[TEST_CH_X], 1U, "the stream registers the stream listener of X once");
    expect_u32(test_listener_calls[TEST_CH_Y], 1U, "the stream registers the stream listener of Y once");
    expect_u32(test_listener_calls[TEST_CH_Z], 1U, "the stream registers the stream listener of Z once");
    expect_true(test_stream_owner[TEST_CH_X] == test_print_context(0U), "X channel belongs to the print instance");
    expect_true(test_stream_owner[TEST_CH_Y] == test_print_context(1U), "Y channel belongs to the print instance");
    expect_true(test_stream_owner[TEST_CH_Z] == test_print_context(2U), "Z channel belongs to the print instance");
    expect_u32(test_listener_calls[TEST_CH_SYRINGE], 1U, "the stream registers the stream listener of the syringe once");
    expect_true(
        test_stream_owner[TEST_CH_SYRINGE] == test_print_context(TEST_MEMBER_SYRINGE),
        "the syringe channel belongs to the print instance as its member 3"
    );
    expect_u32(step_generator_task_instance.channels_held, 1U, "the print instance holds its channels while it streams");
    expect_u32(
        test_watched_axes,
        1U << MOTION_AXIS_X,
        "only X is watched while only X moves: Y and Z may stand on their switches"
    );

    expect_u32(test_prepare_calls[TEST_CH_X], 1U, "the stream of X is prepared");
    expect_u32(test_prepare_calls[TEST_CH_Y], 1U, "the stream of Y is prepared with it");
    expect_u32(test_prepare_calls[TEST_CH_Z], 1U, "the stream of Z is prepared with it");
    expect_u32(test_trigger_calls, 1U, "the group triggers the channels once, after arming them");
    expect_u32(test_prepare_calls[TEST_CH_SYRINGE], 1U, "the stream of the syringe is prepared with it");
    expect_u32(test_trigger_axes, 4U, "one trigger covers all four channels of the group");
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_RUNNING, "the group runs the stream");
    expect_u32(test_wrong_axis_calls, 0U, "every stream call goes to the channel of X, Y, Z or the syringe");
    expect_u32(test_syringe_outputs_calls, 0U, "a block that does not move the syringe leaves its driver alone");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "the driver outputs of X are set once");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 1U, "positive block sets the positive direction");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 0U, "a block that does not move Y leaves its driver alone");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "a block that does not move Z leaves its driver alone");
    expect_u32(test_delay_calls, 1U, "DIR setup time is waited");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "the channel of X starts once both halves are filled");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the channel of Y starts with it");
    expect_u32(test_start_calls[TEST_CH_Z], 1U, "the channel of Z starts with it");
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 1U, "the channel of the syringe starts with it");
    expect_u32(
        test_last_startup_ns[TEST_CH_SYRINGE],
        STEP_STREAM_AXIS_MIN_STARTUP_NS,
        "startup period is passed to the syringe"
    );
    expect_u32(test_last_startup_ns[TEST_CH_X], STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed to X");
    expect_u32(test_last_startup_ns[TEST_CH_Y], STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed to Y");
    expect_u32(test_last_startup_ns[TEST_CH_Z], STEP_STREAM_AXIS_MIN_STARTUP_NS, "startup period is passed to Z");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed after the start");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
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
    expect_u32(test_halves[TEST_CH_SYRINGE][0][0], 8249U, "the syringe waits out the block in one idle record");
    expect_u32(test_halves[TEST_CH_SYRINGE][0][2], 0U, "the syringe has no pulse in a block that does not move it");

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
    expect_u32(test_watched_axes_calls, 3U, "the watched state is set once per axis for the stream");
    expect_u32(test_watched_at_prepare[TEST_CH_X], 1U << MOTION_AXIS_Y, "the axes are narrowed before X is prepared");
    expect_u32(test_watched_at_prepare[TEST_CH_Y], 1U << MOTION_AXIS_Y, "the axes are narrowed before Y is prepared");
    expect_u32(
        test_watched_axes,
        1U << MOTION_AXIS_Y,
        "only Y is watched while only Y moves: X may stand on its switch"
    );
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the stream of Y starts");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 0U, "a block of Y is no axis error");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "a block of Y is no generator error");

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
    expect_u32(test_trigger_axes, 4U, "all four channels start together");
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
    expect_u32(
        test_watched_at_prepare[TEST_CH_Z],
        1U << MOTION_AXIS_Z,
        "the axes are narrowed to Z before Z is prepared"
    );
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Z, "only Z is watched while only Z moves");
    expect_u32(test_start_calls[TEST_CH_Z], 1U, "the stream of Z starts");
    expect_u32(test_trigger_axes, 4U, "Z starts in the same trigger as X, Y and the syringe");
    expect_u32(test_wrong_axis_calls, 0U, "every call goes to a known channel");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "a block of Z is no generator error");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 0U, "a block of Z is no axis error");
    expect_u32(step_generator_task_instance.halted, 0U, "a block of Z does not halt feeding");

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
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "XYZ is no generator error");
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
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "and on the channel of the syringe");
    expect_u32(step_generator_task_instance.diagnostics.streams_stopped, 1U, "stream stop is counted");
    expect_u32(step_generator_task_instance.halted, 0U, "a normal end does not halt the task");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_true(test_stream_owner[channel] == NULL, "the channel is free once its stream has stopped");
    }
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_true(test_limit_contexts[channel] == NULL, "the switch of the axis is free once its stream has stopped");
        expect_u32(test_limit_releases[channel], 1U, "the switch listener is let go once, at the stop of the stream");
    }
    expect_u32(step_generator_task_instance.channels_held, 0U, "the print instance holds nothing after its stream");

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
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 2U, "and on the syringe");
    expect_u32(test_listener_calls[TEST_CH_SYRINGE], 2U, "each stream takes the channel of the syringe again");
    expect_u32(test_listener_calls[TEST_CH_X], 2U, "each stream takes the channel of X again");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_X], 2U, "each stream takes the switch of X again");
    expect_true(test_stream_owner[TEST_CH_X] == test_print_context(0U), "the second stream owns the channel of X");
}

static void test_unusable_timing_disables_feeding(void) {
    reset_fixture();
    push_block(4U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task without a config only waits");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream without a config");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "missing config is counted");
    expect_u32(step_generator_task_instance.halted, 1U, "task is halted without a config");
    PlanBufferDiagnostics_t plan_diagnostics;
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan_diagnostics);
    expect_u32(plan_diagnostics.queued, 1U, "the block stays in the plan buffer");

    /* A STOP reset does not revive a task that has no feeder. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_instance.diagnostics.stop_resets, 0U, "no reset without a feeder");
    expect_u32(step_generator_task_instance.halted, 1U, "still halted");

    reset_fixture();
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = 0U;
    }
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "unusable clock is counted");

    reset_fixture();
    test_task_config.step_pulse_width_ns = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "zero pulse width is counted");

    /* Timing whose tick count does not fit 32 bits is refused when the feeder config is built. */
    reset_fixture();
    test_task_config.step_low_time_ns = UINT32_MAX;
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = UINT32_MAX;
    }
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "overflowing timing is counted");
}

/* The feeder counts every record in the ticks of one clock, so channels on different kernel clocks cannot share it. */
static void test_channels_on_different_clocks_disable_feeding(void) {
    reset_fixture();
    push_block_xy(40U, 3, 3);
    test_timer_clock_hz[TEST_CH_Y] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(
        step_generator_task_instance.diagnostics.generator_errors,
        1U,
        "channels on different clocks are counted"
    );
    expect_u32(step_generator_task_instance.halted, 1U, "no channel is fed on different clocks");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream is prepared on different clocks");
    expect_u32(test_configure_brake_calls[TEST_CH_X], 0U, "no feeder, no ramp");

    /* The channel of Z is checked as well: a clock that differs only there also disables feeding. */
    reset_fixture();
    push_block_xyz(41U, 3, 3, 3);
    test_timer_clock_hz[TEST_CH_Z] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "a different clock of Z is counted");
    expect_u32(step_generator_task_instance.halted, 1U, "no channel is fed when only Z has a different clock");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream is prepared when Z has a different clock");
    expect_u32(test_configure_brake_calls[TEST_CH_Z], 0U, "no feeder, no ramp for Z");

    /* The syringe is member 3: a clock that differs only there also disables the printing stream. */
    reset_fixture();
    push_block_xy(42U, 3, 3);
    test_timer_clock_hz[TEST_CH_SYRINGE] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "a different clock of the syringe is counted");
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 0U, "the print instance is not ready");
    expect_u32(step_generator_task_instance.halted, 1U, "no channel is fed when only the syringe has a different clock");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream is prepared when the syringe has a different clock");
    expect_u32(
        test_configure_brake_calls[TEST_CH_SYRINGE],
        0U,
        "no feeder, no ramp for the syringe"
    );

    /* The first member is the reference of the timing: the other clocks only have to agree with it. All four channels
     * on 137.5 MHz are ready and the records are counted in the ticks of that clock: 10 us are 1375 ticks (the record
     * holds one less), and 5 us are 687.5, rounded up to 688. */
    reset_fixture();
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = TEST_TIMER_CLOCK_HZ / 2U;
    }
    push_block(43U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 1U, "all channels on one clock are ready");
    expect_u32(test_halves[TEST_CH_X][0][0], 1374U, "the interval is counted in the ticks of the common clock");
    expect_u32(test_halves[TEST_CH_X][0][2], 688U, "the pulse is counted in the ticks of the common clock");
}

static void test_stop_halts_and_resets(void) {
    reset_fixture();
    push_block(5U, 3);
    expect_u32(step_generator_instance_is_halted(&step_generator_task_instance), 0U, "a fresh task is not halted");
    /* Only an instance that holds its channels stops them: the one-lane instances hold none here. */
    test_claim_print_channels();
    step_generator_task_halt_all();
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "STOP stops the channel of X at once, only for the instance that holds it");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "STOP stops the channel of Y at once, only for the instance that holds it");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "STOP stops the channel of Z at once, only for the instance that holds it");
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "STOP stops the channel of the syringe at once as well");
    expect_u32(test_wrong_axis_calls, 0U, "STOP reaches the channels of X, Y, Z and the syringe only");
    expect_u32(step_generator_task_instance.halted, 1U, "STOP halts feeding");
    expect_u32(step_generator_instance_is_halted(&step_generator_task_instance), 1U, "the halt is visible to MotionTask");
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "halted task only waits");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "halted task prepares nothing");
    expect_u32(test_start_calls[TEST_CH_X], 0U, "halted task starts nothing");

    reset_fixture();
    push_block(6U, 3);
    step_generator_task_halt_all();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task resets on the STOP reset flag");
    expect_u32(step_generator_task_instance.halted, 0U, "reset lifts the halt");
    expect_u32(step_generator_task_instance.diagnostics.stop_resets, 1U, "reset is counted");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "feeding goes on after the reset");

    reset_fixture();
    step_generator_instance_resume_after_stop(&step_generator_task_instance);
    expect_u32(test_flags_set_calls, 0U, "resume before the task starts is ignored");
    expect_u32(step_generator_task_instance.reset_requested, 0U, "resume before the task starts leaves no request");
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_instance_resume_after_stop(&step_generator_task_instance);
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_STOP_RESET, "resume sets the reset flag");
    expect_u32(step_generator_task_instance.reset_requested, 1U, "resume marks the instance for the reset");
}

static void test_instance_api(void) {
    reset_fixture();
    expect_true(
        step_generator_task_get_print_instance() == &step_generator_task_instance,
        "the print instance is the one the task serves"
    );

    /* A NULL instance is idle and ignores every request. */
    step_generator_instance_halt(NULL);
    step_generator_instance_resume_after_stop(NULL);
    expect_u32(step_generator_instance_is_busy(NULL), 0U, "NULL instance is not busy");
    expect_u32(step_generator_instance_is_halted(NULL), 0U, "NULL instance is not halted");
    expect_u32(step_generator_task_instance.halts_total, 0U, "NULL halt touches no instance");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_stop_calls[channel], 0U, "NULL halt stops no channel");
    }
    expect_u32(step_generator_task_instance.reset_requested, 0U, "NULL resume touches no instance");
    expect_u32(test_flags_set_calls, 0U, "NULL resume signals nothing");

    /* An instance that holds no channel halts feeding but stops no channel: they are not its own. */
    step_generator_instance_halt(&step_generator_task_instance);
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_stop_calls[channel], 0U, "halt of an instance without channels stops none");
    }
    expect_u32(step_generator_task_instance.halted, 1U, "instance halt halts feeding");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL,
        "instance halt records a halt call"
    );

    /* One instance that holds its channels: halt stops exactly its own, each once. */
    reset_fixture();
    test_claim_print_channels();
    step_generator_instance_halt(&step_generator_task_instance);
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "instance halt stops the channel of X");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "instance halt stops the channel of Y");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "instance halt stops the channel of Z");
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "instance halt stops the channel of the syringe");

    /* STOP of all instances reaches the channels of the instance that holds them. */
    reset_fixture();
    test_claim_print_channels();
    step_generator_task_halt_all();
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "STOP of all stops the channel of X once, for its holder");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "STOP of all stops the channel of Y once, for its holder");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "STOP of all stops the channel of Z once, for its holder");
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "STOP of all stops the channel of the syringe once, for its holder");
    expect_u32(step_generator_task_instance.halts_total, 1U, "STOP of all halts the instance once");

    /* Resume marks the instance, then wakes the task. */
    reset_fixture();
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_task_resume_all_after_stop();
    expect_u32(step_generator_task_instance.reset_requested, 1U, "resume of all marks the instance");
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_STOP_RESET, "resume of all wakes the task");

    /* The task takes the mark when it resets the instance. */
    reset_fixture();
    push_block(8U, 3);
    step_generator_task_halt_all();
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_instance_resume_after_stop(&step_generator_task_instance);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_instance.reset_requested, 0U, "the reset consumes the request");
    expect_u32(step_generator_task_instance.diagnostics.stop_resets, 1U, "one request resets once");

    /* Without a request the halt stays. */
    reset_fixture();
    push_block(9U, 3);
    step_generator_task_halt_all();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_instance.diagnostics.stop_resets, 0U, "no request, no reset");
    expect_u32(step_generator_task_instance.halted, 1U, "the halt stays without a request");

    /* Two instances: a STOP, a request and the busy flag of one do not reach the other. */
    reset_fixture();
    static StepGeneratorInstance_t other;
    make_other_instance(&other);
    other.plan_buffer = &test_plan_buffer;
    StepGeneratorInstance_t *const pair[] = {&step_generator_task_instance, &other};
    step_generator_task_halt_list(&pair[0], 1U);
    expect_u32(step_generator_task_instance.halted, 1U, "the halted instance is halted");
    expect_u32(other.halted, 0U, "the other instance keeps feeding");
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_task_resume_list(&pair[1], 1U);
    expect_u32(other.reset_requested, 1U, "the other instance is marked for reset");
    expect_u32(step_generator_task_instance.reset_requested, 0U, "the first instance is not marked");
    step_generator_task_halt_list(pair, 2U);
    expect_u32(other.halted, 1U, "STOP of the list halts the second instance");
    expect_u32(other.halts_total, 1U, "the second instance counts its halt once");
    expect_u32(step_generator_task_instance.halts_total, 2U, "the first instance counts both halts");
    step_generator_task_resume_list(pair, 2U);
    expect_u32(step_generator_task_instance.reset_requested, 1U, "resume of the list marks the first instance");

    /* The setup of an instance takes no channel, whether it succeeds or not: channels are taken when a stream opens. */
    reset_fixture();
    test_task_config.plan_buffer = NULL;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_listener_calls[channel], 0U, "a failed setup offers no stream listener");
        expect_true(test_stream_owner[channel] == NULL, "a failed setup owns no channel");
    }
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_u32(test_limit_listener_calls[channel], 0U, "a failed setup offers no switch listener");
    }
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_listener_calls[channel], 0U, "a good setup does not take the stream listener before a stream");
    }
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_u32(test_limit_listener_calls[channel], 0U, "a good setup does not take the switch listener either");
    }
    expect_u32(step_generator_task_instance.channels_held, 0U, "a ready instance without a stream holds nothing");
}

static void test_start_is_refused_while_halted(void) {
    reset_fixture();
    step_generator_task_instance.halted = 1U;
    expect_u32(
        step_generator_task_port_start(&step_generator_task_instance, 2000U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "a stream cannot start after STOP"
    );
    expect_u32(
        test_start_calls[TEST_CH_X] + test_start_calls[TEST_CH_Y] + test_start_calls[TEST_CH_Z] +
            test_start_calls[TEST_CH_SYRINGE],
        0U,
        "no channel is started while halted"
    );
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed");
}

static void test_direction_is_refused_while_halted(void) {
    reset_fixture();
    step_generator_task_instance.halted = 1U;
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 0U, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver is not touched after STOP"
    );
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 1U, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver of Y is not touched after STOP either"
    );
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 2U, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver of Z is not touched after STOP either"
    );
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, TEST_MEMBER_SYRINGE, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "the driver of the syringe is not touched after STOP either"
    );
    expect_u32(
        test_outputs_calls[MOTION_AXIS_X] + test_outputs_calls[MOTION_AXIS_Y] + test_outputs_calls[MOTION_AXIS_Z] +
            test_syringe_outputs_calls,
        0U,
        "no DIR or ENA write while halted"
    );
    expect_u32(test_delay_calls, 0U, "no settle wait after a refusal while halted");
    expect_u32(test_kernel_lock_depth, 0U, "task switching is resumed");
}

static void test_direction_goes_to_the_axis_of_the_lane(void) {
    reset_fixture();
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 1U, 1U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 1 is set"
    );
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "lane 1 is the driver of Y");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 0U, "lane 1 does not touch X");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 1U, "the direction reaches Y");
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 0U, 0U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 0 is set"
    );
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "lane 0 is the driver of X");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 0U, "the direction reaches X");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "lanes 0 and 1 do not touch Z");
    const uint32_t x_calls_before_lane_2 = test_outputs_calls[MOTION_AXIS_X];
    const uint32_t y_calls_before_lane_2 = test_outputs_calls[MOTION_AXIS_Y];
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 2U, 1U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 2 is set"
    );
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "lane 2 is the driver of Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], x_calls_before_lane_2, "lane 2 does not touch X");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], y_calls_before_lane_2, "lane 2 does not touch Y");
    expect_u32(test_outputs_positive[MOTION_AXIS_Z], 1U, "the direction reaches Z");
    expect_u32(test_syringe_outputs_calls, 0U, "lanes 0 to 2 do not touch the syringe");

    /* Lane 3 is the syringe: its own driver, no axis. */
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, TEST_MEMBER_SYRINGE, 1U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of lane 3 is set"
    );
    expect_u32(test_syringe_outputs_calls, 1U, "lane 3 is the driver of the syringe");
    expect_u32(test_syringe_outputs_positive, 1U, "the positive direction reaches the syringe");
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, TEST_MEMBER_SYRINGE, 0U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the negative direction of lane 3 is set"
    );
    expect_u32(test_syringe_outputs_calls, 2U, "the second write reached the syringe");
    expect_u32(test_syringe_outputs_positive, 0U, "the negative direction reaches the syringe");
    expect_u32(
        test_outputs_calls[MOTION_AXIS_X] + test_outputs_calls[MOTION_AXIS_Y] + test_outputs_calls[MOTION_AXIS_Z],
        x_calls_before_lane_2 + y_calls_before_lane_2 + 1U,
        "lane 3 does not touch an axis"
    );

    /* A syringe driver that refuses (not set up) is a refusal of the lane, without a settle wait. */
    const uint32_t delays_before_refusal = test_delay_calls;
    test_syringe_outputs_result = 0U;
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, TEST_MEMBER_SYRINGE, 1U),
        STEP_STREAM_FEEDER_AXIS_ERROR,
        "a refusal of the syringe driver is a refusal of lane 3"
    );
    expect_u32(test_delay_calls, delays_before_refusal, "no settle wait after a refusal of the syringe driver");
}

/* The DIR setup time: every written direction is followed by a delay of STEP_GENERATOR_TASK_DIRECTION_SETTLE_TICKS. */
static void test_direction_waits_the_settle_time(void) {
    reset_fixture();
    /* 1 ms at the 1 kHz tick of the test configuration is one tick, and one more tick because the first is cut short:
     * two ticks, counted here independently of the macro. */
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, 0U, 1U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of X is written"
    );
    expect_u32(test_delay_calls, 1U, "one wait follows the write");
    expect_u32(test_delay_last_ticks, 2U, "the wait lasts two ticks");
    test_delay_last_ticks = 0U;
    expect_u32(
        step_generator_task_port_set_direction(&step_generator_task_instance, TEST_MEMBER_SYRINGE, 0U),
        STEP_STREAM_FEEDER_AXIS_OK,
        "the direction of the syringe is written"
    );
    expect_u32(test_delay_calls, 2U, "the write of the syringe is followed by a wait too");
    expect_u32(test_delay_last_ticks, 2U, "the wait after the syringe lasts two ticks");

    /* Through a whole stream: the wait comes once per moving channel with that length. */
    reset_fixture();
    push_block(44U, 3);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_delay_calls, 1U, "a stream that moves X waits once");
    expect_u32(test_delay_last_ticks, 2U, "the wait of the stream lasts two ticks");
}

static void test_busy_covers_queue_and_stream(void) {
    reset_fixture();
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 0U, "idle task with an empty plan is not busy");

    push_block(20U, 3);
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 1U, "a queued block makes the task busy");

    /* Looking at the block takes nothing; giving it back is what the busy flag has to survive. */
    const uint32_t epoch = step_generator_task_port_plan_epoch(&step_generator_task_instance);
    MotionPlanBlock_t seen;
    expect_u32(
        step_generator_task_port_peek_block(&step_generator_task_instance, epoch, 0U, &seen),
        STEP_STREAM_FEEDER_PLAN_OK,
        "block is seen"
    );
    expect_u32(seen.sequence, 20U, "the block that was queued");
    expect_u32(
        step_generator_task_port_peek_block(&step_generator_task_instance, epoch, 1U, &seen),
        STEP_STREAM_FEEDER_PLAN_NONE,
        "no second block"
    );
    expect_u32(step_generator_task_instance.stream_busy, 0U, "looking does not mark a stream");
    expect_u32(
        step_generator_task_port_discard_blocks(&step_generator_task_instance, epoch, 1U),
        STEP_STREAM_FEEDER_PLAN_OK,
        "the block is given back"
    );
    expect_u32(
        step_generator_task_instance.stream_busy,
        1U,
        "giving a block back marks the stream before the plan is empty"
    );
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 1U, "a block taken from the plan keeps the task busy");
    expect_u32(
        step_generator_task_port_peek_block(&step_generator_task_instance, epoch, 0U, &seen),
        STEP_STREAM_FEEDER_PLAN_NONE,
        "the plan holds nothing now"
    );
    expect_u32(
        step_generator_task_port_discard_blocks(&step_generator_task_instance, epoch, 1U),
        STEP_STREAM_FEEDER_PLAN_ERROR,
        "more blocks than queued cannot be given back"
    );
    (void)plan_buffer_clear(&test_plan_buffer);
    expect_u32(
        step_generator_task_port_peek_block(&step_generator_task_instance, epoch, 0U, &seen),
        STEP_STREAM_FEEDER_PLAN_STALE,
        "a clear makes the old epoch stale"
    );
    expect_u32(
        step_generator_task_port_discard_blocks(&step_generator_task_instance, epoch, 0U),
        STEP_STREAM_FEEDER_PLAN_STALE,
        "stale for a discard too"
    );
    /* The plan buffer checks the output before the epoch, so a missing output is an error even for a stale epoch. */
    expect_u32(
        step_generator_task_port_peek_block(&step_generator_task_instance, epoch, 0U, NULL),
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
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 0U, "an ended stream is not busy");

    reset_fixture();
    push_block(22U, 3);
    test_fail_start_channel = TEST_CH_X;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 1U, "a faulted stream stays busy until STOP");

    reset_fixture();
    step_generator_task_instance.stream_busy = 1U;
    step_generator_task_instance.halted = 1U;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 0U, "STOP reset clears the busy flag");
}

static void test_faults_halt_the_task(void) {
    reset_fixture();
    push_block(7U, 3);
    test_fail_start_channel = TEST_CH_X;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.halted, 1U, "a refused start halts feeding");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "axis error is counted");
    expect_u32(step_generator_task_instance.diagnostics.streams_started, 0U, "no stream started");
    /* Once by the group that undoes the refused arming, once by the feeder's stop of the instance that holds X. */
    expect_u32(test_stop_calls[TEST_CH_X], 2U, "the channel of X is stopped by the group and by the feeder on a fault");
    expect_u32(
        test_stop_calls[TEST_CH_SYRINGE],
        2U,
        "the channel of the syringe is stopped by the group and by the feeder on a fault as well"
    );
    expect_u32(test_wrong_axis_calls, 0U, "the feeder stops and starts the channels of the group only");

    /* The channel of Y refuses after X was armed: X must not be left waiting for a trigger. */
    reset_fixture();
    push_block(14U, 3);
    test_fail_start_channel = TEST_CH_Y;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.halted, 1U, "a refusal of the channel of Y halts feeding");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "a refusal of Y is an axis error");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "X was armed before Y refused");
    expect_u32(test_axis_streaming[TEST_CH_X], 0U, "the channel of X that was armed is not left running");
    expect_u32(test_trigger_calls, 0U, "no trigger after a refused arming");

    /* The channel of Z refuses last: X and Y were armed and must not be left waiting for a trigger. */
    reset_fixture();
    push_block(16U, 3);
    test_fail_start_channel = TEST_CH_Z;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.halted, 1U, "a refusal of the channel of Z halts feeding");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "a refusal of Z is an axis error");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "X was armed before Z refused");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "Y was armed before Z refused");
    expect_u32(test_axis_streaming[TEST_CH_X], 0U, "the channel of X that was armed is not left running");
    expect_u32(test_axis_streaming[TEST_CH_Y], 0U, "the channel of Y that was armed is not left running");
    expect_u32(test_trigger_calls, 0U, "no trigger after Z refused arming");

    /* The channel of the syringe refuses last: X, Y and Z were armed and must not be left waiting for a trigger. */
    reset_fixture();
    push_block(17U, 3);
    test_fail_start_channel = TEST_CH_SYRINGE;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.halted, 1U, "a refusal of the channel of the syringe halts feeding");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "a refusal of the syringe is an axis error");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "X was armed before the syringe refused");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "Y was armed before the syringe refused");
    expect_u32(test_start_calls[TEST_CH_Z], 1U, "Z was armed before the syringe refused");
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 1U, "the syringe was asked to arm");
    expect_u32(test_axis_streaming[TEST_CH_X], 0U, "the armed channel of X is not left running");
    expect_u32(test_axis_streaming[TEST_CH_Y], 0U, "the armed channel of Y is not left running");
    expect_u32(test_axis_streaming[TEST_CH_Z], 0U, "the armed channel of Z is not left running");
    expect_u32(test_axis_streaming[TEST_CH_SYRINGE], 0U, "the channel of the syringe is not running");
    expect_u32(test_trigger_calls, 0U, "no trigger after the syringe refused arming");

    /* The driver of the syringe refuses DIR (not set up): a block that moves it does not start. */
    reset_fixture();
    {
        MotionPlanBlock_t syringe_block = make_syringe_block(18U, 3);
        expect_u32(plan_buffer_try_push(&test_plan_buffer, &syringe_block), PLAN_BUFFER_STATUS_OK, "block is queued");
    }
    test_syringe_outputs_result = 0U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_syringe_outputs_calls, 1U, "the driver of the syringe was asked once");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "a refused DIR of the syringe is an axis error");
    expect_u32(step_generator_task_instance.halted, 1U, "a refused DIR of the syringe halts feeding");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
        "the refusal is a feeder error"
    );
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 0U, "nothing starts after the refusal");
    expect_u32(test_delay_calls, 0U, "no settle wait after the refusal");

    reset_fixture();
    push_block(8U, 3);
    test_outputs_result = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "a busy driver refuses the direction");
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
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "prepare failure is an axis error");

    reset_fixture();
    push_block(11U, 3);
    test_fail_acquire = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "acquire failure is an axis error");

    reset_fixture();
    push_block(12U, 3);
    test_fail_commit = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "commit failure is an axis error");

    reset_fixture();
    MotionPlanBlock_t invalid = make_block(13U, 3, 0);
    invalid.master_step_count = 0U;
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &invalid), PLAN_BUFFER_STATUS_OK, "invalid block is queued");
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "a rejected block is a generator error");
    expect_u32(step_generator_task_instance.halted, 1U, "a rejected block halts feeding");
    expect_u32(test_start_calls[TEST_CH_X], 0U, "nothing starts after a rejected block");

    /* A block of X and Z is played on both lanes: Z has a channel, so it is no longer refused. */
    reset_fixture();
    push_block_xyz(15U, 3, 0, 3);
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_task_instance.diagnostics.generator_errors,
        0U,
        "a block of X and Z is no generator error"
    );
    expect_u32(step_generator_task_instance.halted, 0U, "a block of X and Z does not halt feeding");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "a block of X and Z starts the stream");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "a block of X and Z sets the driver of Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 0U, "a block of X and Z leaves Y alone");
}

static void test_listener_and_notifications(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();

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
    expect_u32(step_generator_task_instance.halted, 0U, "a released half does not halt the task");
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Y].on_fault(test_stream_listener[TEST_CH_Y].context, STEP_STREAM_AXIS_FAULT_UNDERRUN);
    expect_u32(step_generator_task_instance.halted, 1U, "a stream fault halts feeding");
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_FAULT,
        "a channel fault puts the group in FAULT"
    );
    expect_u32(
        step_stream_group_get_fault(test_group()),
        (uint32_t)STEP_STREAM_AXIS_FAULT_UNDERRUN,
        "the group keeps the fault of the channel"
    );
    expect_u32(step_stream_group_get_fault_member(test_group()), 1U, "the fault is that of member 1, Y");
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the channel of X is stopped for the fault of Y");
    expect_u32(test_stop_calls[TEST_CH_Y], 0U, "the failed channel of Y stopped itself");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "the channel of Z is stopped for the fault of Y");
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "the channel of the syringe is stopped for the fault of Y");
    expect_u32(test_wrong_axis_calls, 0U, "the group stops and starts the channels of its members only");

    step_generator_task_instance.halted = 0U;
    test_stream_listener[TEST_CH_X].on_brake(test_stream_listener[TEST_CH_X].context);
    expect_u32(step_generator_task_instance.halted, 1U, "braking of X on a limit halts feeding");
    step_generator_task_instance.halted = 0U;
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    expect_u32(step_generator_task_instance.halted, 1U, "braking of Y on a limit halts feeding");
    step_generator_task_instance.halted = 0U;
    test_stream_listener[TEST_CH_Z].on_brake(test_stream_listener[TEST_CH_Z].context);
    expect_u32(step_generator_task_instance.halted, 1U, "braking of Z on a limit halts feeding");
    expect_u32(step_generator_task_instance.halt_member, 2U, "braking of Z is blamed on member 2");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT,
        "braking is named as a limit"
    );

    /* A fault of the channel of Z is member 2 and the rest of the group is stopped. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    test_stream_listener[TEST_CH_Z].on_half_released(test_stream_listener[TEST_CH_Z].context);
    expect_u32(
        test_flags_set_value,
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED,
        "a released ring half of Z wakes the task"
    );
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Z].on_fault(test_stream_listener[TEST_CH_Z].context, STEP_STREAM_AXIS_FAULT_DMA);
    expect_u32(step_generator_task_instance.halted, 1U, "a fault of Z halts feeding");
    expect_u32(step_stream_group_get_fault_member(test_group()), 2U, "the fault is that of member 2, Z");
    expect_u32(
        step_stream_group_get_fault(test_group()),
        (uint32_t)STEP_STREAM_AXIS_FAULT_DMA,
        "the group keeps the fault of Z"
    );
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the channel of X is stopped for the fault of Z");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "the channel of Y is stopped for the fault of Z");
    expect_u32(test_stop_calls[TEST_CH_Z], 0U, "the failed channel of Z stopped itself");
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "the channel of the syringe is stopped for the fault of Z");

    /* The channel of the syringe is member 3: its events reach the print instance with that number. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    test_flags_set_value = 0U;
    test_stream_listener[TEST_CH_SYRINGE].on_half_released(test_stream_listener[TEST_CH_SYRINGE].context);
    expect_u32(
        test_flags_set_value,
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED,
        "a released ring half of the syringe wakes the task"
    );
    expect_u32(step_generator_task_instance.halted, 0U, "a released half of the syringe does not halt the task");
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_SYRINGE].on_fault(
        test_stream_listener[TEST_CH_SYRINGE].context,
        STEP_STREAM_AXIS_FAULT_UNDERRUN
    );
    expect_u32(step_generator_task_instance.halted, 1U, "a fault of the syringe halts feeding");
    expect_u32(step_stream_group_get_fault_member(test_group()), TEST_MEMBER_SYRINGE, "the fault is that of member 3");
    expect_u32(
        step_stream_group_get_fault(test_group()),
        (uint32_t)STEP_STREAM_AXIS_FAULT_UNDERRUN,
        "the group keeps the fault of the syringe"
    );
    expect_u32(step_generator_task_instance.halt_member, TEST_MEMBER_SYRINGE, "the halt is blamed on member 3");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT,
        "the fault of the syringe is a group fault"
    );
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the channel of X is stopped for the fault of the syringe");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "the channel of Y is stopped for the fault of the syringe");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "the channel of Z is stopped for the fault of the syringe");
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 0U, "the failed channel of the syringe stopped itself");

    /* The syringe brakes with the axes when the group does: with no switch reported it is the braking member. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    test_stream_listener[TEST_CH_SYRINGE].on_brake(test_stream_listener[TEST_CH_SYRINGE].context);
    expect_u32(step_generator_task_instance.halted, 1U, "braking of the syringe halts feeding");
    expect_u32(step_generator_task_instance.halt_member, TEST_MEMBER_SYRINGE, "braking of the syringe is blamed on member 3");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT,
        "braking of the syringe is named as a limit"
    );

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

    step_generator_task_halt_all();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halted, 1U, "STOP halts");
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "STOP is named as the cause");
    expect_u32(stream.halt_member, STEP_GENERATOR_TASK_NO_MEMBER, "STOP blames no channel");
    expect_u32(stream.halts_total, 1U, "STOP is counted");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_limit_listeners[MOTION_AXIS_Y](
        test_limit_contexts[MOTION_AXIS_Y],
        MOTION_AXIS_Y,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    /* The group halts the feeding once a channel reports that it brakes, not at the switch itself. */
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "a switch is named as the cause");
    expect_u32(stream.halt_member, 1U, "the switch of Y is member 1");
    expect_u32(stream.members[0].limit_trips, 0U, "X reported no switch");
    expect_u32(stream.members[1].limit_trips, 1U, "Y reported one switch");
    expect_u32(stream.members[2].limit_trips, 0U, "Z reported no switch");

    /* MotionTask ends the move after the switch stopped it: that second halt must not hide the switch. */
    step_generator_task_halt_all();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "a later halt keeps the first cause");
    expect_u32(stream.halt_member, 1U, "a later halt keeps the first member");
    expect_u32(stream.halts_total, 2U, "a later halt is still counted");

    /* After the reset the next halt records its own cause. */
    step_generator_task_instance.halted = 0U;
    step_generator_task_halt_all();
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(
        stream.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL,
        "the first halt after a reset sets the cause"
    );
    expect_u32(stream.halt_member, STEP_GENERATOR_TASK_NO_MEMBER, "and its member");
    expect_u32(stream.halts_total, 3U, "three halts in all");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
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
    expect_u32(
        stream.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED,
        "a missing feeder is named as the cause"
    );

    /* A channel that still runs after the reset keeps the task halted. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_wait_actions[0] = TEST_ACTION_Y_STREAMING;
    (void)run_task_until_jump(&test_task_config);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(
        stream.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED,
        "a reset that fails is named as the cause"
    );

    /* A STOP reset watches every axis again; a stream that moves only Z watches only Z. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_watched_axes = 0x01U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_watched_axes, TEST_ALL_AXES_WATCHED, "the reset watches every axis again");
    step_generator_task_instance.feeder.direction_known_mask = MOTION_PLANNER_AXIS_Z_MASK;
    step_generator_task_watch_moving_axes(&step_generator_task_instance);
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Z, "a stream that moves only Z watches only Z");

    /* A limit of Z is member 2: it is counted for Z only and named as the cause. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    test_stream_listener[TEST_CH_Z].on_brake(test_stream_listener[TEST_CH_Z].context);
    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
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
        stream.members[2].hardware_stop_reason,
        (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
        "Z stop reason"
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
        stream.members[0].hardware_stop_reason,
        (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
        "X stop reason"
    );
    expect_u32(
        stream.members[1].hardware_stop_reason,
        (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
        "Y stop reason"
    );
}

/* The diagnostic frame lists the three axes; the syringe, member 3 of the instance, writes no entry of its own. */
static void test_stream_diagnostics_list_only_the_axes(void) {
    reset_fixture();
    /* The frame is followed by a canary: a write of a fourth entry would overwrite it. */
    struct {
        StepGeneratorTaskStreamDiagnostics_t frame;
        StepGeneratorTaskMemberDiagnostics_t canary;
    } framed;
    (void)memset(&framed, 0xA5, sizeof(framed));
    expect_u32(
        (uint32_t)((const char *)&framed.canary - (const char *)&framed.frame.members[3]),
        0U,
        "the canary sits right behind the third entry"
    );
    /* The syringe has a state of its own that differs from the others, so that a write of its entry would show. */
    test_axis_streaming[TEST_CH_SYRINGE] = 1U;
    test_axis_fault[TEST_CH_SYRINGE] = STEP_STREAM_AXIS_FAULT_DMA;
    step_generator_task_instance.limit_trips[TEST_MEMBER_SYRINGE] = 7U;
    step_generator_task_get_stream_diagnostics(&framed.frame);
    const unsigned char *const canary_bytes = (const unsigned char *)&framed.canary;
    uint32_t canary_intact = 1U;
    for (uint32_t index = 0U; index < sizeof(framed.canary); ++index) {
        if (canary_bytes[index] != 0xA5U) {
            canary_intact = 0U;
        }
    }
    expect_u32(canary_intact, 1U, "the syringe writes nothing behind the entries of the axes");
    expect_u32(framed.frame.members[0].channel_state, (uint32_t)STEP_STREAM_AXIS_STATE_IDLE, "X entry is written");
    expect_u32(framed.frame.members[2].limit_trips, 0U, "Z entry is written");

    /* A fault of the syringe is blamed on member 3 in the frame, and on no axis in the progress. */
    reset_fixture();
    const uint32_t fault = (uint32_t)STEP_STREAM_AXIS_FAULT_DMA;
    step_generator_task_group_fault(&step_generator_task_instance, TEST_MEMBER_SYRINGE, fault);
    StepGeneratorTaskStreamDiagnostics_t stream = {0};
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "a ring fault is the cause");
    expect_u32(stream.halt_member, TEST_MEMBER_SYRINGE, "the frame names member 3");
    expect_u32(stream.halt_fault, fault, "the fault code is kept");
    const StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "the syringe is no axis to blame");
    expect_u32(progress.halt_detail, fault, "the detail is the fault code");
}

static void test_limit_switch_brakes_through_the_group(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_limit_listener_calls[MOTION_AXIS_X], 0U, "no switch listener is taken before a stream opens");
    test_claim_print_channels();
    expect_u32(test_limit_listener_calls[MOTION_AXIS_X], 1U, "the claim registers the limit listener of X once");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_Y], 1U, "the claim registers the limit listener of Y once");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_Z], 1U, "the claim registers the limit listener of Z once");
    for (uint32_t axis = 0U; axis < MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_true(
            test_limit_listeners[axis] == step_generator_task_limit_reached,
            "the limit handler of the generator is installed for the axis"
        );
        expect_true(
            test_limit_contexts[axis] == &step_generator_task_instance,
            "the switch of the axis reports to the print instance"
        );
    }

    run_group_prepared_and_started();
    test_limit_listeners[MOTION_AXIS_X](
        test_limit_contexts[MOTION_AXIS_X],
        MOTION_AXIS_HARDWARE_COUNT,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(
        test_brake_requests[TEST_CH_X] + test_brake_requests[TEST_CH_Y] + test_brake_requests[TEST_CH_Z] +
            test_brake_requests[TEST_CH_SYRINGE],
        0U,
        "a switch of an axis without a channel brakes nothing"
    );
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_RUNNING,
        "a switch of an axis without a channel does not stop the group"
    );
    StepGeneratorTaskStreamDiagnostics_t no_channel = {0};
    step_generator_task_get_stream_diagnostics(&no_channel);
    expect_u32(
        no_channel.members[0].limit_trips + no_channel.members[1].limit_trips + no_channel.members[2].limit_trips +
            step_generator_task_instance.limit_trips[TEST_MEMBER_SYRINGE],
        0U,
        "a switch of an axis without a channel is counted for no member"
    );

    /* The group moves its axes together, so the switch of any axis brakes all channels. */
    test_limit_listeners[MOTION_AXIS_X](
        test_limit_contexts[MOTION_AXIS_X],
        MOTION_AXIS_X,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of X brakes the channel of X");
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the switch of X brakes the channel of Y too");
    expect_u32(test_brake_requests[TEST_CH_Z], 1U, "the switch of X brakes the channel of Z too");
    expect_u32(test_brake_requests[TEST_CH_SYRINGE], 1U, "the switch of X brakes the channel of the syringe too");
    expect_u32(
        step_generator_task_instance.limit_trips[TEST_MEMBER_SYRINGE],
        0U,
        "the switch of an axis is not counted for the syringe"
    );
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group stops when a switch is reached"
    );
    expect_u32(test_wrong_axis_calls, 0U, "the brake goes to the channels of X, Y and Z");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_limit_listeners[MOTION_AXIS_Y](
        test_limit_contexts[MOTION_AXIS_Y],
        MOTION_AXIS_Y,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of Y brakes the channel of X");
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the switch of Y brakes the channel of Y");
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group stops when the switch of Y is reached"
    );
    expect_u32(test_brake_requests[TEST_CH_Z], 1U, "the switch of Y brakes the channel of Z");

    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of Z brakes the channel of X");
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the switch of Z brakes the channel of Y");
    expect_u32(test_brake_requests[TEST_CH_Z], 1U, "the switch of Z brakes the channel of Z");
    expect_u32(test_brake_requests[TEST_CH_SYRINGE], 1U, "the switch of Z brakes the channel of the syringe too");
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group stops when the switch of Z is reached"
    );
}

/* The switch that fired first is the cause of the halt, not the channel that happened to brake first. */
static void test_halt_member_is_the_axis_whose_switch_fired(void) {
    StepGeneratorTaskStreamDiagnostics_t stream = {0};

    /* Z fires its end switch, but the channel of Y is the first to report braking. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    expect_u32(step_generator_task_instance.limit_member, STEP_GENERATOR_TASK_NO_MEMBER, "no switch yet, no member");
    expect_u32(step_generator_task_instance.limit_reason, 0U, "no switch yet, no reason");
    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "braking after a switch is named as a limit");
    expect_u32(stream.halt_member, 2U, "the halt is blamed on Z whose switch fired, not on Y that braked first");
    expect_u32(step_generator_task_instance.limit_member, 2U, "the instance latches member 2");
    expect_u32(step_generator_task_instance.limit_reason, 3U, "the instance latches the END_LIMIT reason");

    /* A second switch of another axis does not replace the first one, but it is counted. */
    test_limit_listeners[MOTION_AXIS_Y](
        test_limit_contexts[MOTION_AXIS_Y],
        MOTION_AXIS_Y,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT
    );
    expect_u32(step_generator_task_instance.limit_member, 2U, "the first switch keeps the member");
    expect_u32(step_generator_task_instance.limit_reason, 3U, "the first switch keeps the reason");
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_member, 2U, "the halt stays blamed on Z");
    expect_u32(stream.members[0].limit_trips, 0U, "X reported no switch");
    expect_u32(stream.members[1].limit_trips, 1U, "Y switch is counted");
    expect_u32(stream.members[2].limit_trips, 1U, "Z switch is counted");

    /* A brake that the task asked for itself, with no switch, is blamed on the channel that brakes. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "braking without a switch is named as a limit");
    expect_u32(stream.halt_member, 1U, "without a switch the braking channel Y is blamed");
    expect_u32(step_generator_task_instance.limit_member, STEP_GENERATOR_TASK_NO_MEMBER, "no switch, no latched member");
    expect_u32(step_generator_task_instance.limit_reason, 0U, "no switch, no latched reason");

    /* The reset after STOP forgets the latched switch; the next one is recorded anew. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(step_generator_task_instance.limit_member, 2U, "the switch of Z is latched before the reset");
    step_generator_task_instance.reset_requested = 1U;
    step_generator_task_run_instance(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.limit_member, STEP_GENERATOR_TASK_NO_MEMBER, "the reset clears the member");
    expect_u32(step_generator_task_instance.limit_reason, 0U, "the reset clears the reason");

    /* A switch is latched only while the group runs, and the fake channels of this fixture keep the group from resetting
     * to idle, so the next switch is recorded in a fresh fixture, whose latch is as empty as one after the reset. */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
    run_group_prepared_and_started();
    step_generator_task_limit_reached(
        &step_generator_task_instance,
        MOTION_AXIS_Y,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT
    );
    expect_u32(step_generator_task_instance.limit_member, 1U, "the next switch, Y, is latched after the reset");
    expect_u32(step_generator_task_instance.limit_reason, 2U, "the next switch keeps its START_LIMIT reason");
}

static void prepare_print_fixture(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_claim_print_channels();
}

static void report_limit(MotionAxis_t axis, MotionAxisHardwareStopReason_t reason) {
    step_generator_task_limit_reached(&step_generator_task_instance, axis, reason);
}

static void expect_latch(uint32_t member, uint32_t reason, const char *message) {
    expect_u32(step_generator_task_instance.limit_member, member, message);
    expect_u32(step_generator_task_instance.limit_reason, reason, message);
}

/* A switch that finds the group IDLE brakes nothing, so it must not be blamed for a later brake. */
static void test_limit_in_idle_group_is_counted_but_not_latched(void) {
    StepGeneratorTaskStreamDiagnostics_t stream = {0};

    prepare_print_fixture();
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_IDLE, "the group is IDLE");
    report_limit(MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.members[2].limit_trips, 1U, "the switch of Z in an IDLE group is counted");
    expect_latch(STEP_GENERATOR_TASK_NO_MEMBER, 0U, "the switch in an IDLE group is not latched");
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_IDLE, "an IDLE group stays IDLE");

    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_member, 1U, "the later brake is blamed on Y that braked, not on Z of the IDLE switch");
}

/* PRIMING: switches are queued by the group and the latch is written; the order of reasons is checked here, because a
 * RUNNING group goes to STOPPING at the first switch. */
static void test_limit_in_priming_group_latches_and_real_limit_outranks_release(void) {
    StepGeneratorTaskStreamDiagnostics_t stream = {0};

    prepare_print_fixture();
    expect_u32(step_stream_group_prepare(test_group()), STEP_STREAM_GROUP_STATUS_OK, "the group prepares");
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_PRIMING, "the group is PRIMING");
    report_limit(MOTION_AXIS_Y, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE);
    expect_latch(1U, 4U, "a release of Y is latched in PRIMING");
    report_limit(MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE);
    expect_latch(1U, 4U, "a second release does not replace the latched one");
    report_limit(MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
    expect_latch(2U, 3U, "a real END_LIMIT of Z replaces the latched release");
    report_limit(MOTION_AXIS_Y, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT);
    expect_latch(2U, 3U, "a real START_LIMIT of Y does not replace a real limit");
    report_limit(MOTION_AXIS_X, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE);
    expect_latch(2U, 3U, "a release does not replace a real limit");
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.members[0].limit_trips, 1U, "the release of X is counted");
    expect_u32(stream.members[1].limit_trips, 2U, "two switches of Y are counted");
    expect_u32(stream.members[2].limit_trips, 2U, "two switches of Z are counted");
}

/* A real switch replaces a latched release in PRIMING, and the replaced one is the cause of the later halt. */
static void test_limit_in_priming_group_blames_the_real_limit_on_halt(void) {
    StepGeneratorTaskStreamDiagnostics_t stream = {0};

    prepare_print_fixture();
    expect_u32(step_stream_group_prepare(test_group()), STEP_STREAM_GROUP_STATUS_OK, "the group prepares");
    report_limit(MOTION_AXIS_Y, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE);
    report_limit(MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
    expect_u32(step_stream_group_start(test_group(), 2500U), STEP_STREAM_GROUP_STATUS_OK, "the group starts");
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the queued switches stop the group at start"
    );
    test_stream_listener[TEST_CH_X].on_brake(test_stream_listener[TEST_CH_X].context);
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.halt_member, 2U, "the halt is blamed on Z whose real limit replaced the release of Y");
}

/* RUNNING: the first switch is latched, however weak, and then the group is STOPPING. */
static void test_limit_in_running_group_latches_a_release(void) {
    prepare_print_fixture();
    run_group_prepared_and_started();
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_RUNNING, "the group is RUNNING");
    report_limit(MOTION_AXIS_Y, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE);
    expect_latch(1U, 4U, "a release of Y is latched in a RUNNING group");
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_STOPPING, "the release stops it");
    /* The group is STOPPING now: a later switch, even a real one, is not latched. */
    report_limit(MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
    expect_latch(1U, 4U, "a real limit in a STOPPING group does not replace the latch");
}

/* STOPPING and FAULT without a latch: the switch is counted, nothing is blamed. */
static void test_limit_in_stopping_or_faulted_group_is_not_latched(void) {
    StepGeneratorTaskStreamDiagnostics_t stream = {0};

    prepare_print_fixture();
    run_group_prepared_and_started();
    step_stream_group_on_limit(test_group(), 0U);
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_STOPPING, "the group is STOPPING");
    report_limit(MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
    expect_latch(STEP_GENERATOR_TASK_NO_MEMBER, 0U, "a switch in a STOPPING group is not latched");
    step_generator_task_get_stream_diagnostics(&stream);
    expect_u32(stream.members[2].limit_trips, 1U, "the switch in a STOPPING group is counted");

    prepare_print_fixture();
    run_group_prepared_and_started();
    step_stream_group_on_member_fault(test_group(), 0U, 1U);
    expect_u32(step_stream_group_get_state(test_group()), STEP_STREAM_GROUP_STATE_FAULT, "the group is FAULT");
    report_limit(MOTION_AXIS_Y, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT);
    expect_latch(STEP_GENERATOR_TASK_NO_MEMBER, 0U, "a switch in a faulted group is not latched");
}

static void test_group_ends_with_its_stream(void) {
    reset_fixture();
    test_wait_script_count = 2U;
    test_wait_actions[0] = TEST_ACTION_RUN_GROUP_THEN_END;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_stream_group_get_state(test_group()),
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
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_IDLE,
        "the reset after STOP clears the fault of the group"
    );
    expect_u32(step_stream_group_get_fault(test_group()), 0U, "the reset forgets the fault");
    expect_u32(step_generator_task_instance.halted, 0U, "feeding goes on after the reset");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 0U, "a clean reset is no error");
    expect_u32(step_generator_task_instance.channels_held, 0U, "a clean reset lets go of the channels");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_true(test_stream_owner[channel] == NULL, "the channel is free after the reset");
    }
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_true(test_limit_contexts[channel] == NULL, "the switch is free after the reset");
    }
}

static void test_reset_is_refused_while_a_channel_runs(void) {
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_actions[0] = TEST_ACTION_Y_STREAMING;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_task_instance.diagnostics.axis_errors,
        1U,
        "a channel of Y that still runs is an axis error"
    );
    expect_u32(step_generator_task_instance.halted, 1U, "nothing is fed while a channel runs after STOP");

    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_actions[0] = TEST_ACTION_Z_STREAMING;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_task_instance.diagnostics.axis_errors,
        1U,
        "a channel of Z that still runs is an axis error"
    );
    expect_u32(step_generator_task_instance.halted, 1U, "nothing is fed while the channel of Z runs after STOP");

    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_actions[0] = TEST_ACTION_SYRINGE_STREAMING;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_task_instance.diagnostics.axis_errors,
        1U,
        "a channel of the syringe that still runs is an axis error"
    );
    expect_u32(step_generator_task_instance.halted, 1U, "nothing is fed while the channel of the syringe runs after STOP");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED,
        "the cause is the failed reset"
    );
}

static void test_brake_ramp_is_built_from_the_driver_timing(void) {
    reset_fixture();
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_configure_brake_calls[TEST_CH_X], 1U, "the braking ramp of X is configured once");
    expect_u32(test_configure_brake_calls[TEST_CH_Y], 1U, "the braking ramp of Y is configured once");
    expect_u32(test_configure_brake_calls[TEST_CH_Z], 1U, "the braking ramp of Z is configured once");
    expect_u32(
        test_configure_brake_calls[TEST_CH_SYRINGE],
        1U,
        "the braking ramp of the syringe is configured once: the group brakes it with the axes"
    );
    expect_u32(test_brake_config.timer_clock_hz, TEST_TIMER_CLOCK_HZ, "ramp uses the timer clock");
    expect_u32(test_brake_config.pulse_ticks, 1375U, "ramp uses the STEP pulse of the driver");
    expect_u32(test_brake_config.min_period_ticks, 2063U, "ramp keeps pulse and low time");
    expect_u32(test_brake_config.idle_record_ticks, 2750U, "ramp uses the filler record of the feeder");
    expect_u32(test_brake_config.deceleration_steps_per_s2, 600000U, "ramp uses the limit deceleration");
    expect_u32(
        test_brake_config.max_step_rate_hz,
        STEP_GENERATOR_TASK_BRAKE_MAX_RATE_HZ,
        "ramp starts at the seek rate"
    );
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "a good ramp is no error");

    /* A longer shortest record than pulse plus low time wins. */
    reset_fixture();
    test_task_config.step_pulse_width_ns = 400U;
    test_task_config.step_low_time_ns = 400U;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_brake_config.min_period_ticks, 550U, "the shortest record sets the ramp period");
    expect_u32(test_brake_config.pulse_ticks, 110U, "ramp pulse follows the driver");

    /* Every instance builds the ramps of its own members: with all of them set up, each axis channel gets one ramp from
     * the print instance and one from the instance of its axis, the syringe only the one of the print instance. */
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    init_axis_plan_buffer(MOTION_AXIS_Y);
    init_axis_plan_buffer(MOTION_AXIS_Z);
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_ready(step_generator_task_get_axis_instance(MOTION_AXIS_Y)), 1U, "Y instance is ready");
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_u32(test_configure_brake_calls[channel], 2U, "an axis channel gets a ramp from the print instance and its own");
    }
    expect_u32(test_configure_brake_calls[TEST_CH_SYRINGE], 1U, "the syringe gets the ramp of the print instance only");
}

/* The print instance cannot be built (the syringe is on another clock): the instances of the axes still build the ramps
 * of their own channels, so the homing of an axis keeps its braking. */
static void test_axis_instances_build_their_ramps_when_the_print_instance_cannot_be_built(void) {
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    init_axis_plan_buffer(MOTION_AXIS_Y);
    init_axis_plan_buffer(MOTION_AXIS_Z);
    test_timer_clock_hz[TEST_CH_SYRINGE] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_ready(step_generator_task_get_print_instance()), 0U, "the print instance is not ready");
    for (uint32_t axis = 0U; axis < TEST_CHANNELS; ++axis) {
        expect_u32(
            step_generator_instance_is_ready(step_generator_task_get_axis_instance((MotionAxis_t)axis)),
            1U,
            "the instance of the axis is ready"
        );
        expect_u32(test_configure_brake_calls[axis], 1U, "the instance of the axis builds the ramp of its channel once");
    }
    expect_u32(test_configure_brake_calls[TEST_CH_SYRINGE], 0U, "no instance owns the syringe but the print instance");
    expect_u32(test_brake_config.timer_clock_hz, TEST_TIMER_CLOCK_HZ, "the ramps use the clock of the axes");
    expect_u32(test_brake_config.min_period_ticks, 2063U, "the ramp keeps pulse and low time");
}

static void test_brake_is_skipped_or_counted(void) {
    reset_fixture();
    test_task_config.limit_deceleration_steps_per_s2 = 0U;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        test_configure_brake_calls[TEST_CH_X] + test_configure_brake_calls[TEST_CH_Y] +
            test_configure_brake_calls[TEST_CH_Z] + test_configure_brake_calls[TEST_CH_SYRINGE],
        0U,
        "no deceleration setting, no ramp"
    );
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "a missing ramp is not an error here");

    reset_fixture();
    test_configure_brake_result = STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    test_wait_script_count = 1U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "a ramp the axes refuse is counted once");
    expect_u32(step_generator_task_instance.halted, 0U, "feeding goes on without a ramp");

    /* The error count adds to the feeder's own and saturates with it. */
    step_generator_task_instance.feeder.diagnostics.generator_errors = 4U;
    step_generator_task_publish_diagnostics(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 5U, "ramp error adds to the feeder errors");
    step_generator_task_instance.feeder.diagnostics.generator_errors = UINT32_MAX;
    step_generator_task_publish_diagnostics(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, UINT32_MAX, "the sum saturates");

    /* Resets that left a channel running add to the axis errors of the feeder and saturate with them. */
    step_generator_task_instance.feeder.diagnostics.axis_errors = 6U;
    step_generator_task_instance.reset_failures = 2U;
    step_generator_task_publish_diagnostics(&step_generator_task_instance);
    expect_u32(
        step_generator_task_instance.diagnostics.axis_errors,
        8U,
        "reset failures add to the feeder axis errors"
    );
    step_generator_task_instance.feeder.diagnostics.axis_errors = UINT32_MAX;
    step_generator_task_publish_diagnostics(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, UINT32_MAX, "the axis error sum saturates");

    /* An unusable timing builds no feeder, so no ramp either. */
    reset_fixture();
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = 0U;
    }
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        test_configure_brake_calls[TEST_CH_X] + test_configure_brake_calls[TEST_CH_Y] +
            test_configure_brake_calls[TEST_CH_Z] + test_configure_brake_calls[TEST_CH_SYRINGE],
        0U,
        "no feeder, no ramp"
    );
}

static void test_callbacks_act_on_their_own_instance(void) {
    reset_fixture();
    StepGeneratorInstance_t other;
    make_other_instance(&other);

    /* A limit report with the context of the other instance counts there and brakes the group of that instance only. */
    other.group.state = STEP_STREAM_GROUP_STATE_RUNNING;
    step_generator_task_limit_reached(&other, MOTION_AXIS_Y, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
    expect_u32(other.limit_trips[1], 1U, "the limit is counted in the instance of the context");
    expect_u32(other.limit_trips[0] + other.limit_trips[2], 0U, "only the member of the axis is counted");
    expect_u32(
        step_generator_task_instance.limit_trips[0] + step_generator_task_instance.limit_trips[1] +
            step_generator_task_instance.limit_trips[2],
        0U,
        "the printing instance does not see the limit of the other"
    );
    expect_u32(
        step_stream_group_get_state(&other.group),
        STEP_STREAM_GROUP_STATE_STOPPING,
        "the group of the other instance brakes on its limit"
    );
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_IDLE,
        "the group of printing ignores the limit of the other"
    );

    /* A channel that brakes reports to the group of its own instance, which halts that instance. */
    reset_fixture();
    make_other_instance(&other);
    other.group.state = STEP_STREAM_GROUP_STATE_RUNNING;
    other.stream_listeners[1].on_brake(other.stream_listeners[1].context);
    expect_u32(other.halted, 1U, "a braking channel halts its own instance");
    expect_u32(other.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "the other instance names the limit");
    expect_u32(other.halt_member, 1U, "the halt names the braking member");
    expect_u32(step_generator_task_instance.halted, 0U, "the printing instance does not halt on the other brake");
    expect_u32(step_generator_task_instance.halts_total, 0U, "the printing instance counts no halt of the other");

    /* A channel fault reaches the group and the halt of the instance behind the listener context. */
    reset_fixture();
    make_other_instance(&other);
    other.group.state = STEP_STREAM_GROUP_STATE_RUNNING;
    other.stream_listeners[2].on_fault(other.stream_listeners[2].context, STEP_STREAM_AXIS_FAULT_SEQUENCE);
    expect_u32(
        step_stream_group_get_state(&other.group),
        STEP_STREAM_GROUP_STATE_FAULT,
        "the group of the other faults"
    );
    expect_u32(step_stream_group_get_fault_member(&other.group), 2U, "the fault names the member of the listener");
    expect_u32(other.halted, 1U, "the other instance halts");
    expect_u32(other.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "the other instance names the fault");
    expect_u32(other.halt_member, 2U, "the halt names the member of the listener");
    expect_u32(other.halt_fault, (uint32_t)STEP_STREAM_AXIS_FAULT_SEQUENCE, "the halt keeps the fault");
    expect_u32(
        step_stream_group_get_state(test_group()),
        STEP_STREAM_GROUP_STATE_IDLE,
        "the group of printing is idle"
    );
    expect_u32(step_generator_task_instance.halted, 0U, "the printing instance goes on");
    expect_u32(step_generator_task_instance.halts_total, 0U, "the printing instance counts no halt");
}

/* The config names the queue; the task reads that one and no other. */
static void test_config_without_plan_buffer_halts_the_task(void) {
    reset_fixture();
    push_block(5U, 3);
    test_task_config.plan_buffer = NULL;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "no stream without a queue");
    expect_u32(test_start_calls[TEST_CH_X], 0U, "no channel starts without a queue");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "the missing queue is counted");
    expect_u32(step_generator_task_instance.halted, 1U, "the task is halted without a queue");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED,
        "the missing queue is named as a failed setup"
    );
    expect_true(step_generator_task_instance.plan_buffer == NULL, "the task keeps no queue the config did not name");
    /* The task has no queue to look at, so the block of the fixture's buffer is not its work. */
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 0U, "a task without a queue is not busy");
    PlanBufferDiagnostics_t plan_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan_diagnostics);
    expect_u32(plan_diagnostics.queued, 1U, "the block of the other queue stays untouched");
    expect_u32(plan_diagnostics.dequeued_total, 0U, "nothing is taken from the other queue");
}

static void test_ports_read_the_queue_of_the_config(void) {
    static PlanBuffer_t config_buffer;
    static MotionPlanBlock_t config_buffer_entries[4];
    reset_fixture();
    (void)memset(&config_buffer, 0, sizeof(config_buffer));
    expect_u32(
        plan_buffer_init(&config_buffer, config_buffer_entries, 4U),
        PLAN_BUFFER_STATUS_OK,
        "the queue of the config initializes"
    );
    /* One clear gives it epoch 1; the fixture's queue stays at epoch 0. */
    expect_u32(plan_buffer_clear(&config_buffer), PLAN_BUFFER_STATUS_OK, "the queue of the config clears");
    const MotionPlanBlock_t config_block = make_block(6U, 3, 0);
    expect_u32(plan_buffer_try_push(&config_buffer, &config_block), PLAN_BUFFER_STATUS_OK, "a block queues in the config queue");
    push_block(7U, 3);

    test_task_config.plan_buffer = &config_buffer;
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");

    expect_true(step_generator_task_instance.plan_buffer == &config_buffer, "the task takes the queue of the config");
    expect_u32(step_generator_task_port_plan_epoch(&step_generator_task_instance), 1U, "the epoch port reads the config queue");
    expect_u32(plan_buffer_get_epoch(&test_plan_buffer), 0U, "the other queue was never cleared");
    expect_u32(test_start_calls[TEST_CH_X], 1U, "the block of the config queue is streamed");
    expect_u32(step_generator_task_instance.diagnostics.blocks_started, 1U, "exactly one block was started");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "a queue in the config is no error");

    PlanBufferDiagnostics_t config_diagnostics = {0};
    PlanBufferDiagnostics_t other_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&config_buffer, &config_diagnostics);
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &other_diagnostics);
    expect_u32(config_diagnostics.queued, 0U, "the streamed block left the config queue");
    expect_u32(config_diagnostics.dequeued_total, 1U, "the config queue gave exactly its block");
    expect_u32(other_diagnostics.queued, 1U, "the block of the other queue is not taken");
    expect_u32(other_diagnostics.dequeued_total, 0U, "nothing is discarded from the other queue");
}

static const StepGeneratorMember_t test_owner_members_x[] = {
    {&step_generator_channels_axis_driver, MOTION_AXIS_X, MOTION_AXIS_X},
};
static const StepGeneratorMember_t test_owner_members_yz[] = {
    {&step_generator_channels_axis_driver, MOTION_AXIS_Y, MOTION_AXIS_Y},
    {&step_generator_channels_axis_driver, MOTION_AXIS_Z, MOTION_AXIS_Z},
};

/* Two instances hold different axes at the same time: the switch of an axis reaches only the instance that owns it. */
static void test_two_instances_own_different_axes(void) {
    reset_fixture();
    StepGeneratorInstance_t first;
    StepGeneratorInstance_t second;
    make_other_instance(&first);
    make_other_instance(&second);
    first.members = test_owner_members_x;
    first.member_count = 1U;
    second.members = test_owner_members_yz;
    second.member_count = 2U;

    expect_u32(step_generator_task_claim_channels(&first), 1U, "the first instance takes the channel of X");
    expect_u32(step_generator_task_claim_channels(&second), 1U, "the second instance takes the channels of Y and Z");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == &first, "the switch of X goes to the first instance");
    expect_true(test_limit_contexts[MOTION_AXIS_Y] == &second, "the switch of Y goes to the second instance");
    expect_true(test_limit_contexts[MOTION_AXIS_Z] == &second, "the switch of Z goes to the second instance");
    expect_true(
        test_limit_listeners[MOTION_AXIS_Y] == step_generator_task_limit_reached,
        "the owner of Y gets the limit handler of the generator"
    );
    expect_true(
        test_stream_listener[TEST_CH_X].context == &first.member_contexts[0],
        "the channel of X reports to the first instance"
    );
    expect_true(
        test_stream_listener[TEST_CH_Z].context == &second.member_contexts[1],
        "the channel of Z reports to the second instance as its member 1"
    );

    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(second.limit_trips[1], 1U, "the limit of Z is counted as member 1 of the second instance");
    expect_u32(second.limit_trips[0], 0U, "Y of the second instance saw no limit");
    expect_u32(first.limit_trips[0], 0U, "the first instance does not see the limit of Z");
}

static int test_foreign_context;
static void test_foreign_limit(void *context, MotionAxis_t axis, MotionAxisHardwareStopReason_t reason) {
    (void)context;
    (void)axis;
    (void)reason;
}

/* Gives the one-lane instance of the axis a queue of its own in the config. */
static void init_axis_plan_buffer(MotionAxis_t axis) {
    expect_u32(
        plan_buffer_init(&test_axis_plan_buffer[axis], test_axis_plan_entries[axis], PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "axis plan buffer initializes"
    );
    test_task_config.axis_plan_buffers[axis] = &test_axis_plan_buffer[axis];
}

/* A block of one axis alone: `steps` steps of the axis whose planner axis number is the MotionAxis_t. */
static void push_axis_block(MotionAxis_t axis, uint32_t sequence, int64_t steps) {
    int64_t delta[MOTION_AXIS_HARDWARE_COUNT] = {0, 0, 0};
    delta[axis] = steps;
    const MotionPlanBlock_t block = make_block_xyz(sequence, delta[0], delta[1], delta[2]);
    expect_u32(
        plan_buffer_try_push(&test_axis_plan_buffer[axis], &block),
        PLAN_BUFFER_STATUS_OK,
        "axis block is queued"
    );
}

/* The task runs one pass over every instance and stops at the second wait. */
static void run_one_service_pass(void) {
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
}

/* The instances of X and Y each play a block of their own queue: X forward, Y backward. */
static void start_streams_of_x_and_y(void) {
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    init_axis_plan_buffer(MOTION_AXIS_Y);
    push_axis_block(MOTION_AXIS_X, 60U, 3);
    push_axis_block(MOTION_AXIS_Y, 61U, -3);
    test_watched_axes = 0U;
    run_one_service_pass();
}

/* HOME of Y: the instance of Y plays its block on one lane of planner axis 1 and touches nothing else. */
static void test_axis_instance_plays_a_block_on_its_own_lane(void) {
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_Y);
    push_axis_block(MOTION_AXIS_Y, 50U, 3);
    test_watched_axes = 0U;
    run_one_service_pass();
    const StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];

    expect_u32(test_prepare_calls[TEST_CH_Y], 1U, "the channel of Y is prepared");
    expect_u32(test_prepare_calls[TEST_CH_X], 0U, "the channel of X is not prepared by a block of Y");
    expect_u32(test_prepare_calls[TEST_CH_Z], 0U, "the channel of Z is not prepared by a block of Y");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the channel of Y starts");
    expect_u32(test_start_calls[TEST_CH_X], 0U, "the channel of X does not start");
    expect_u32(test_start_calls[TEST_CH_Z], 0U, "the channel of Z does not start");
    expect_u32(test_trigger_calls, 1U, "the one-lane group triggers once");
    expect_u32(test_trigger_axes, 1U, "the trigger covers only the channel of Y");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "the driver of Y is set once");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 1U, "a positive block sets the positive direction of Y");
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 0U, "the driver of X is not touched");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "the driver of Z is not touched");
    expect_u32(test_delay_calls, 1U, "DIR setup time is waited once");
    /* 3 steps of 10 us: the interval is 2750 ticks at 275 MHz, the record holds it minus one; the pulse is 5 us. */
    expect_u32(test_halves[TEST_CH_Y][0][0], 2749U, "the ring of Y holds the step interval");
    expect_u32(test_halves[TEST_CH_Y][0][2], 1375U, "the ring of Y holds the pulse");
    expect_u32(test_halves[TEST_CH_X][0][0], 0U, "the ring of X is not written");
    expect_u32(test_halves[TEST_CH_X][0][2], 0U, "the ring of X has no pulse");
    expect_u32(test_halves[TEST_CH_Z][0][0], 0U, "the ring of Z is not written");
    expect_u32(test_halves[TEST_CH_Z][0][2], 0U, "the ring of Z has no pulse");
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Y, "only the axis of the instance is watched");
    expect_u32(test_watched_axes_calls, 1U, "the watched state is set for the one member");

    expect_u32(axis_y->diagnostics.streams_started, 1U, "the instance of Y started one stream");
    expect_u32(axis_y->diagnostics.blocks_started, 1U, "the instance of Y started the block");
    expect_u32(axis_y->diagnostics.blocks_completed, 1U, "the instance of Y generated the whole block");
    expect_u32(axis_y->diagnostics.halves_filled, 2U, "both halves of the ring of Y are filled");
    expect_u32(axis_y->diagnostics.generator_errors, 0U, "no generator error on Y");
    expect_u32(axis_y->diagnostics.axis_errors, 0U, "no axis error on Y");
    expect_u32(axis_y->halted, 0U, "the instance of Y feeds");
    expect_u32(step_generator_instance_is_busy(axis_y), 1U, "the instance of Y is busy with its stream");
    expect_u32(step_generator_instance_is_busy(&step_generator_task_axis_instances[MOTION_AXIS_X]), 0U, "X is not busy");
    expect_u32(step_generator_instance_is_busy(&step_generator_task_axis_instances[MOTION_AXIS_Z]), 0U, "Z is not busy");
    expect_u32(step_generator_instance_is_busy(&step_generator_task_instance), 0U, "the print instance is not busy");

    PlanBufferDiagnostics_t queue = {0};
    (void)plan_buffer_get_diagnostics(&test_axis_plan_buffer[MOTION_AXIS_Y], &queue);
    expect_u32(queue.dequeued_total, 1U, "the block left the queue of Y");
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &queue);
    expect_u32(queue.dequeued_total, 0U, "the print queue gave nothing");

    expect_u32(step_generator_task_instance.diagnostics.streams_started, 0U, "the print instance started no stream");
    expect_u32(
        step_stream_feeder_get_phase(&step_generator_task_instance.feeder),
        STEP_STREAM_FEEDER_PHASE_IDLE,
        "the print feeder stays idle"
    );
    expect_u32(step_generator_task_instance.channels_held, 0U, "the print instance holds no channel");
    expect_true(test_stream_owner[TEST_CH_Y] == test_axis_context(MOTION_AXIS_Y), "the instance of Y owns the channel of Y");
    expect_true(test_stream_owner[TEST_CH_X] == NULL, "the channel of X has no owner");
    expect_true(test_stream_owner[TEST_CH_Z] == NULL, "the channel of Z has no owner");
    expect_true(test_limit_contexts[MOTION_AXIS_Y] == axis_y, "the switch of Y reports to the instance of Y");
    expect_true(test_limit_listeners[MOTION_AXIS_X] == NULL, "the switch of X has no listener");
    expect_true(test_limit_listeners[MOTION_AXIS_Z] == NULL, "the switch of Z has no listener");
    expect_u32(
        test_listener_calls[TEST_CH_X] + test_listener_calls[TEST_CH_Z] + test_listener_calls[TEST_CH_SYRINGE],
        0U,
        "X, Z and the syringe are never offered a listener"
    );
    expect_u32(test_prepare_calls[TEST_CH_SYRINGE] + test_start_calls[TEST_CH_SYRINGE], 0U, "the syringe is not touched");
    expect_u32(test_syringe_outputs_calls, 0U, "the DIR of the syringe is not written by a block of Y");
    /* The print instance builds the ramps of its four members; only the instance of Y, the one with a queue, builds one
     * more, for its own channel. */
    expect_u32(test_configure_brake_calls[TEST_CH_X], 1U, "the ramp of X is built by the print instance only");
    expect_u32(test_configure_brake_calls[TEST_CH_Y], 2U, "the ramp of Y is built by the print instance and by its own");
    expect_u32(test_configure_brake_calls[TEST_CH_Z], 1U, "the ramp of Z is built by the print instance only");
    expect_u32(test_configure_brake_calls[TEST_CH_SYRINGE], 1U, "the ramp of the syringe is built once");
}

/* X and Y stream at the same time, each with its own channel, switch listener and group. */
static void test_two_axis_instances_stream_together(void) {
    start_streams_of_x_and_y();
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];

    expect_u32(test_start_calls[TEST_CH_X], 1U, "the channel of X starts");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the channel of Y starts");
    expect_u32(test_start_calls[TEST_CH_Z], 0U, "the channel of Z does not start");
    expect_u32(test_trigger_calls, 2U, "each instance triggers its own group");
    expect_u32(test_trigger_axes, 1U, "each trigger covers one channel");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 1U, "X goes the positive way");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 0U, "Y goes the negative way");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 0U, "the driver of Z is not touched");
    expect_u32(test_halves[TEST_CH_X][0][2], 1375U, "the ring of X holds a pulse");
    expect_u32(test_halves[TEST_CH_Y][0][2], 1375U, "the ring of Y holds a pulse");
    expect_u32(test_watched_axes, (1U << MOTION_AXIS_X) | (1U << MOTION_AXIS_Y), "X and Y are watched, Z is not");
    expect_true(test_stream_owner[TEST_CH_X] == test_axis_context(MOTION_AXIS_X), "the instance of X owns its channel");
    expect_true(test_stream_owner[TEST_CH_Y] == test_axis_context(MOTION_AXIS_Y), "the instance of Y owns its channel");
    expect_true(test_stream_owner[TEST_CH_Z] == NULL, "the channel of Z is free");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == axis_x, "the switch of X reports to the instance of X");
    expect_true(test_limit_contexts[MOTION_AXIS_Y] == axis_y, "the switch of Y reports to the instance of Y");
    expect_u32(step_stream_group_get_state(&axis_x->group), STEP_STREAM_GROUP_STATE_RUNNING, "the group of X runs");
    expect_u32(step_stream_group_get_state(&axis_y->group), STEP_STREAM_GROUP_STATE_RUNNING, "the group of Y runs");

    /* A ring fault of X halts the instance of X; the member number is local to the instance. */
    test_stream_listener[TEST_CH_X].on_fault(test_stream_listener[TEST_CH_X].context, STEP_STREAM_AXIS_FAULT_SEQUENCE);
    expect_u32(axis_x->halted, 1U, "the fault of X halts the instance of X");
    expect_u32(axis_x->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "the instance of X names the fault");
    expect_u32(axis_x->halt_member, 0U, "the channel of X is member 0 of its instance");
    expect_u32(axis_x->halt_fault, (uint32_t)STEP_STREAM_AXIS_FAULT_SEQUENCE, "the fault code is kept");
    expect_u32(axis_y->halted, 0U, "the fault of X does not halt the instance of Y");
    expect_u32(step_generator_task_instance.halted, 0U, "the fault of X does not halt the print instance");
    expect_u32(step_stream_group_get_state(&axis_x->group), STEP_STREAM_GROUP_STATE_FAULT, "the group of X faults");
    expect_u32(step_stream_group_get_state(&axis_y->group), STEP_STREAM_GROUP_STATE_RUNNING, "the group of Y goes on");
    expect_u32(test_stop_calls[TEST_CH_Y], 0U, "the channel of Y is not stopped for the fault of X");

    /* A braking Y halts the instance of Y as its member 0 and leaves the cause of X alone. */
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    expect_u32(axis_y->halted, 1U, "the braking of Y halts the instance of Y");
    expect_u32(axis_y->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "the instance of Y names the limit");
    expect_u32(axis_y->halt_member, 0U, "the channel of Y is member 0 of its instance");
    expect_u32(axis_x->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "the cause of X is unchanged");
    expect_u32(step_generator_task_instance.halts_total, 0U, "the print instance counts no halt");
}

/* The switch of an axis reaches only the instance that owns the axis. */
static void test_limit_switch_reaches_only_its_instance(void) {
    start_streams_of_x_and_y();
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];

    test_limit_listeners[MOTION_AXIS_Y](
        test_limit_contexts[MOTION_AXIS_Y],
        MOTION_AXIS_Y,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(axis_y->limit_trips[0], 1U, "the switch of Y is counted by the instance of Y");
    expect_u32(axis_x->limit_trips[0], 0U, "the instance of X does not see the switch of Y");
    expect_u32(
        step_generator_task_instance.limit_trips[0] + step_generator_task_instance.limit_trips[1] +
            step_generator_task_instance.limit_trips[2],
        0U,
        "the print instance does not see the switch of Y"
    );
    expect_u32(test_brake_requests[TEST_CH_Y], 1U, "the channel of Y brakes");
    expect_u32(test_brake_requests[TEST_CH_X], 0U, "the channel of X goes on");
    expect_u32(test_brake_requests[TEST_CH_Z], 0U, "the channel of Z is not asked to brake");
    expect_u32(step_stream_group_get_state(&axis_y->group), STEP_STREAM_GROUP_STATE_STOPPING, "the group of Y stops");
    expect_u32(step_stream_group_get_state(&axis_x->group), STEP_STREAM_GROUP_STATE_RUNNING, "the group of X runs on");

    /* The listener of X knows only the axis of X: a report of another axis is not its member. */
    test_limit_listeners[MOTION_AXIS_X](
        test_limit_contexts[MOTION_AXIS_X],
        MOTION_AXIS_Y,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(axis_x->limit_trips[0], 0U, "a switch of another axis is not counted by the instance of X");
    expect_u32(test_brake_requests[TEST_CH_X], 0U, "a switch of another axis does not brake X");
    test_limit_listeners[MOTION_AXIS_X](
        test_limit_contexts[MOTION_AXIS_X],
        MOTION_AXIS_X,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    expect_u32(axis_x->limit_trips[0], 1U, "the switch of X is counted by the instance of X");
    expect_u32(test_brake_requests[TEST_CH_X], 1U, "the switch of X brakes the channel of X");
    expect_u32(axis_y->limit_trips[0], 1U, "the instance of Y did not count the switch of X");
}

/* A channel that another instance holds is refused: the stream cannot open and the owner keeps what it has. */
static void test_refused_claim_fails_the_stream_and_spares_the_owner(void) {
    /* X is held by the instance of X, which the task does not set up (no queue), so it keeps what it took. */
    reset_fixture();
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    expect_u32(step_generator_task_claim_channels(axis_x), 1U, "the instance of X takes its channel");
    push_block_xy(70U, 3, 3);
    run_one_service_pass();

    expect_u32(step_generator_task_instance.halted, 1U, "a refused claim halts the print instance");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
        "the refusal is a feeder error"
    );
    expect_u32(
        step_generator_task_instance.feeder_status,
        (uint32_t)STEP_STREAM_FEEDER_STATUS_AXIS_ERROR,
        "the feeder reports an axis error"
    );
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "the refusal is one axis error");
    expect_u32(step_generator_task_instance.diagnostics.streams_started, 0U, "no stream started");
    expect_u32(test_listener_calls[TEST_CH_X], 2U, "the channel of X was offered by its owner and by the print instance");
    expect_u32(test_listener_calls[TEST_CH_Y], 0U, "the members after the refused one are not asked");
    expect_u32(test_listener_calls[TEST_CH_Z], 0U, "Z is not asked either");
    expect_u32(test_listener_calls[TEST_CH_SYRINGE], 0U, "the syringe is not asked either");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_X], 1U, "the switch of X is asked once, by its owner");
    expect_u32(
        test_prepare_calls[TEST_CH_X] + test_prepare_calls[TEST_CH_Y] + test_prepare_calls[TEST_CH_Z] +
            test_prepare_calls[TEST_CH_SYRINGE],
        0U,
        "no ring is prepared"
    );
    expect_u32(
        test_start_calls[TEST_CH_X] + test_start_calls[TEST_CH_Y] + test_start_calls[TEST_CH_Z] +
            test_start_calls[TEST_CH_SYRINGE],
        0U,
        "no channel starts"
    );
    expect_u32(
        test_outputs_calls[MOTION_AXIS_X] + test_outputs_calls[MOTION_AXIS_Y] + test_outputs_calls[MOTION_AXIS_Z],
        0U,
        "no driver is set"
    );
    expect_true(test_stream_owner[TEST_CH_X] == test_axis_context(MOTION_AXIS_X), "the owner keeps the channel of X");
    expect_true(test_stream_owner[TEST_CH_Y] == NULL, "the channel of Y stays free");
    expect_true(test_stream_owner[TEST_CH_Z] == NULL, "the channel of Z stays free");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == axis_x, "the owner keeps the switch of X");
    expect_true(test_limit_listeners[MOTION_AXIS_X] == step_generator_task_limit_reached, "its handler is still installed");
    expect_u32(axis_x->channels_held, 1U, "the owner still holds its channels");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z],
        0U,
        "a refused instance stops no channel: they are not its own"
    );
    expect_u32(step_generator_task_instance.channels_held, 0U, "the refused instance holds nothing");

    /* The second member is refused: the channel of X that the print instance did take is let go, the owner of Y stays. */
    reset_fixture();
    StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];
    expect_u32(step_generator_task_claim_channels(axis_y), 1U, "the instance of Y takes its channel");
    push_block_xy(71U, 3, 3);
    run_one_service_pass();
    expect_u32(step_generator_task_instance.halted, 1U, "a refusal of the second member halts the print instance");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
        "the refusal is a feeder error"
    );
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "the refusal is one axis error");
    expect_u32(test_listener_calls[TEST_CH_X], 1U, "X was offered once");
    expect_u32(test_listener_calls[TEST_CH_Y], 2U, "Y was offered by its owner and by the print instance");
    expect_u32(test_listener_calls[TEST_CH_Z], 0U, "Z is not asked after the refusal");
    expect_u32(test_listener_calls[TEST_CH_SYRINGE], 0U, "the syringe is not asked after the refusal");
    expect_true(test_stream_owner[TEST_CH_X] == NULL, "the channel of X that the print instance took is let go");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == NULL, "the switch of X that the print instance took is let go");
    expect_true(test_limit_listeners[MOTION_AXIS_X] == NULL, "X has no switch listener after the rollback");
    expect_u32(test_limit_releases[MOTION_AXIS_X], 1U, "the switch of X is let go once");
    expect_true(test_stream_owner[TEST_CH_Y] == test_axis_context(MOTION_AXIS_Y), "the owner keeps the channel of Y");
    expect_true(test_limit_contexts[MOTION_AXIS_Y] == axis_y, "the owner keeps the switch of Y");
    expect_u32(step_generator_task_instance.channels_held, 0U, "the refused instance holds nothing after the rollback");
    expect_u32(axis_y->channels_held, 1U, "the owner of Y still holds its channels");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z],
        0U,
        "the rollback stops no channel, neither the one it let go nor the foreign one"
    );

    /* Only the switch listener of Y is held by another: the channel listener of Y that was taken is let go too. */
    reset_fixture();
    expect_u32(
        motion_axis_hardware_set_stream_limit_listener(MOTION_AXIS_Y, test_foreign_limit, &test_foreign_context),
        1U,
        "a foreign owner takes the switch of Y"
    );
    push_block_xy(72U, 3, 3);
    run_one_service_pass();
    expect_u32(step_generator_task_instance.halted, 1U, "a refused switch listener halts the print instance");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "the refusal is one axis error");
    expect_u32(test_listener_calls[TEST_CH_Y], 1U, "the channel of Y was offered first");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_Y], 2U, "the switch of Y was offered by the foreign owner and by us");
    expect_u32(test_listener_calls[TEST_CH_Z], 0U, "Z is not asked after the refusal");
    expect_true(test_stream_owner[TEST_CH_Y] == NULL, "the channel of Y that the print instance took is let go");
    expect_true(test_stream_owner[TEST_CH_X] == NULL, "the channel of X that the print instance took is let go");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == NULL, "the switch of X that the print instance took is let go");
    expect_true(test_limit_contexts[MOTION_AXIS_Y] == &test_foreign_context, "the foreign owner keeps the switch of Y");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z],
        0U,
        "the rollback of a refused switch stops no channel"
    );
    expect_true(test_limit_listeners[MOTION_AXIS_Y] == test_foreign_limit, "its handler is still installed");
    expect_u32(step_generator_task_instance.channels_held, 0U, "the refused instance holds nothing");

    /* The channel of the syringe, the last member, refuses: the three axes that were taken are let go. */
    reset_fixture();
    test_axis_streaming[TEST_CH_SYRINGE] = 1U;
    expect_u32(step_generator_task_claim_channels(&step_generator_task_instance), 0U, "a streaming syringe refuses the claim");
    expect_u32(test_listener_calls[TEST_CH_SYRINGE], 1U, "the syringe was offered the listener");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_true(test_stream_owner[channel] == NULL, "no channel stays with the refused instance");
    }
    for (uint32_t axis = 0U; axis < TEST_CHANNELS; ++axis) {
        expect_true(test_limit_contexts[axis] == NULL, "no switch stays with the refused instance");
        expect_u32(test_limit_releases[axis], 1U, "the switch of an axis that was taken is let go once");
    }
    expect_u32(step_generator_task_instance.channels_held, 0U, "the refused instance holds nothing after the rollback");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z] +
            test_stop_calls[TEST_CH_SYRINGE],
        0U,
        "the rollback stops no channel, least of all the one that streams"
    );
}

/* A release lets go of the listeners of its own instance only; the instance that holds nothing releases nothing. */
static void test_release_lets_go_only_of_its_own(void) {
    reset_fixture();
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    test_claim_print_channels();
    test_claim_print_channels();
    expect_u32(test_listener_calls[TEST_CH_X], 2U, "the owner takes its channel again");
    expect_u32(step_generator_task_instance.channels_held, 1U, "the print instance holds its channels");
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        expect_true(test_stream_owner[member] == test_print_context(member), "the print instance owns the channel");
    }
    for (uint32_t axis = 0U; axis < TEST_CHANNELS; ++axis) {
        expect_true(test_limit_contexts[axis] == &step_generator_task_instance, "and the switch of its axis");
    }

    expect_u32(step_generator_task_claim_channels(axis_x), 0U, "the instance of X cannot take what the print instance holds");
    expect_u32(test_listener_calls[TEST_CH_X], 3U, "the channel of X was offered once more");
    expect_u32(test_listener_releases[TEST_CH_X], 1U, "the refused claim let go of its own, which was nothing");
    expect_u32(axis_x->channels_held, 0U, "the refused instance holds nothing");
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        expect_true(test_stream_owner[member] == test_print_context(member), "the release did not take the channel away");
    }
    for (uint32_t axis = 0U; axis < TEST_CHANNELS; ++axis) {
        expect_true(test_limit_contexts[axis] == &step_generator_task_instance, "nor the switch");
        expect_true(test_limit_listeners[axis] == step_generator_task_limit_reached, "nor its handler");
    }

    step_generator_task_release_channels(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.channels_held, 0U, "the owner let go of everything");
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        expect_true(test_stream_owner[member] == NULL, "the channel is free after the release of its owner");
    }
    for (uint32_t axis = 0U; axis < TEST_CHANNELS; ++axis) {
        expect_true(test_limit_contexts[axis] == NULL, "the switch is free after the release of its owner");
        expect_true(test_limit_listeners[axis] == NULL, "its handler is removed");
    }
    const uint32_t releases_before = test_listener_releases[TEST_CH_X];
    step_generator_task_release_channels(&step_generator_task_instance);
    expect_u32(test_listener_releases[TEST_CH_X], releases_before, "an instance that holds nothing asks nothing");
    expect_u32(test_limit_releases[MOTION_AXIS_X], 2U, "the switch of X was let go by the refused claim and by the owner");
}

/* The channels still stream at the end of the stream: the instance keeps them and lets go on a later pass. */
static void test_release_is_retried_while_a_channel_streams(void) {
    reset_fixture();
    push_block(80U, 3);
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_axis_stuck[channel] = 1U;
    }
    test_wait_script_count = 3U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[0] = TEST_ACTION_RELEASE_HALF_0;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[1] = TEST_ACTION_RELEASE_HALF_1;
    test_wait_results[2] = STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED;
    test_wait_actions[2] = TEST_ACTION_FREE_STUCK_CHANNELS;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");

    expect_u32(step_generator_task_instance.diagnostics.streams_stopped, 1U, "the stream ended");
    expect_u32(step_generator_task_instance.channels_held, 0U, "the channels are let go once they are at rest");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_true(test_stream_owner[channel] == NULL, "the channel is free after the retry");
    }
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_true(test_limit_contexts[channel] == NULL, "the switch is free after the retry");
    }
    expect_u32(test_listener_releases[TEST_CH_X], 3U, "two refused attempts and the one that succeeded");
    expect_u32(test_listener_releases[TEST_CH_SYRINGE], 3U, "the syringe was released with the same attempts");

    /* Only the channel of the syringe still streams: the axes are let go, the instance keeps holding the syringe. */
    reset_fixture();
    test_claim_print_channels();
    test_axis_streaming[TEST_CH_SYRINGE] = 1U;
    step_generator_task_release_channels(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.channels_held, 1U, "a streaming syringe keeps the instance the holder");
    expect_true(
        test_stream_owner[TEST_CH_SYRINGE] == test_print_context(TEST_MEMBER_SYRINGE),
        "the instance stays the owner of the channel of the syringe"
    );
    expect_true(test_stream_owner[TEST_CH_X] == NULL, "the channel of X is let go");
    expect_true(test_stream_owner[TEST_CH_Y] == NULL, "the channel of Y is let go");
    expect_true(test_stream_owner[TEST_CH_Z] == NULL, "the channel of Z is let go");
    test_axis_streaming[TEST_CH_SYRINGE] = 0U;
    step_generator_task_release_channels(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.channels_held, 0U, "the next attempt lets go of the syringe");
    expect_true(test_stream_owner[TEST_CH_SYRINGE] == NULL, "the channel of the syringe is free after the next attempt");

    /* Only the switch listener refuses: the channel listener is let go but the instance still holds the axis. */
    reset_fixture();
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    expect_u32(
        motion_axis_hardware_set_stream_limit_listener(MOTION_AXIS_X, step_generator_task_limit_reached, axis_x),
        1U,
        "the instance of X owns the switch of X"
    );
    axis_x->channels_held = 1U;
    test_axis_streaming[TEST_CH_X] = 1U;
    step_generator_task_release_channels(axis_x);
    expect_u32(axis_x->channels_held, 1U, "a refusing switch listener keeps the instance the holder");
    expect_u32(test_limit_releases[MOTION_AXIS_X], 1U, "the switch listener was asked to let go");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == axis_x, "the owner keeps the switch while its channel streams");
    test_axis_streaming[TEST_CH_X] = 0U;
    step_generator_task_release_channels(axis_x);
    expect_u32(axis_x->channels_held, 0U, "the next attempt lets go");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == NULL, "the switch is free after the next attempt");
}

/* A halt of one instance stops its channels only; STOP of all halts the four; busy follows each queue. */
static void test_halt_and_busy_are_per_instance(void) {
    reset_fixture();
    StepGeneratorInstance_t *const print = &step_generator_task_instance;
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];
    StepGeneratorInstance_t *const axis_z = &step_generator_task_axis_instances[MOTION_AXIS_Z];
    for (uint32_t axis = 0U; axis < MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        init_axis_plan_buffer((MotionAxis_t)axis);
        step_generator_task_axis_instances[axis].plan_buffer = &test_axis_plan_buffer[axis];
    }

    push_axis_block(MOTION_AXIS_Y, 90U, 3);
    expect_u32(step_generator_instance_is_busy(axis_y), 1U, "a block in the queue of Y makes the instance of Y busy");
    expect_u32(step_generator_instance_is_busy(axis_x), 0U, "the instance of X is not busy");
    expect_u32(step_generator_instance_is_busy(axis_z), 0U, "the instance of Z is not busy");
    expect_u32(step_generator_instance_is_busy(print), 0U, "the print instance is not busy");
    axis_z->stream_busy = 1U;
    expect_u32(step_generator_instance_is_busy(axis_z), 1U, "a running stream makes the instance of Z busy");
    expect_u32(step_generator_instance_is_busy(axis_x), 0U, "the instance of X is still not busy");
    axis_z->stream_busy = 0U;
    expect_u32(plan_buffer_clear(&test_axis_plan_buffer[MOTION_AXIS_Y]), PLAN_BUFFER_STATUS_OK, "the queue of Y clears");
    expect_u32(step_generator_instance_is_busy(axis_y), 0U, "the instance of Y is idle after the clear");

    /* The three one-lane instances hold their channels; the print instance holds none. */
    expect_u32(step_generator_task_claim_channels(axis_x), 1U, "the instance of X takes its channel");
    expect_u32(step_generator_task_claim_channels(axis_y), 1U, "the instance of Y takes its channel");
    expect_u32(step_generator_task_claim_channels(axis_z), 1U, "the instance of Z takes its channel");
    step_generator_instance_halt(axis_x);
    expect_u32(step_generator_instance_is_halted(axis_x), 1U, "the instance of X is halted");
    expect_u32(step_generator_instance_is_halted(axis_y), 0U, "the instance of Y is not");
    expect_u32(step_generator_instance_is_halted(axis_z), 0U, "the instance of Z is not");
    expect_u32(step_generator_instance_is_halted(print), 0U, "the print instance is not");
    expect_u32(axis_x->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "the halt is a halt call");
    expect_u32(axis_x->halts_total, 1U, "the halt is counted by the instance of X");
    expect_u32(axis_y->halts_total + axis_z->halts_total + print->halts_total, 0U, "the others count no halt");
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the channel of X is stopped");
    expect_u32(test_stop_calls[TEST_CH_Y], 0U, "the channel of Y is not");
    expect_u32(test_stop_calls[TEST_CH_Z], 0U, "the channel of Z is not");

    step_generator_task_halt_all();
    expect_u32(step_generator_instance_is_halted(axis_x), 1U, "STOP halts the instance of X");
    expect_u32(step_generator_instance_is_halted(axis_y), 1U, "STOP halts the instance of Y");
    expect_u32(step_generator_instance_is_halted(axis_z), 1U, "STOP halts the instance of Z");
    expect_u32(step_generator_instance_is_halted(print), 1U, "STOP halts the print instance");
    expect_u32(axis_x->halts_total, 2U, "X counts both halts");
    expect_u32(axis_y->halts_total, 1U, "Y counts the STOP");
    expect_u32(axis_z->halts_total, 1U, "Z counts the STOP");
    expect_u32(print->halts_total, 1U, "the print instance counts the STOP");
    /* The print instance holds nothing, so it stops nothing: X is stopped by the earlier halt and by STOP. */
    expect_u32(test_stop_calls[TEST_CH_X], 2U, "X: the earlier halt and the STOP of the instance of X");
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "Y: the STOP of the instance of Y only");
    expect_u32(test_stop_calls[TEST_CH_Z], 1U, "Z: the STOP of the instance of Z only");

    /* The reset is asked per instance too. */
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_instance_resume_after_stop(axis_y);
    expect_u32(axis_y->reset_requested, 1U, "the instance of Y is marked");
    expect_u32(axis_x->reset_requested + axis_z->reset_requested + print->reset_requested, 0U, "the others are not");
    step_generator_task_resume_all_after_stop();
    expect_u32(
        axis_x->reset_requested + axis_y->reset_requested + axis_z->reset_requested + print->reset_requested,
        4U,
        "resume of all marks the four instances"
    );
}

/* An instance of an axis without a queue fails alone; the others go on, and the other way round. */
static void test_axis_instance_without_a_queue_fails_alone(void) {
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_Y);
    push_axis_block(MOTION_AXIS_Y, 95U, 3);
    run_one_service_pass();
    const MotionAxis_t without_queue[] = {MOTION_AXIS_X, MOTION_AXIS_Z};
    for (uint32_t index = 0U; index < sizeof(without_queue) / sizeof(without_queue[0]); ++index) {
        const StepGeneratorInstance_t *const instance = &step_generator_task_axis_instances[without_queue[index]];
        expect_u32(instance->ready, 0U, "an instance without a queue is not set up");
        expect_u32(instance->halted, 1U, "an instance without a queue is halted");
        expect_u32(instance->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED, "the cause is a failed setup");
        expect_u32(instance->halt_member, STEP_GENERATOR_TASK_NO_MEMBER, "no channel is to blame");
        expect_u32(instance->diagnostics.generator_errors, 1U, "the missing queue is counted once");
        expect_true(
            step_generator_instance_get_plan_buffer(instance) == NULL,
            "an instance without a queue keeps none"
        );
    }
    const StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];
    expect_u32(axis_y->ready, 1U, "the instance of Y is set up");
    expect_u32(axis_y->halted, 0U, "the instance of Y feeds");
    expect_u32(axis_y->diagnostics.generator_errors, 0U, "the instance of Y has no error");
    expect_u32(axis_y->diagnostics.streams_started, 1U, "the instance of Y started its stream");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the channel of Y starts");
    expect_u32(test_start_calls[TEST_CH_X] + test_start_calls[TEST_CH_Z], 0U, "X and Z start nothing");
    expect_u32(step_generator_task_instance.ready, 1U, "the print instance is set up");
    expect_u32(step_generator_task_instance.halted, 0U, "the print instance feeds");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "the print instance has no error");

    /* Without the print queue the print instance fails alone. */
    reset_fixture();
    test_task_config.plan_buffer = NULL;
    init_axis_plan_buffer(MOTION_AXIS_Y);
    push_axis_block(MOTION_AXIS_Y, 96U, 3);
    run_one_service_pass();
    expect_u32(step_generator_task_instance.ready, 0U, "the print instance without a queue is not set up");
    expect_u32(step_generator_task_instance.halted, 1U, "the print instance without a queue is halted");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED,
        "the print cause is a failed setup"
    );
    expect_u32(step_generator_task_axis_instances[MOTION_AXIS_Y].halted, 0U, "the instance of Y is not affected");
    expect_u32(test_start_calls[TEST_CH_Y], 1U, "the channel of Y starts");

    /* Without a config none of the four is set up. */
    reset_fixture();
    test_wait_script_count = 0U;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task ends at the first wait");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "the print instance counts the missing config");
    for (uint32_t axis = 0U; axis < MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        const StepGeneratorInstance_t *const instance = &step_generator_task_axis_instances[axis];
        expect_u32(instance->ready, 0U, "no instance is set up without a config");
        expect_u32(instance->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED, "the cause is a failed setup");
        expect_u32(instance->diagnostics.generator_errors, 1U, "the missing config is counted once per instance");
    }
}

static void test_instance_lookup_and_queues(void) {
    reset_fixture();
    expect_true(
        step_generator_task_get_axis_instance(MOTION_AXIS_X) == &step_generator_task_axis_instances[MOTION_AXIS_X],
        "the instance of X"
    );
    expect_true(
        step_generator_task_get_axis_instance(MOTION_AXIS_Y) == &step_generator_task_axis_instances[MOTION_AXIS_Y],
        "the instance of Y"
    );
    expect_true(
        step_generator_task_get_axis_instance(MOTION_AXIS_Z) == &step_generator_task_axis_instances[MOTION_AXIS_Z],
        "the instance of Z"
    );
    expect_true(
        step_generator_task_get_axis_instance(MOTION_AXIS_HARDWARE_COUNT) == NULL,
        "an axis past Z has no instance"
    );
    expect_true(step_generator_task_get_axis_instance((MotionAxis_t)99) == NULL, "an unknown axis has no instance");
    expect_true(step_generator_instance_get_plan_buffer(NULL) == NULL, "a NULL instance has no queue");
    expect_true(
        step_generator_instance_get_plan_buffer(step_generator_task_get_axis_instance(MOTION_AXIS_Y)) == NULL,
        "an instance has no queue before the task starts"
    );

    init_axis_plan_buffer(MOTION_AXIS_X);
    init_axis_plan_buffer(MOTION_AXIS_Y);
    init_axis_plan_buffer(MOTION_AXIS_Z);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_true(
        step_generator_instance_get_plan_buffer(step_generator_task_get_axis_instance(MOTION_AXIS_X)) ==
            &test_axis_plan_buffer[MOTION_AXIS_X],
        "the instance of X plays the queue of X"
    );
    expect_true(
        step_generator_instance_get_plan_buffer(step_generator_task_get_axis_instance(MOTION_AXIS_Y)) ==
            &test_axis_plan_buffer[MOTION_AXIS_Y],
        "the instance of Y plays the queue of Y"
    );
    expect_true(
        step_generator_instance_get_plan_buffer(step_generator_task_get_axis_instance(MOTION_AXIS_Z)) ==
            &test_axis_plan_buffer[MOTION_AXIS_Z],
        "the instance of Z plays the queue of Z"
    );
    expect_true(
        step_generator_instance_get_plan_buffer(step_generator_task_get_print_instance()) == &test_plan_buffer,
        "the print instance plays the queue of the config"
    );
}

/* An instance stops only the channels it holds: a halt of another instance must not cut a stream that is not its own. */
static void test_halt_stops_only_the_channels_the_instance_holds(void) {
    reset_fixture();
    StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    expect_u32(step_generator_task_claim_channels(axis_x), 1U, "the instance of X takes its channel");
    step_generator_instance_halt(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.halted, 1U, "the print instance is halted");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z] +
            test_stop_calls[TEST_CH_SYRINGE],
        0U,
        "the halt of the print instance stops no channel, X belongs to the instance of X"
    );
    expect_u32(axis_x->halted, 0U, "the instance of X is not halted by it");
    step_generator_instance_halt(axis_x);
    expect_u32(test_stop_calls[TEST_CH_X], 1U, "the halt of the holder stops its channel of X");
    expect_u32(
        test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z] + test_stop_calls[TEST_CH_SYRINGE],
        0U,
        "and no other channel"
    );

    /* The instance of Y holds only Y: its halt leaves X and Z alone. */
    reset_fixture();
    expect_u32(
        step_generator_task_claim_channels(&step_generator_task_axis_instances[MOTION_AXIS_Y]),
        1U,
        "the instance of Y takes its channel"
    );
    step_generator_instance_halt(&step_generator_task_axis_instances[MOTION_AXIS_Y]);
    expect_u32(test_stop_calls[TEST_CH_Y], 1U, "the instance of Y stops the channel of Y once");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Z] + test_stop_calls[TEST_CH_SYRINGE],
        0U,
        "and not the channels of X, Z and the syringe"
    );

    /* A refused claim while another instance streams on X: the stream goes on, no channel is stopped. */
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    push_axis_block(MOTION_AXIS_X, 110U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    test_wait_actions[0] = TEST_ACTION_PUSH_PRINT_BLOCK_XY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    const StepGeneratorInstance_t *const streaming_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    expect_u32(test_start_calls[TEST_CH_X], 1U, "the instance of X started its stream before the print block came");
    expect_u32(step_generator_task_instance.halted, 1U, "the print instance is refused and halted");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
        "the refusal is a feeder error"
    );
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 1U, "the refusal is one axis error");
    expect_u32(
        test_stop_calls[TEST_CH_X] + test_stop_calls[TEST_CH_Y] + test_stop_calls[TEST_CH_Z] +
            test_stop_calls[TEST_CH_SYRINGE],
        0U,
        "the refusal stops no channel, least of all the one that streams"
    );
    expect_u32(test_axis_streaming[TEST_CH_X], 1U, "the stream of X goes on");
    expect_u32(streaming_x->halted, 0U, "the instance of X is not halted");
    expect_u32(step_stream_group_get_state(&streaming_x->group), STEP_STREAM_GROUP_STATE_RUNNING, "its group runs on");
    expect_true(test_stream_owner[TEST_CH_X] == test_axis_context(MOTION_AXIS_X), "the instance of X keeps its channel");
    expect_u32(streaming_x->channels_held, 1U, "and its hold");
}

/* After a STOP reset the channels are free again, so another instance can take them; a reset that fails keeps them. */
static void test_stop_reset_frees_the_channels(void) {
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    push_axis_block(MOTION_AXIS_X, 100U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_wait_actions[0] = TEST_ACTION_HALT_X_AXIS_INSTANCE;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    const StepGeneratorInstance_t *const axis_x = &step_generator_task_axis_instances[MOTION_AXIS_X];
    expect_u32(axis_x->diagnostics.stop_resets, 1U, "the instance of X was reset");
    expect_u32(axis_x->halted, 0U, "the reset lifts the halt");
    expect_u32(axis_x->channels_held, 0U, "the reset lets go of the channels");
    expect_true(test_stream_owner[TEST_CH_X] == NULL, "the channel of X is free after the reset");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == NULL, "the switch of X is free after the reset");
    expect_true(test_limit_listeners[MOTION_AXIS_X] == NULL, "its handler is removed");
    expect_u32(step_generator_task_claim_channels(&step_generator_task_instance), 1U, "another instance takes the channels");
    expect_true(test_stream_owner[TEST_CH_X] == test_print_context(0U), "the print instance owns the channel of X now");
    expect_true(test_limit_contexts[MOTION_AXIS_X] == &step_generator_task_instance, "and the switch of X");
    expect_true(
        test_stream_owner[TEST_CH_SYRINGE] == test_print_context(TEST_MEMBER_SYRINGE),
        "and the channel of the syringe"
    );

    /* A channel that still runs after STOP: the reset fails and the instance goes on holding its channels. */
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    push_axis_block(MOTION_AXIS_X, 101U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_wait_actions[0] = TEST_ACTION_HALT_X_AXIS_INSTANCE_CHANNEL_RUNS;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(axis_x->halted, 1U, "a reset that fails leaves the instance halted");
    expect_u32(axis_x->halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED, "the cause is the failed reset");
    expect_u32(axis_x->diagnostics.axis_errors, 1U, "the failed reset is an axis error");
    expect_u32(axis_x->channels_held, 1U, "the instance holds its channels while the channel runs");
    expect_u32(test_listener_releases[TEST_CH_X], 0U, "no release is tried after a failed reset");
    expect_true(test_stream_owner[TEST_CH_X] == test_axis_context(MOTION_AXIS_X), "the owner keeps the channel of X");
}

/* MotionTask asks whether an instance can be reset at all: only one whose feeder was built can. */
static void test_instance_is_ready_once_its_feeder_is_built(void) {
    reset_fixture();
    expect_u32(step_generator_instance_is_ready(NULL), 0U, "a NULL instance is not ready");
    expect_u32(
        step_generator_instance_is_ready(&step_generator_task_instance),
        0U,
        "an instance the task has not set up yet is not ready"
    );

    /* The setup of the print instance succeeds; an axis instance without a queue does not. */
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    test_wait_script_count = 1U;
    test_wait_actions[0] = TEST_ACTION_SAMPLE_PRINT;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(test_sampled_ready[0], 1U, "the print instance is ready by the first wait of the task");
    expect_u32(step_generator_instance_is_ready(step_generator_task_get_print_instance()), 1U, "the print instance stays ready");
    expect_u32(
        step_generator_instance_is_ready(step_generator_task_get_axis_instance(MOTION_AXIS_X)),
        1U,
        "the instance of X with a queue is ready"
    );
    expect_u32(
        step_generator_instance_is_ready(step_generator_task_get_axis_instance(MOTION_AXIS_Y)),
        0U,
        "the instance of Y without a queue is not ready"
    );
    expect_u32(
        step_generator_instance_is_ready(step_generator_task_get_axis_instance(MOTION_AXIS_Z)),
        0U,
        "the instance of Z without a queue is not ready"
    );

    /* Every way the setup fails leaves the print instance not ready, and halted. */
    reset_fixture();
    (void)run_task_until_jump(NULL);
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 0U, "no config, not ready");
    expect_u32(step_generator_instance_is_halted(&step_generator_task_instance), 1U, "no config, halted");

    reset_fixture();
    test_task_config.plan_buffer = NULL;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 0U, "no queue, not ready");

    reset_fixture();
    test_task_config.step_pulse_width_ns = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 0U, "unusable timing, not ready");

    reset_fixture();
    test_timer_clock_hz[TEST_CH_Y] = TEST_TIMER_CLOCK_HZ / 2U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 0U, "clocks that differ, not ready");

    reset_fixture();
    test_timer_clock_hz[TEST_CH_SYRINGE] = TEST_TIMER_CLOCK_HZ / 2U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_instance_is_ready(&step_generator_task_instance),
        0U,
        "the syringe on another clock than the first member: not ready"
    );
}

/* MotionTask waits for this count after STOP: it rises once per finished reset, and only for an instance that is reset. */
static void test_reset_count_rises_by_one_per_finished_reset(void) {
    reset_fixture();
    expect_u32(step_generator_instance_reset_count(NULL), 0U, "a NULL instance has no resets");
    expect_u32(step_generator_instance_reset_count(&step_generator_task_instance), 0U, "a fresh instance has no resets");

    /* Two resets with a pass in between that has no request. The samples are taken before the reset of their wait. */
    reset_fixture();
    step_generator_task_halt_all();
    test_wait_script_count = 3U;
    test_wait_actions[0] = TEST_ACTION_SAMPLE_PRINT;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    test_wait_actions[1] = TEST_ACTION_SAMPLE_PRINT;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    test_wait_actions[2] = TEST_ACTION_SAMPLE_PRINT;
    test_wait_results[2] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(test_sampled_reset_count[0], 0U, "no reset has finished before the first request");
    expect_u32(test_sampled_halted[0], 1U, "the instance is halted until the reset");
    expect_u32(test_sampled_reset_count[1], 1U, "the first reset raised the count to 1");
    expect_u32(test_sampled_halted[1], 0U, "the first reset lifted the halt");
    expect_u32(test_sampled_reset_count[2], 1U, "a pass without a request does not raise the count");
    expect_u32(
        step_generator_instance_reset_count(&step_generator_task_instance),
        2U,
        "the second reset raised the count to 2"
    );
    expect_u32(step_generator_instance_is_halted(&step_generator_task_instance), 0U, "the second reset lifted the halt");
    StepGeneratorTaskDiagnostics diagnostics = {0};
    step_generator_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.stop_resets, 2U, "the count is the stop_resets of the diagnostics");

    /* Without a request the halt stays and nothing is counted. */
    reset_fixture();
    step_generator_task_halt_all();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_instance_reset_count(&step_generator_task_instance), 0U, "no request, no finished reset");
    expect_u32(step_generator_instance_is_halted(&step_generator_task_instance), 1U, "no request, still halted");

    /* An instance that was not set up is never reset, whatever is asked. */
    reset_fixture();
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(NULL), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_instance_reset_count(&step_generator_task_instance), 0U, "no feeder, no reset");
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 0U, "and it is not ready");
    expect_u32(step_generator_instance_is_halted(&step_generator_task_instance), 1U, "and it stays halted");

    /* Each instance counts its own resets: only the instance of X is asked here. */
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_X);
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_instance_resume_after_stop(step_generator_task_get_axis_instance(MOTION_AXIS_X));
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(
        step_generator_instance_reset_count(step_generator_task_get_axis_instance(MOTION_AXIS_X)),
        1U,
        "the instance of X was reset"
    );
    expect_u32(step_generator_instance_reset_count(&step_generator_task_instance), 0U, "the print instance was not asked");
    expect_u32(
        step_generator_instance_reset_count(step_generator_task_get_axis_instance(MOTION_AXIS_Y)),
        0U,
        "the instance of Y was not asked"
    );
}

/* A channel that still runs makes the reset fail: the count rises all the same, after the instance is halted again. */
static void test_reset_count_rises_when_the_reset_fails(void) {
    reset_fixture();
    step_generator_task_halt_all();
    test_wait_script_count = 2U;
    test_wait_actions[0] = TEST_ACTION_Y_STREAMING;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    test_wait_actions[1] = TEST_ACTION_SAMPLE_PRINT;
    test_wait_results[1] = STEP_GENERATOR_TASK_FLAG_STOP_RESET;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(test_sampled_reset_count[1], 0U, "no reset has finished before the request");
    expect_u32(test_sampled_halted[1], 1U, "the instance is halted before the reset");
    expect_u32(step_generator_instance_reset_count(&step_generator_task_instance), 1U, "the failed reset is counted");
    expect_u32(
        step_generator_instance_is_halted(&step_generator_task_instance),
        1U,
        "the instance is halted again once the count has risen"
    );
    /* The cause is recorded only when the halt was lifted first: the reset lifted it, then failed and halted again. */
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED,
        "the halt after the count rose is the failed reset"
    );
    expect_u32(step_generator_task_instance.halts_total, 2U, "STOP halted once and the failed reset halted again");
}

/* ---- Progress snapshot ---- */

static StepGeneratorProgress_t read_progress(const StepGeneratorInstance_t *instance) {
    StepGeneratorProgress_t progress;
    (void)memset(&progress, 0xFF, sizeof(progress));
    step_generator_instance_get_progress(instance, &progress);
    return progress;
}

static void run_print_pass(void) { step_generator_task_run_instance(&step_generator_task_instance); }

/* The syringe idles like Z in the blocks of these tests, so it has played as many records as Z. */
static void set_records_played_all(uint32_t x, uint32_t y, uint32_t z, uint32_t syringe) {
    test_records_played[TEST_CH_X] = x;
    test_records_played[TEST_CH_Y] = y;
    test_records_played[TEST_CH_Z] = z;
    test_records_played[TEST_CH_SYRINGE] = syringe;
}

static void set_records_played(uint32_t x, uint32_t y, uint32_t z) { set_records_played_all(x, y, z, z); }

/* A block of x_steps on X whose MotionPlanBlock_t.order is `order`. */
static void push_ordered_block(uint32_t order, int64_t x_steps) {
    MotionPlanBlock_t block = make_block(500U + order, x_steps, 0);
    block.order = order;
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "ordered block is queued");
}

/* The task has run one pass: the print stream plays three blocks of 3 steps on X (orders 21, 22, 23) and its channels
 * stream. On Y, Z and the syringe each block is one idle record, so the block ends there are records 1, 2 and 3. */
static void start_print_stream_with_three_blocks(void) {
    reset_fixture();
    push_ordered_block(21U, 3);
    push_ordered_block(22U, 3);
    push_ordered_block(23U, 3);
    test_wait_script_count = 0U;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_task_instance.channels_held, 1U, "the print stream holds its channels");
    expect_u32(step_generator_task_instance.halted, 0U, "the print stream is not halted");
}

/* The task has run one pass on an empty queue and is ready; nothing streams. */
static void start_idle_print_task(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");
    expect_u32(step_generator_instance_is_ready(&step_generator_task_instance), 1U, "the print instance is ready");
}

static void test_progress_of_a_fresh_instance_and_null_arguments(void) {
    reset_fixture();
    StepGeneratorProgress_t marker;
    (void)memset(&marker, 0xAB, sizeof(marker));
    StepGeneratorProgress_t progress = marker;
    step_generator_instance_get_progress(NULL, &progress);
    expect_true(memcmp(&progress, &marker, sizeof(progress)) == 0, "a NULL instance writes nothing");

    /* A NULL progress is ignored; the instance keeps what it had. */
    step_generator_instance_get_progress(&step_generator_task_instance, NULL);
    expect_u32(step_generator_task_instance.generation, 0U, "a NULL progress changes nothing");

    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.generation, 0U, "fresh generation is 0");
    expect_u32(progress.played_blocks, 0U, "fresh played_blocks is 0");
    expect_u32(progress.played_order, 0U, "fresh played_order is 0");
    expect_u32(progress.stop_resolved, 0U, "fresh stop is not resolved");
    expect_u32(progress.has_unplayed, 0U, "fresh has_unplayed is 0");
    expect_u32(progress.unplayed_order, 0U, "fresh unplayed_order is 0");
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_NONE, "fresh halt_cause is 0");
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "fresh halt_axis names no axis");
    expect_u32(progress.halt_detail, 0U, "fresh halt_detail is 0");
}

static void test_generation_rises_with_every_halt(void) {
    reset_fixture();
    step_generator_instance_halt(&step_generator_task_instance);
    expect_u32(read_progress(&step_generator_task_instance).generation, 1U, "the first halt makes generation 1");
    step_generator_instance_halt(&step_generator_task_instance);
    expect_u32(read_progress(&step_generator_task_instance).generation, 2U, "a repeated halt makes generation 2");
    step_generator_instance_halt(NULL);
    expect_u32(read_progress(&step_generator_task_instance).generation, 2U, "a NULL halt changes no generation");
    for (uint32_t axis = 0U; axis < STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT; ++axis) {
        expect_u32(
            read_progress(step_generator_task_get_axis_instance((MotionAxis_t)axis)).generation,
            0U,
            "a halt of the print instance leaves the generation of an axis instance"
        );
    }

    /* STOP of all: every instance counts it once. */
    step_generator_task_halt_all();
    expect_u32(read_progress(&step_generator_task_instance).generation, 3U, "STOP of all counts in the print instance");
    for (uint32_t axis = 0U; axis < STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT; ++axis) {
        expect_u32(
            read_progress(step_generator_task_get_axis_instance((MotionAxis_t)axis)).generation,
            1U,
            "STOP of all counts once in each axis instance"
        );
    }
    step_generator_task_halt_all();
    expect_u32(read_progress(&step_generator_task_instance).generation, 4U, "a second STOP of all counts again");
    expect_u32(
        read_progress(step_generator_task_get_axis_instance(MOTION_AXIS_Z)).generation,
        2U,
        "a second STOP of all counts again in an axis instance"
    );
}

static void test_halt_wakes_the_task_when_it_runs(void) {
    reset_fixture();
    step_generator_instance_halt(&step_generator_task_instance);
    expect_u32(test_flags_set_calls, 0U, "a halt before the task starts wakes nobody");
    expect_u32(step_generator_task_instance.halted, 1U, "the halt itself still happens");

    reset_fixture();
    step_generator_task_thread_id = TEST_THREAD_ID;
    step_generator_instance_halt(&step_generator_task_instance);
    expect_u32(test_flags_set_calls, 1U, "a halt wakes the task once");
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED, "the halt sets the wake flag");
    expect_true(test_flags_set_target == TEST_THREAD_ID, "the wake goes to the generator task");

    /* A fault reported by a channel interrupt wakes it as well. */
    start_idle_print_task();
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_flags_set_value = 0U;
    test_stream_listener[TEST_CH_Y].on_fault(test_stream_listener[TEST_CH_Y].context, STEP_STREAM_AXIS_FAULT_DMA);
    expect_u32(test_flags_set_value, STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED, "a channel fault wakes the task");
}

static void test_running_stream_advances_played_blocks_by_block_ends(void) {
    start_print_stream_with_three_blocks();
    expect_u32(test_records_played_calls[TEST_CH_X], 1U, "the pass asked X for its played records once");
    expect_u32(test_records_played_calls[TEST_CH_Y], 1U, "the pass asked Y for its played records once");
    expect_u32(test_records_played_calls[TEST_CH_Z], 1U, "the pass asked Z for its played records once");
    expect_u32(test_records_played_calls[TEST_CH_SYRINGE], 1U, "the pass asked the syringe for its played records once");
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 0U, "nothing is played at the start");
    expect_u32(progress.played_order, 0U, "no block order is reported at the start");

    /* X is done, Y and Z are not (their first block ends at record 1) ... */
    set_records_played(1000U, 1000U, 0U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 0U, "a block is not played while Z has not reached its end");
    /* ... and the other way round. */
    set_records_played(0U, 1000U, 1000U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 0U, "a block is not played while X has not reached its end");

    set_records_played(1000U, 1U, 1U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 1U, "the first block is played once every lane has reached its end");
    expect_u32(progress.played_order, 21U, "the order of the first block is reported");

    set_records_played(1000U, 2U, 2U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 2U, "the second block is played");
    expect_u32(progress.played_order, 22U, "the order of the second block is reported");

    set_records_played(1000U, 3U, 3U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 3U, "the third block is played");
    expect_u32(progress.played_order, 23U, "the order of the third block is reported");
    expect_u32(progress.stop_resolved, 0U, "a stream that is not halted resolves no stop");
    expect_u32(progress.has_unplayed, 0U, "a stream that is not halted names no unplayed block");
    expect_u32(progress.generation, 0U, "playing blocks does not change the generation");
}

/* The syringe is a lane of the stream: a block is played only once the syringe, too, has reached its end. */
static void test_a_block_is_played_only_when_the_syringe_reaches_its_end(void) {
    start_print_stream_with_three_blocks();
    set_records_played_all(1000U, 1000U, 1000U, 0U);
    run_print_pass();
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 0U, "no block is played while the syringe has not reached its end");
    expect_u32(progress.played_order, 0U, "no order is reported while the syringe lags");

    set_records_played_all(1000U, 1000U, 1000U, 1U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 1U, "the first block is played once the syringe has reached record 1");
    expect_u32(progress.played_order, 21U, "the order of the first block is reported");

    set_records_played_all(1000U, 1000U, 1000U, 2U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.played_blocks, 2U, "the second block is played with the syringe at record 2");
    expect_u32(progress.played_order, 22U, "the order of the second block is reported");
}

/* Z trips its end switch and Y is the first channel to brake: the stop is resolved only when every channel rests. */
static void test_limit_stop_is_resolved_when_every_channel_rests(void) {
    start_print_stream_with_three_blocks();
    set_records_played(1000U, 1U, 1U);
    run_print_pass();
    expect_u32(read_progress(&step_generator_task_instance).played_blocks, 1U, "block 21 is played before the stop");

    test_limit_listeners[MOTION_AXIS_Z](
        test_limit_contexts[MOTION_AXIS_Z],
        MOTION_AXIS_Z,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT
    );
    test_stream_listener[TEST_CH_Y].on_brake(test_stream_listener[TEST_CH_Y].context);
    expect_u32(step_generator_task_instance.halted, 1U, "the brake halts the print instance");

    /* Only Y still runs: the records it has played are not final, so nothing is counted and nothing is resolved. */
    test_axis_streaming[TEST_CH_X] = 0U;
    test_axis_streaming[TEST_CH_Y] = 1U;
    test_axis_streaming[TEST_CH_Z] = 0U;
    test_axis_streaming[TEST_CH_SYRINGE] = 0U;
    set_records_played(1000U, 1000U, 1000U);
    run_print_pass();
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.stop_resolved, 0U, "the stop is not resolved while Y streams");
    expect_u32(progress.played_blocks, 1U, "no block is counted while Y streams");
    expect_u32(progress.played_order, 21U, "the last played order is kept while Y streams");
    expect_u32(progress.has_unplayed, 0U, "no unplayed block is named while Y streams");
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "the cause is known at once");
    expect_u32(progress.halt_axis, (uint32_t)MOTION_AXIS_Z, "the stop is blamed on Z, whose switch fired");
    expect_u32(progress.halt_detail, 3U, "the detail is END_LIMIT");

    /* Every channel rests; Y and Z have played 2 records: block 22 is played, block 23 is cut off. */
    test_axis_streaming[TEST_CH_Y] = 0U;
    set_records_played(1000U, 2U, 2U);
    run_print_pass();
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.stop_resolved, 1U, "the stop is resolved once every channel rests");
    expect_u32(progress.played_blocks, 2U, "blocks 21 and 22 are played completely");
    expect_u32(progress.played_order, 22U, "block 22 is the last played one");
    expect_u32(progress.has_unplayed, 1U, "a block was cut off");
    expect_u32(progress.unplayed_order, 23U, "the first cut off block is 23");
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, "the cause stays LIMIT");
    expect_u32(progress.halt_axis, (uint32_t)MOTION_AXIS_Z, "the axis stays Z");
    expect_u32(progress.halt_detail, 3U, "the detail stays END_LIMIT");
    expect_u32(progress.generation, 0U, "a limit stop is not a STOP call");

    /* The stop is resolved once: later passes with other numbers change nothing. */
    set_records_played(1000U, 3U, 3U);
    run_print_pass();
    run_print_pass();
    const StepGeneratorProgress_t again = read_progress(&step_generator_task_instance);
    expect_u32(again.stop_resolved, 1U, "the stop stays resolved");
    expect_u32(again.played_blocks, 2U, "the played blocks are final after the resolve");
    expect_u32(again.played_order, 22U, "the played order is final after the resolve");
    expect_u32(again.has_unplayed, 1U, "the unplayed block stays named");
    expect_u32(again.unplayed_order, 23U, "the unplayed order is not resolved again");
}

static void test_stop_with_blocks_only_in_the_plan_names_the_head_of_the_plan(void) {
    start_idle_print_task();
    push_ordered_block(31U, 3);
    push_ordered_block(32U, 3);
    step_generator_instance_halt(&step_generator_task_instance);
    run_print_pass();
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.stop_resolved, 1U, "the stop of a resting instance is resolved at once");
    expect_u32(progress.has_unplayed, 1U, "the queued blocks are cut off");
    expect_u32(progress.unplayed_order, 31U, "the first queued block is named, not the last");
    expect_u32(progress.played_blocks, 0U, "nothing was played");
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "a STOP call is the cause");
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "a STOP call blames no axis");
    expect_u32(progress.halt_detail, 0U, "a STOP call has no detail");
    expect_u32(progress.generation, 1U, "the STOP call raised the generation");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_records_played_calls[channel], 0U, "an instance that holds no channel reads no records");
    }

    /* Lifting the halt clears the stop part of the snapshot and keeps the cause and the generation. */
    step_generator_instance_resume_after_stop(&step_generator_task_instance);
    run_print_pass();
    expect_u32(step_generator_task_instance.halted, 0U, "the reset lifted the halt");
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.stop_resolved, 0U, "the reset clears stop_resolved");
    expect_u32(progress.has_unplayed, 0U, "the reset clears has_unplayed");
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "the cause survives the reset");
    expect_u32(progress.generation, 1U, "the reset does not change the generation");
}

static void test_stop_without_any_block_has_nothing_unplayed(void) {
    start_idle_print_task();
    /* A leftover of an earlier stop must be overwritten, not kept. */
    step_generator_task_instance.stop_unplayed_order = 99U;
    step_generator_instance_halt(&step_generator_task_instance);
    run_print_pass();
    const StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.stop_resolved, 1U, "the stop is resolved");
    expect_u32(progress.has_unplayed, 0U, "no block anywhere, so none is cut off");
    expect_u32(progress.unplayed_order, 0U, "no unplayed order is reported");
}

static void test_halt_detail_follows_the_cause(void) {
    /* A fault of the channel of Y. */
    start_idle_print_task();
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Y].on_fault(test_stream_listener[TEST_CH_Y].context, STEP_STREAM_AXIS_FAULT_DMA);
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "a ring fault is the cause");
    expect_u32(progress.halt_axis, (uint32_t)MOTION_AXIS_Y, "the faulted channel is Y");
    expect_u32(progress.halt_detail, (uint32_t)STEP_STREAM_AXIS_FAULT_DMA, "the detail is the fault code");

    /* A feeder error. */
    reset_fixture();
    push_block(70U, 3);
    test_fail_prepare = 1U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR, "a feeder error is the cause");
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "a feeder error blames no channel");
    expect_u32(progress.halt_detail, (uint32_t)STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "the detail is the feeder status");

    /* The causes without a detail read 0 even when the fields of an earlier cause hold values. */
    start_idle_print_task();
    step_generator_instance_halt(&step_generator_task_instance);
    step_generator_task_instance.halt_fault = 7U;
    step_generator_task_instance.feeder_status = 9U;
    step_generator_task_instance.limit_reason = 3U;
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, "a STOP call is the cause");
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "a STOP call blames no axis");
    expect_u32(progress.halt_detail, 0U, "a STOP call has no detail");

    reset_fixture();
    (void)run_task_until_jump(NULL);
    step_generator_task_instance.halt_fault = 7U;
    step_generator_task_instance.feeder_status = 9U;
    step_generator_task_instance.limit_reason = 3U;
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED, "a failed setup is the cause");
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "a failed setup blames no axis");
    expect_u32(progress.halt_detail, 0U, "a failed setup has no detail");
}

static void test_halt_detail_carries_the_generator_status(void) {
    /* The generator rejects the block (denominator * 1e9 does not fit 64 bits): the feeder reports GENERATOR_ERROR (2)
     * and the generator status is ARITHMETIC_OVERFLOW (3), so the detail is (3 << 4) | 2. */
    reset_fixture();
    MotionPlanBlock_t block = make_block(80U, 3, 0);
    block.step_event_rate_denominator = UINT64_MAX / STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND + 1U;
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "the bad block is queued");
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR, "a feeder error is the cause");
    expect_u32(step_generator_task_instance.feeder_status, 2U, "the feeder status is GENERATOR_ERROR");
    expect_u32(step_generator_task_instance.feeder_generator_status, 3U, "the generator status is ARITHMETIC_OVERFLOW");
    expect_u32(progress.halt_detail, 0x32U, "generator status in the high nibble, feeder status in the low one");

    /* A feeder error that is not the generator's keeps the high nibble at 0. */
    reset_fixture();
    push_block(81U, 3);
    test_fail_prepare = 1U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_detail, (uint32_t)STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "no generator status, no high nibble");

    /* The other causes ignore the generator status. */
    start_idle_print_task();
    test_claim_print_channels();
    run_group_prepared_and_started();
    test_stream_listener[TEST_CH_Y].on_fault(test_stream_listener[TEST_CH_Y].context, STEP_STREAM_AXIS_FAULT_DMA);
    step_generator_task_instance.feeder_generator_status = 3U;
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "a ring fault is the cause");
    expect_u32(progress.halt_detail, (uint32_t)STEP_STREAM_AXIS_FAULT_DMA, "a ring fault detail is unchanged");

    step_generator_task_instance.halt_cause = STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT;
    step_generator_task_instance.limit_reason = (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    progress = read_progress(&step_generator_task_instance);
    expect_u32(progress.halt_detail, 3U, "a limit detail is unchanged (END_LIMIT)");
}

/* What the listener saw of the print snapshot when it was called: the snapshot is published before the call. */
static uint32_t test_progress_listener_calls;
static uint32_t test_progress_listener_played_blocks;
static uint32_t test_progress_listener_stop_resolved;

static void count_progress_listener(void) {
    ++test_progress_listener_calls;
    const StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    test_progress_listener_played_blocks = progress.played_blocks;
    test_progress_listener_stop_resolved = progress.stop_resolved;
}

/* MotionTask is woken when the print snapshot changes: blocks played, or the stop resolved. */
static void test_progress_listener_is_called_when_the_print_snapshot_changes(void) {
    test_progress_listener_calls = 0U;
    step_generator_task_set_progress_listener(count_progress_listener);
    start_print_stream_with_three_blocks();
    expect_u32(test_progress_listener_calls, 0U, "a stream that has played nothing reports no progress");

    set_records_played(1000U, 1U, 1U);
    run_print_pass();
    expect_u32(test_progress_listener_calls, 1U, "the first played block calls the listener");
    expect_u32(test_progress_listener_played_blocks, 1U, "the snapshot is published before the listener is called");
    run_print_pass();
    expect_u32(test_progress_listener_calls, 1U, "a pass that changes nothing does not call the listener");

    set_records_played(1000U, 2U, 2U);
    run_print_pass();
    expect_u32(test_progress_listener_calls, 2U, "the second played block calls the listener");
    expect_u32(test_progress_listener_played_blocks, 2U, "the listener sees two played blocks");

    step_generator_instance_halt(&step_generator_task_instance);
    run_print_pass();
    expect_u32(test_progress_listener_calls, 3U, "the resolved stop calls the listener");
    expect_u32(test_progress_listener_stop_resolved, 1U, "the stop is resolved when the listener is called");
    expect_u32(test_progress_listener_played_blocks, 2U, "the stop does not change the played blocks");
    run_print_pass();
    expect_u32(test_progress_listener_calls, 3U, "a stop is reported once");

    /* A new order with the same count of played blocks is a change too (a stream that restarted between two passes). */
    start_print_stream_with_three_blocks();
    set_records_played(1000U, 1U, 1U);
    run_print_pass();
    expect_u32(test_progress_listener_calls, 4U, "the first block of the new stream calls the listener");
    step_generator_task_instance.played_order = 99U;
    run_print_pass();
    expect_u32(test_progress_listener_calls, 5U, "another order with the same played blocks calls the listener");
    expect_u32(read_progress(&step_generator_task_instance).played_order, 21U, "the snapshot carries the order that was played");

    /* Without a listener the snapshot is still published. */
    step_generator_task_set_progress_listener(NULL);
    start_print_stream_with_three_blocks();
    set_records_played(1000U, 1U, 1U);
    run_print_pass();
    expect_u32(read_progress(&step_generator_task_instance).played_blocks, 1U, "the snapshot is published without a listener");
    expect_u32(test_progress_listener_calls, 5U, "a removed listener is not called");

    /* The instances of the axes have no moves to answer: their changes do not call the listener. */
    step_generator_task_set_progress_listener(count_progress_listener);
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_Y);
    run_one_service_pass();
    StepGeneratorInstance_t *const axis_y = &step_generator_task_axis_instances[MOTION_AXIS_Y];
    expect_u32(step_generator_instance_is_ready(axis_y), 1U, "the instance of Y is ready");
    const uint32_t calls_before = test_progress_listener_calls;
    step_generator_instance_halt(axis_y);
    step_generator_task_run_instance(axis_y);
    expect_u32(read_progress(axis_y).stop_resolved, 1U, "the instance of Y resolved its stop");
    expect_u32(test_progress_listener_calls, calls_before, "the snapshot of an axis instance does not call the listener");
    step_generator_task_set_progress_listener(NULL);
}

/* ---- The syringe: the fourth member of the printing stream ---- */

/* A block of X, Y, Z and the syringe: X and Z move by `x_steps` and `z_steps`, the syringe by `syringe_steps`. */
static MotionPlanBlock_t make_block_with_syringe(
    uint32_t sequence,
    int64_t x_steps,
    int64_t y_steps,
    int64_t z_steps,
    int64_t syringe_steps
) {
    MotionPlanBlock_t block = make_block_xyz(sequence, x_steps, y_steps, z_steps);
    const uint64_t syringe_magnitude = (uint64_t)(syringe_steps < 0 ? -syringe_steps : syringe_steps);
    block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE] = syringe_steps;
    if (syringe_magnitude > block.master_step_count) {
        block.master_step_count = syringe_magnitude;
        block.path_length_um = syringe_magnitude;
    }
    if (syringe_steps != 0) {
        block.moving_axes_mask = (uint8_t)(block.moving_axes_mask | MOTION_PLAN_CHANNEL_SYRINGE_MASK);
        if (syringe_steps > 0) {
            block.positive_direction_mask = (uint8_t)(block.positive_direction_mask | MOTION_PLAN_CHANNEL_SYRINGE_MASK);
        }
    }
    return block;
}

static void push_block_with_syringe(
    uint32_t sequence,
    int64_t x_steps,
    int64_t y_steps,
    int64_t z_steps,
    int64_t syringe_steps
) {
    const MotionPlanBlock_t block = make_block_with_syringe(sequence, x_steps, y_steps, z_steps, syringe_steps);
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "block is queued");
}

/* A block of the syringe alone in the printing queue plays on all four channels: the syringe steps, the axes wait. */
static void test_a_syringe_block_plays_in_the_print_stream(void) {
    reset_fixture();
    push_syringe_block(80U, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    expect_u32((uint32_t)run_task_until_jump(&test_task_config), 2U, "task ends at the scripted wait");

    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_prepare_calls[channel], 1U, "every channel of the group is prepared");
        expect_u32(test_start_calls[channel], 1U, "every channel of the group starts");
    }
    expect_u32(test_trigger_calls, 1U, "the group triggers once");
    expect_u32(test_trigger_axes, 4U, "the trigger covers all four channels");
    expect_u32(test_syringe_outputs_calls, 1U, "DIR of the syringe is written once");
    expect_u32(test_syringe_outputs_positive, 1U, "a positive block sets the positive direction");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(test_outputs_calls[axis], 0U, "the driver of an axis is not touched by a block of the syringe");
    }
    expect_u32(test_delay_calls, 1U, "DIR setup time is waited once, for the syringe");
    expect_u32(test_delay_last_ticks, 2U, "the DIR wait lasts two ticks");
    /* 3 steps of 10 us: the interval is 2750 ticks at 275 MHz, the record holds it minus one; the pulse is 5 us. The
     * axes wait out the 30 us block in one idle record of 8250 ticks. */
    expect_u32(test_halves[TEST_CH_SYRINGE][0][0], 2749U, "the ring of the syringe holds the step interval");
    expect_u32(test_halves[TEST_CH_SYRINGE][0][2], 1375U, "the ring of the syringe holds the pulse");
    for (uint32_t channel = 0U; channel < TEST_CHANNELS; ++channel) {
        expect_u32(test_halves[channel][0][0], 8249U, "an axis waits out the block in one idle record");
        expect_u32(test_halves[channel][0][2], 0U, "an axis has no pulse in a block of the syringe");
    }
    /* The block moves no axis, so no axis is watched: the switches of an axis that only idles stop nothing. The syringe
     * has no switch and its watch is ignored, so only the three axes are set. */
    expect_u32(test_watched_axes, 0U, "no axis is watched while only the syringe moves");
    expect_u32(test_watched_axes_calls, 3U, "the watched state is set for the three axes, not for the syringe");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_watched_at_prepare[channel], 0U, "the axes are narrowed before every channel is prepared");
    }
    expect_u32(test_limit_listener_calls[MOTION_AXIS_X], 1U, "the switch of X is still taken by the stream");
    expect_true(
        test_stream_owner[TEST_CH_SYRINGE] == test_print_context(TEST_MEMBER_SYRINGE),
        "the print instance owns the channel of the syringe"
    );
    expect_u32(test_wrong_axis_calls, 0U, "every call went to a known channel");

    expect_u32(step_generator_task_instance.diagnostics.streams_started, 1U, "one stream started");
    expect_u32(step_generator_task_instance.diagnostics.blocks_started, 1U, "it started the block");
    expect_u32(step_generator_task_instance.diagnostics.blocks_completed, 1U, "it generated the whole block");
    expect_u32(step_generator_task_instance.diagnostics.halves_filled, 2U, "both halves of the ring are filled");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "no generator error");
    expect_u32(step_generator_task_instance.diagnostics.axis_errors, 0U, "no axis error");
    expect_u32(step_generator_task_instance.halted, 0U, "the print instance feeds");
    PlanBufferDiagnostics_t queue = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &queue);
    expect_u32(queue.dequeued_total, 1U, "the block left the print queue");

    /* A retract block sets the negative direction, again on the syringe only. */
    reset_fixture();
    push_syringe_block(81U, -3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_syringe_outputs_calls, 1U, "DIR of the syringe is written once");
    expect_u32(test_syringe_outputs_positive, 0U, "a negative block sets the negative direction");
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 1U, "the channel of the syringe starts");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(test_outputs_calls[axis], 0U, "no driver of an axis is written");
    }
    expect_u32(test_halves[TEST_CH_SYRINGE][0][2], 1375U, "the retract has a pulse as well");
}

/* The directions follow the channels that a block moves: each driver gets its own. */
static void test_a_block_of_the_axes_and_the_syringe_sets_the_directions_of_the_moving_channels(void) {
    reset_fixture();
    push_block_with_syringe(82U, 3, 0, -3, 3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_outputs_calls[MOTION_AXIS_X], 1U, "X driver is set");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 1U, "X goes the positive way");
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 0U, "Y is not moved, its driver is left alone");
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], 1U, "Z driver is set");
    expect_u32(test_outputs_positive[MOTION_AXIS_Z], 0U, "Z goes the negative way");
    expect_u32(test_syringe_outputs_calls, 1U, "the driver of the syringe is set");
    expect_u32(test_syringe_outputs_positive, 1U, "the syringe goes the positive way");
    expect_u32(test_delay_calls, 3U, "DIR setup time is waited once per moving channel");
    expect_u32(test_halves[TEST_CH_X][0][2], 1375U, "X has a pulse");
    expect_u32(test_halves[TEST_CH_Y][0][2], 0U, "Y has no pulse");
    expect_u32(test_halves[TEST_CH_Z][0][2], 1375U, "Z has a pulse");
    expect_u32(test_halves[TEST_CH_SYRINGE][0][2], 1375U, "the syringe has a pulse");
    expect_u32(test_watched_axes, (1U << MOTION_AXIS_X) | (1U << MOTION_AXIS_Z), "X and Z are watched, Y is not");
    expect_u32(test_trigger_axes, 4U, "all four channels start together");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "no generator error");

    /* The syringe retracts while Y goes forward. */
    reset_fixture();
    push_block_with_syringe(83U, 0, 3, 0, -3);
    test_wait_script_count = 1U;
    test_wait_results[0] = STEP_GENERATOR_TASK_FLAG_BLOCK_READY;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_outputs_calls[MOTION_AXIS_Y], 1U, "Y driver is set");
    expect_u32(test_outputs_positive[MOTION_AXIS_Y], 1U, "Y goes the positive way");
    expect_u32(test_syringe_outputs_calls, 1U, "the driver of the syringe is set");
    expect_u32(test_syringe_outputs_positive, 0U, "the syringe goes the negative way");
    expect_u32(test_outputs_calls[MOTION_AXIS_X] + test_outputs_calls[MOTION_AXIS_Z], 0U, "X and Z are left alone");
    expect_u32(test_watched_axes, 1U << MOTION_AXIS_Y, "only Y is watched");
}

/* DIR changes only while the stream is stopped: a block that turns the syringe waits for the stream to end. */
static void test_a_syringe_direction_change_waits_for_the_stream_to_stop(void) {
    reset_fixture();
    push_syringe_block(84U, 3);
    push_syringe_block(85U, -3);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(test_syringe_outputs_calls, 1U, "the stream sets DIR of the syringe once");
    expect_u32(test_syringe_outputs_positive, 1U, "the first block decides the positive direction");
    expect_u32(step_generator_task_instance.diagnostics.blocks_started, 1U, "only the first block starts");
    expect_u32(step_generator_task_instance.diagnostics.streams_started, 1U, "one stream is open");
    PlanBufferDiagnostics_t queue = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &queue);
    expect_u32(queue.queued, 1U, "the block that turns the syringe stays in the queue");

    /* The first half has played: the stream carries no block any more and is not stopped yet. */
    release_half(0U);
    run_print_pass();
    expect_u32(test_syringe_outputs_calls, 1U, "DIR is not changed while the stream runs");
    expect_u32(step_generator_task_instance.diagnostics.streams_stopped, 0U, "the stream is not stopped yet");

    /* Both halves of every lane are idle now: the stream stops and the next one opens with the other direction. */
    release_half(1U);
    run_print_pass();
    expect_u32(step_generator_task_instance.diagnostics.streams_stopped, 1U, "the first stream stopped");
    expect_u32(step_generator_task_instance.diagnostics.streams_started, 2U, "a second stream opened");
    expect_u32(test_syringe_outputs_calls, 2U, "DIR of the syringe is written again");
    expect_u32(test_syringe_outputs_positive, 0U, "the second block turns the syringe back");
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 2U, "the channel of the syringe starts a second time");
    expect_u32(step_generator_task_instance.diagnostics.blocks_started, 2U, "the second block started");
    expect_u32(step_generator_task_instance.halted, 0U, "a direction change is no error");
}

/* The highest STEP rate of an instance: the timer clock over the longer of pulse + low time and the shortest record. */
static void test_the_maximum_step_rate_follows_the_timing_of_the_generator(void) {
    expect_u32(step_generator_instance_get_max_step_rate_hz(NULL), 0U, "a NULL instance has no rate");

    /* An instance that is not set up does not know its timing. */
    reset_fixture();
    expect_u32(
        step_generator_instance_get_max_step_rate_hz(&step_generator_task_instance),
        0U,
        "an instance that is not ready has no rate"
    );

    /* 275 MHz, pulse 5000 ns = 1375 ticks, low 2500 ns = 687.5 -> 688 ticks, the shortest record 2000 ns = 550 ticks:
     * pulse plus low is 2063 ticks and wins. 2063 ticks last 2063e9 / 275e6 = 7501.8 ns, so a step needs 7502 whole
     * nanoseconds; 1e9 / 7502 = 133297.8, rounded down 133297 (7502 * 133297 = 999994094, 7502 * 133298 > 1e9). */
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_instance_get_max_step_rate_hz(&step_generator_task_instance),
        133297U,
        "the rate is the clock over pulse plus low time, in whole nanoseconds"
    );

    /* The shortest record wins: 400 ns + 400 ns are 110 + 110 ticks, the record is 550 ticks = 2000 ns exactly, so a step
     * needs 2000 ns and 1e9 / 2000 = 500000. */
    reset_fixture();
    test_task_config.step_pulse_width_ns = 400U;
    test_task_config.step_low_time_ns = 400U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_instance_get_max_step_rate_hz(&step_generator_task_instance),
        500000U,
        "the rate is the clock over the shortest record when it is longer than pulse plus low time"
    );

    /* 137.5 MHz, the default timing: pulse 687.5 -> 688 ticks, low 343.75 -> 344 ticks, the record 275 ticks; the sum is
     * 1032 ticks, which last 1032e9 / 137.5e6 = 7505.45 ns, so a step needs 7506 ns; 1e9 / 7506 = 133226.7 -> 133226
     * (7506 * 133226 = 999994356). */
    reset_fixture();
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_timer_clock_hz[channel] = TEST_TIMER_CLOCK_HZ / 2U;
    }
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_instance_get_max_step_rate_hz(&step_generator_task_instance),
        133226U,
        "the rate follows the clock of the timers of the instance"
    );

    /* The rate of one instance is that of its own timing: a single-axis instance on the same clock reads the same. */
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_Y);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(
        step_generator_instance_get_max_step_rate_hz(step_generator_task_get_axis_instance(MOTION_AXIS_Y)),
        133297U,
        "the instance of Y has the same rate on the same timing"
    );
    expect_u32(
        step_generator_instance_get_max_step_rate_hz(step_generator_task_get_axis_instance(MOTION_AXIS_X)),
        0U,
        "an instance of an axis without a queue is not ready and has no rate"
    );

}

/* A block of the syringe alone at a given rate. */
static void push_syringe_block_at_rate(uint32_t sequence, int64_t steps, uint32_t rate_hz) {
    MotionPlanBlock_t block = make_syringe_block(sequence, steps);
    block.step_event_rate_numerator = rate_hz;
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "syringe block is queued");
}

/* The rate that the instance reports is what its real generator plays: a block at that rate runs without a generator
 * error, a block well above it stops the stream. */
static void test_the_reported_maximum_rate_is_played_by_the_generator(void) {
    reset_fixture();
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    const uint32_t max_rate_hz = step_generator_instance_get_max_step_rate_hz(&step_generator_task_instance);
    expect_u32(max_rate_hz, 133297U, "the rate of the default timing");

    /* 60 steps at 133297 Hz: the duration is 60e9 / 133297 = 450122.66 ns, rounded up to 450123 ns, so the steps last
     * 7502 ns (57 of them) or 7503 ns (3 of them). 7502 ns are 2063.05 ticks at 275 MHz: never less than the 2063 ticks
     * of pulse plus low time. */
    reset_fixture();
    push_syringe_block_at_rate(90U, 60, max_rate_hz);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.halted, 0U, "a block at the reported rate does not halt the stream");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 0U, "the generator accepts the reported rate");
    expect_u32(step_generator_task_instance.feeder_status, 0U, "the feeder reports no error");
    expect_u32(step_generator_task_instance.diagnostics.blocks_completed, 1U, "the whole block is generated");
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 1U, "the stream starts");
    uint32_t ticks_total = 0U;
    for (uint32_t record = 0U; record < 60U; ++record) {
        const uint16_t *const words = &test_halves[TEST_CH_SYRINGE][0][record * 3U];
        expect_true(words[0] >= 2062U, "every step lasts at least the 2063 ticks of pulse plus low time");
        expect_u32(words[2], 1375U, "every step has its pulse");
        ticks_total += (uint32_t)words[0] + 1U;
    }
    /* The steps add up to 450123 ns: 450123 * 275 MHz = 123783.825 ticks, whole ticks 123783. */
    expect_u32(ticks_total, 123783U, "the steps last as long as the block");
    expect_u32(test_halves[TEST_CH_SYRINGE][0][60U * 3U + 2U], 0U, "the record after the steps is idle");

    /* 133400 Hz is well above it: the steps last 7496 or 7497 ns, which are about 2061 ticks, fewer than 2063. The
     * generator refuses the first step and the stream halts with the feeder error. (One hertz above the reported rate
     * is not used: the generator rounds the duration up and carries the tick remainder, so on a given count of steps it
     * may still pass.) */
    reset_fixture();
    push_syringe_block_at_rate(91U, 60, 133400U);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.halted, 1U, "a block above the reported rate halts the stream");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
        "the cause is a feeder error"
    );
    expect_u32(
        step_generator_task_instance.feeder_status,
        (uint32_t)STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "the feeder names the generator"
    );
    expect_u32(
        step_generator_task_instance.feeder_generator_status,
        (uint32_t)STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT,
        "the generator refused a step that is too short"
    );
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "the generator error is counted");
    expect_u32(test_start_calls[TEST_CH_SYRINGE], 0U, "no stream starts");
}

/* STOP while the stream of the syringe runs: all four channels are stopped, and the reset frees them. */
static void test_stop_during_a_syringe_stream_stops_all_four_channels(void) {
    reset_fixture();
    push_syringe_block(86U, 3);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_task_instance.channels_held, 1U, "the print instance holds its channels");
    expect_u32(test_axis_streaming[TEST_CH_SYRINGE], 1U, "the channel of the syringe streams");

    step_generator_task_halt_all();
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_u32(test_stop_calls[channel], 1U, "STOP stops every channel of the print instance once");
        expect_u32(test_axis_streaming[channel], 0U, "the channel is at rest after STOP");
    }
    expect_u32(step_generator_task_instance.halted, 1U, "STOP halts the print instance");

    step_generator_task_resume_all_after_stop();
    run_print_pass();
    expect_u32(step_generator_task_instance.halted, 0U, "the reset lifts the halt");
    expect_u32(step_generator_task_instance.diagnostics.stop_resets, 1U, "the reset is counted");
    expect_u32(step_generator_task_instance.channels_held, 0U, "the reset lets go of the channels");
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        expect_true(test_stream_owner[channel] == NULL, "the channel is free after the reset");
    }
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "the reset stops nothing again");

    /* The channel of the syringe still runs after STOP (a braking ramp): the reset fails and the channels stay held. */
    reset_fixture();
    push_syringe_block(87U, 3);
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    test_axis_stuck[TEST_CH_SYRINGE] = 1U;
    step_generator_task_halt_all();
    expect_u32(test_stop_calls[TEST_CH_SYRINGE], 1U, "STOP asks the syringe to stop");
    expect_u32(test_axis_streaming[TEST_CH_SYRINGE], 1U, "the syringe goes on after the stop");
    expect_u32(test_axis_streaming[TEST_CH_X], 0U, "the other channels are at rest");
    step_generator_task_resume_all_after_stop();
    run_print_pass();
    expect_u32(step_generator_task_instance.halted, 1U, "the reset fails while the syringe runs");
    expect_u32(
        step_generator_task_instance.halt_cause,
        STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED,
        "the cause is the failed reset"
    );
    expect_u32(step_generator_task_instance.reset_failures, 1U, "the failed reset is counted");
    expect_u32(step_generator_task_instance.channels_held, 1U, "the instance goes on holding its channels");
    expect_true(
        test_stream_owner[TEST_CH_SYRINGE] == test_print_context(TEST_MEMBER_SYRINGE),
        "the channel of the syringe is still owned"
    );
    /* The syringe is at rest at last: the next reset succeeds. */
    test_axis_stuck[TEST_CH_SYRINGE] = 0U;
    test_axis_streaming[TEST_CH_SYRINGE] = 0U;
    step_generator_task_resume_all_after_stop();
    run_print_pass();
    expect_u32(step_generator_task_instance.halted, 0U, "the second reset lifts the halt");
    expect_u32(step_generator_task_instance.channels_held, 0U, "the second reset lets go of the channels");
}

/* The tables of the members: which driver, which axis a halt blames, which channel of the plan block. */
static void test_the_member_tables_name_the_channels(void) {
    for (uint32_t member = 0U; member < 3U; ++member) {
        expect_true(
            step_generator_channels_print_members[member].driver == &step_generator_channels_axis_driver,
            "an axis member uses the axis driver"
        );
        expect_u32(step_generator_channels_print_members[member].id, member, "the id of an axis member is its MotionAxis_t");
        expect_u32(
            step_generator_channels_print_members[member].reported_axis,
            member,
            "a halt of an axis member blames that axis"
        );
        expect_u32(step_generator_channels_print_planner_channels[member], member, "an axis member plays its plan channel");
        expect_true(
            step_generator_channels_axis_members[member].driver == &step_generator_channels_axis_driver,
            "a single-axis member uses the axis driver"
        );
        expect_u32(step_generator_channels_axis_members[member].id, member, "the id of a single-axis member is its axis");
        expect_u32(
            step_generator_channels_axis_members[member].reported_axis,
            member,
            "a halt of a single-axis member blames its axis"
        );
        expect_u32(step_generator_channels_axis_planner_channels[member], member, "a single-axis member plays its plan channel");
    }
    expect_true(
        step_generator_channels_print_members[TEST_MEMBER_SYRINGE].driver == &step_generator_channels_syringe_driver,
        "member 3 uses the syringe driver"
    );
    expect_u32(
        step_generator_channels_print_members[TEST_MEMBER_SYRINGE].reported_axis,
        STEP_GENERATOR_CHANNELS_NO_AXIS,
        "a halt of the syringe blames no axis"
    );
    expect_u32(
        step_generator_channels_print_planner_channels[TEST_MEMBER_SYRINGE],
        MOTION_PLAN_CHANNEL_SYRINGE,
        "the syringe plays the syringe channel of the plan block"
    );
}

/* What the static instances are before any test touches them: the policy and the tables of each. */
static void test_the_static_instances_are_defined_as_designed(void) {
    /* The instances are built afresh by the initializer macros, so the result does not depend on what ran before. */
    reset_fixture();
    expect_true(step_generator_task_instance.members == step_generator_channels_print_members, "print members");
    expect_true(
        step_generator_task_instance.member_planner_channels == step_generator_channels_print_planner_channels,
        "print plan channels"
    );
    expect_u32(step_generator_task_instance.member_count, 4U, "the print instance has four members");
    expect_u32(step_generator_task_instance.group.member_count, 4U, "its group has four members");
    expect_u32(step_generator_task_instance.reports_progress, 1U, "only the print instance reports its progress");
    for (uint32_t axis = 0U; axis < STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT; ++axis) {
        const StepGeneratorInstance_t *const instance = &step_generator_task_axis_instances[axis];
        expect_true(instance->members == &step_generator_channels_axis_members[axis], "single-axis member");
        expect_true(
            instance->member_planner_channels == &step_generator_channels_axis_planner_channels[axis],
            "single-axis plan channel"
        );
        expect_u32(instance->member_count, 1U, "a single-axis instance has one member");
        expect_u32(instance->group.member_count, 1U, "its group has one member");
        expect_u32(instance->reports_progress, 0U, "a single-axis instance reports no progress");
    }
}

/* The reference of the timing is the first member of an instance: an instance of Y is timed by Y itself. */
static void test_a_single_axis_instance_is_timed_by_its_own_channel(void) {
    reset_fixture();
    init_axis_plan_buffer(MOTION_AXIS_Y);
    test_timer_clock_hz[TEST_CH_X] = TEST_TIMER_CLOCK_HZ / 2U;
    test_wait_script_count = 0U;
    (void)run_task_until_jump(&test_task_config);
    expect_u32(step_generator_instance_is_ready(step_generator_task_get_print_instance()), 0U, "X on another clock: not ready");
    expect_u32(step_generator_task_instance.diagnostics.generator_errors, 1U, "the mismatch is counted");
    expect_u32(step_generator_task_instance.halted, 1U, "the print instance halts");
    expect_u32(
        step_generator_instance_is_ready(step_generator_task_get_axis_instance(MOTION_AXIS_Y)),
        1U,
        "the one-lane instance of Y is ready on its own clock"
    );
}

static void test_halt_axis_is_the_reported_axis_of_the_member(void) {
    reset_fixture();
    const uint32_t fault = (uint32_t)STEP_STREAM_AXIS_FAULT_DMA;

    step_generator_task_group_fault(&step_generator_task_instance, TEST_MEMBER_SYRINGE, fault);
    StepGeneratorProgress_t progress = read_progress(&step_generator_task_instance);
    expect_u32(step_generator_task_instance.halt_member, TEST_MEMBER_SYRINGE, "the fault is member 3, the syringe");
    expect_u32(progress.halt_cause, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, "a ring fault is the cause");
    expect_u32(progress.halt_axis, STEP_GENERATOR_TASK_NO_AXIS, "the syringe is no axis to blame");
    expect_u32(progress.halt_detail, fault, "the detail is the fault code");

    /* The members of the axes in the printing instance blame their own axis. */
    reset_fixture();
    step_generator_task_group_fault(&step_generator_task_instance, 0U, fault);
    expect_u32(read_progress(&step_generator_task_instance).halt_axis, (uint32_t)MOTION_AXIS_X, "member 0 blames X");
    reset_fixture();
    step_generator_task_group_fault(&step_generator_task_instance, 2U, fault);
    expect_u32(read_progress(&step_generator_task_instance).halt_axis, (uint32_t)MOTION_AXIS_Z, "member 2 blames Z");

    step_generator_task_group_fault(&step_generator_task_axis_instances[MOTION_AXIS_X], 0U, fault);
    progress = read_progress(&step_generator_task_axis_instances[MOTION_AXIS_X]);
    expect_u32(progress.halt_axis, (uint32_t)MOTION_AXIS_X, "the lane of the instance of X blames X");

    /* The lane 0 of the instance of Z is Z, not the axis with the number of the lane. */
    step_generator_task_group_fault(&step_generator_task_axis_instances[MOTION_AXIS_Z], 0U, fault);
    progress = read_progress(&step_generator_task_axis_instances[MOTION_AXIS_Z]);
    expect_u32(progress.halt_axis, (uint32_t)MOTION_AXIS_Z, "the lane of the instance of Z blames Z");
}

/* A switch of an axis names no member of the syringe: it is not counted and does not latch. */
static void test_a_switch_report_does_not_reach_a_member_of_the_syringe(void) {
    reset_fixture();
    StepGeneratorInstance_t *const print = &step_generator_task_instance;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        step_generator_task_limit_reached(print, (MotionAxis_t)axis, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT);
        expect_u32(print->limit_trips[axis], 1U, "the switch of an axis is counted for that axis member");
    }
    expect_u32(print->limit_trips[TEST_MEMBER_SYRINGE], 0U, "no switch of an axis is counted for the syringe member");
    expect_u32(print->halted, 0U, "a switch that finds the group idle halts nothing");
}

static void test_axis_driver_delegates_to_the_axis_hardware(void) {
    reset_fixture();
    const StepGeneratorChannelDriver_t *const driver = &step_generator_channels_axis_driver;

    expect_true(driver->stream(MOTION_AXIS_X) == (StepStreamAxis_t *)&test_axis_markers[TEST_CH_X], "the stream of X");
    expect_true(driver->stream(MOTION_AXIS_Y) == (StepStreamAxis_t *)&test_axis_markers[TEST_CH_Y], "the stream of Y");
    expect_true(driver->stream(MOTION_AXIS_Z) == (StepStreamAxis_t *)&test_axis_markers[TEST_CH_Z], "the stream of Z");

    /* A direction is written when the axis ends in a status it can start from; any other is a refusal. */
    const struct {
        MotionAxisHardwareStatus_t status;
        uint8_t written;
    } directions[] = {
        {MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_READY, 1U},
        {MOTION_AXIS_HARDWARE_STATUS_STARTING, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_WAITING, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_RUNNING, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_COMPLETE, 1U},
        {MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_FAULT_UNDERRUN, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_FAULT_CLOCK, 0U},
        {MOTION_AXIS_HARDWARE_STATUS_STOPPED, 1U},
    };
    for (uint32_t index = 0U; index < sizeof(directions) / sizeof(directions[0]); ++index) {
        test_outputs_result = directions[index].status;
        expect_u32(
            driver->set_direction(MOTION_AXIS_Z, 1U),
            directions[index].written,
            "the result of the direction follows the status the axis is left in"
        );
    }
    expect_u32(test_outputs_calls[MOTION_AXIS_Z], (uint32_t)(sizeof(directions) / sizeof(directions[0])), "every write reached Z");
    expect_u32(test_outputs_positive[MOTION_AXIS_Z], 1U, "the direction reached Z");
    expect_u32(test_outputs_calls[MOTION_AXIS_X] + test_outputs_calls[MOTION_AXIS_Y], 0U, "X and Y are not written");
    test_outputs_result = MOTION_AXIS_HARDWARE_STATUS_READY;
    expect_u32(driver->set_direction(MOTION_AXIS_X, 0U), 1U, "a negative direction is written");
    expect_u32(test_outputs_positive[MOTION_AXIS_X], 0U, "the negative direction reached X");
    expect_u32(test_syringe_outputs_calls, 0U, "the DIR of the syringe is not written by an axis");

    /* The status and the stop reason of the axis named by the id. */
    test_hardware_status[MOTION_AXIS_X] = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    test_hardware_stop_reason[MOTION_AXIS_X] = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    test_hardware_status[MOTION_AXIS_Y] = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    test_hardware_stop_reason[MOTION_AXIS_Y] = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    uint32_t status = UINT32_MAX;
    uint32_t stop_reason = UINT32_MAX;
    driver->get_hardware(MOTION_AXIS_Y, &status, &stop_reason);
    expect_u32(status, (uint32_t)MOTION_AXIS_HARDWARE_STATUS_STOPPED, "the status of Y");
    expect_u32(stop_reason, (uint32_t)MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, "the stop reason of Y");

    /* The watched state of one axis. */
    test_watched_axes = TEST_ALL_AXES_WATCHED;
    driver->set_watched(MOTION_AXIS_Y, 0U);
    expect_u32(test_watched_axes, 0x05U, "Y is no longer watched");
    expect_u32(test_watched_axes_calls, 1U, "one call reached the axis hardware");
    driver->set_watched(MOTION_AXIS_Y, 1U);
    expect_u32(test_watched_axes, TEST_ALL_AXES_WATCHED, "Y is watched again");
    expect_u32(test_watched_axes_calls, 2U, "the second call reached the axis hardware");

    /* The switch listener: the answer is the one of the axis hardware. */
    int owner_a = 0;
    int owner_b = 0;
    expect_u32(driver->set_limit_listener(MOTION_AXIS_Z, test_foreign_limit, &owner_a), 1U, "the free switch of Z is taken");
    expect_true(test_limit_listeners[MOTION_AXIS_Z] == test_foreign_limit, "the listener of Z is the offered one");
    expect_true(test_limit_contexts[MOTION_AXIS_Z] == &owner_a, "the owner of the switch of Z is the offered one");
    expect_u32(driver->set_limit_listener(MOTION_AXIS_Z, test_foreign_limit, &owner_b), 0U, "a switch another owner holds is refused");
    expect_true(test_limit_contexts[MOTION_AXIS_Z] == &owner_a, "the refusal leaves the owner");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_Z], 2U, "both offers reached the axis hardware");
    expect_u32(driver->set_limit_listener(MOTION_AXIS_Z, NULL, &owner_a), 1U, "the owner lets go of the switch");
    expect_true(test_limit_listeners[MOTION_AXIS_Z] == NULL, "the switch of Z is free again");
    expect_u32(test_limit_releases[MOTION_AXIS_Z], 1U, "the release reached the axis hardware");
    expect_u32(test_limit_listener_calls[MOTION_AXIS_X] + test_limit_listener_calls[MOTION_AXIS_Y], 0U, "X and Y were not offered a listener");
}

static void test_syringe_driver_drives_only_the_syringe(void) {
    reset_fixture();
    const StepGeneratorChannelDriver_t *const driver = &step_generator_channels_syringe_driver;

    expect_true(driver->stream(0U) == step_stream_axis_get_syringe(), "the stream is the channel of the syringe");
    expect_true(driver->stream(2U) == (StepStreamAxis_t *)&test_axis_markers[TEST_CH_SYRINGE], "whatever the id, it is the syringe");

    /* DIR goes to the driver of the syringe with the direction as given, and its answer is the answer of the member. */
    expect_u32(driver->set_direction(0U, 1U), 1U, "a written direction is reported");
    expect_u32(test_syringe_outputs_calls, 1U, "the syringe driver was asked once");
    expect_u32(test_syringe_outputs_positive, 1U, "the positive direction reached it");
    expect_u32(driver->set_direction(0U, 0U), 1U, "a negative direction is written");
    expect_u32(test_syringe_outputs_positive, 0U, "the negative direction reached it");
    test_syringe_outputs_result = 0U;
    expect_u32(driver->set_direction(0U, 1U), 0U, "a refusal of the syringe driver is reported");
    expect_u32(test_syringe_outputs_calls, 3U, "all three writes reached the syringe driver");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(test_outputs_calls[axis], 0U, "no driver of an axis is written");
    }

    /* No switches: the watched state and the listener of an axis are left alone. */
    test_watched_axes = 0x05U;
    driver->set_watched(0U, 0U);
    driver->set_watched(0U, 1U);
    expect_u32(test_watched_axes, 0x05U, "the watched axes are unchanged");
    expect_u32(test_watched_axes_calls, 0U, "no watch is set");
    int owner = 0;
    expect_u32(driver->set_limit_listener(0U, test_foreign_limit, &owner), 1U, "a listener is always taken");
    expect_u32(driver->set_limit_listener(0U, NULL, &owner), 1U, "a release always succeeds");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(test_limit_listener_calls[axis], 0U, "no axis is offered a listener");
        expect_u32(test_limit_releases[axis], 0U, "no axis is asked to let go");
        expect_true(test_limit_listeners[axis] == NULL, "no axis keeps a listener");
    }

    /* No hardware axis: both values are 0 whatever the axes report. */
    test_hardware_status[MOTION_AXIS_X] = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    test_hardware_stop_reason[MOTION_AXIS_X] = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    uint32_t status = UINT32_MAX;
    uint32_t stop_reason = UINT32_MAX;
    driver->get_hardware(0U, &status, &stop_reason);
    expect_u32(status, 0U, "the syringe has no hardware status");
    expect_u32(stop_reason, 0U, "the syringe has no stop reason");
}

int main(void) {
    for (uint32_t channel = 0U; channel < TEST_ALL_CHANNELS; ++channel) {
        test_axis_markers[channel] = (int)channel;
    }
    test_the_static_instances_are_defined_as_designed();
    test_the_member_tables_name_the_channels();
    test_task_streams_a_block();
    test_instance_api();
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
    test_halt_member_is_the_axis_whose_switch_fired();
    test_limit_in_idle_group_is_counted_but_not_latched();
    test_limit_in_priming_group_latches_and_real_limit_outranks_release();
    test_limit_in_priming_group_blames_the_real_limit_on_halt();
    test_limit_in_running_group_latches_a_release();
    test_limit_in_stopping_or_faulted_group_is_not_latched();
    test_group_ends_with_its_stream();
    test_stop_reset_clears_a_group_fault();
    test_reset_is_refused_while_a_channel_runs();
    test_brake_ramp_is_built_from_the_driver_timing();
    test_axis_instances_build_their_ramps_when_the_print_instance_cannot_be_built();
    test_brake_is_skipped_or_counted();
    test_callbacks_act_on_their_own_instance();
    test_config_without_plan_buffer_halts_the_task();
    test_ports_read_the_queue_of_the_config();
    test_two_instances_own_different_axes();
    test_axis_instance_plays_a_block_on_its_own_lane();
    test_two_axis_instances_stream_together();
    test_limit_switch_reaches_only_its_instance();
    test_refused_claim_fails_the_stream_and_spares_the_owner();
    test_release_lets_go_only_of_its_own();
    test_release_is_retried_while_a_channel_streams();
    test_halt_and_busy_are_per_instance();
    test_axis_instance_without_a_queue_fails_alone();
    test_instance_lookup_and_queues();
    test_halt_stops_only_the_channels_the_instance_holds();
    test_stop_reset_frees_the_channels();
    test_instance_is_ready_once_its_feeder_is_built();
    test_reset_count_rises_by_one_per_finished_reset();
    test_reset_count_rises_when_the_reset_fails();
    test_progress_of_a_fresh_instance_and_null_arguments();
    test_generation_rises_with_every_halt();
    test_halt_wakes_the_task_when_it_runs();
    test_running_stream_advances_played_blocks_by_block_ends();
    test_limit_stop_is_resolved_when_every_channel_rests();
    test_stop_with_blocks_only_in_the_plan_names_the_head_of_the_plan();
    test_stop_without_any_block_has_nothing_unplayed();
    test_halt_detail_follows_the_cause();
    test_halt_detail_carries_the_generator_status();
    test_progress_listener_is_called_when_the_print_snapshot_changes();
    test_a_block_is_played_only_when_the_syringe_reaches_its_end();
    test_a_syringe_block_plays_in_the_print_stream();
    test_a_block_of_the_axes_and_the_syringe_sets_the_directions_of_the_moving_channels();
    test_a_syringe_direction_change_waits_for_the_stream_to_stop();
    test_the_maximum_step_rate_follows_the_timing_of_the_generator();
    test_the_reported_maximum_rate_is_played_by_the_generator();
    test_stop_during_a_syringe_stream_stops_all_four_channels();
    test_a_single_axis_instance_is_timed_by_its_own_channel();
    test_halt_axis_is_the_reported_axis_of_the_member();
    test_a_switch_report_does_not_reach_a_member_of_the_syringe();
    test_stream_diagnostics_list_only_the_axes();
    test_direction_waits_the_settle_time();
    test_axis_driver_delegates_to_the_axis_hardware();
    test_syringe_driver_drives_only_the_syringe();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Step generator task unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Step generator task unit tests passed.\n");
    return 0;
}

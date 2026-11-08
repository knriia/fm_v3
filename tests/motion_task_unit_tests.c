#include "command/command_queue.h"
#include "motion/axis_bench.h"
#include "motion/axis_calibration_run.h"
#include "motion/calibration_store.h"
#include "motion/plan_buffer.h"
#include "motion/step_executor.h"
#include "motion_axis_hardware.h"
#include "motion_task.h"

#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    osStatus_t status;
    CommandDTO_t command;
} TestQueueRead_t;

static uint32_t test_failures;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint8_t test_jump_on_delay;
static uint8_t test_block_when_empty;
static uint32_t test_delay_calls;
static uint32_t test_last_delay_ms;
static uint32_t test_receive_calls;
static uint32_t test_last_receive_timeout;
static uint32_t test_receive_script_count;
static uint32_t test_receive_script_position;
static TestQueueRead_t test_receive_script[16];

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

static void expect_i64(int64_t actual, int64_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %lld, got %lld)\n", message, (long long)expected, (long long)actual);
    }
}

typedef struct {
    uint8_t apply;
    MotionAxisHardwareStatus_t status;
    MotionAxisHardwareStopReason_t stop_reason;
    uint8_t start_limit_active;
} TestXHardwareState_t;

/* State and call counts of one faked axis. */
typedef struct {
    MotionAxisHardwareStatus_t status;
    MotionAxisHardwareStopReason_t stop_reason;
    uint8_t start_limit_active;
    uint8_t end_limit_active;
    uint32_t init_calls;
    uint32_t init_receive_calls;
    uint32_t stop_requests;
    uint32_t clear_fault_calls;
    uint32_t timing_peak_resets;
    uint32_t ready_after_home_calls;
    uint32_t bench_starts;
    int32_t bench_last_signed_steps;
    uint32_t bench_last_initial_rate_hz;
    uint32_t bench_last_target_rate_hz;
    uint32_t bench_last_acceleration;
    uint32_t release_stop_arms;
    /* 1-based index of the profile start that fails, leaving the hardware in failed_start_status. */
    uint32_t failing_start;
    MotionAxisHardwareStatus_t failed_start_status;
    MotionAxisHardwareStartLimitHook_t start_limit_hook;
    int32_t encoder_um;
    /* Hardware state applied just before the matching scripted queue read, i.e. while MotionTask is blocked. */
    TestXHardwareState_t state_script[16];
} FakeAxis_t;

static FakeAxis_t fake_axes[MOTION_AXIS_HARDWARE_COUNT];

/* Most tests drive X; these names keep them short. */
#define fake_x_status fake_axes[MOTION_AXIS_X].status
#define fake_x_stop_reason fake_axes[MOTION_AXIS_X].stop_reason
#define fake_x_start_limit_active fake_axes[MOTION_AXIS_X].start_limit_active
#define fake_x_end_limit_active fake_axes[MOTION_AXIS_X].end_limit_active
#define fake_x_init_calls fake_axes[MOTION_AXIS_X].init_calls
#define fake_x_init_receive_calls fake_axes[MOTION_AXIS_X].init_receive_calls
#define fake_x_stop_requests fake_axes[MOTION_AXIS_X].stop_requests
#define fake_x_clear_fault_calls fake_axes[MOTION_AXIS_X].clear_fault_calls
#define fake_x_ready_after_home_calls fake_axes[MOTION_AXIS_X].ready_after_home_calls
#define fake_x_bench_starts fake_axes[MOTION_AXIS_X].bench_starts
#define fake_x_bench_last_signed_steps fake_axes[MOTION_AXIS_X].bench_last_signed_steps
#define fake_x_release_stop_arms fake_axes[MOTION_AXIS_X].release_stop_arms
#define fake_x_failing_start fake_axes[MOTION_AXIS_X].failing_start
#define fake_x_failed_start_status fake_axes[MOTION_AXIS_X].failed_start_status
#define fake_x_start_limit_hook fake_axes[MOTION_AXIS_X].start_limit_hook
#define fake_encoder_x_um fake_axes[MOTION_AXIS_X].encoder_um
#define test_x_state_script fake_axes[MOTION_AXIS_X].state_script
#define fake_y fake_axes[MOTION_AXIS_Y]
#define fake_z fake_axes[MOTION_AXIS_Z]

static void script_axis_state(
    MotionAxis_t axis,
    uint32_t index,
    MotionAxisHardwareStatus_t status,
    MotionAxisHardwareStopReason_t stop_reason,
    uint8_t start_limit_active
) {
    fake_axes[axis].state_script[index] = (TestXHardwareState_t){
        .apply = 1U,
        .status = status,
        .stop_reason = stop_reason,
        .start_limit_active = start_limit_active,
    };
}

static void script_x_state(
    uint32_t index,
    MotionAxisHardwareStatus_t status,
    MotionAxisHardwareStopReason_t stop_reason,
    uint8_t start_limit_active
) {
    script_axis_state(MOTION_AXIS_X, index, status, stop_reason, start_limit_active);
}

osStatus_t command_queue_receive(CommandDTO_t *command, uint32_t timeout_ms) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        FakeAxis_t *fake = &fake_axes[axis];
        if (test_receive_calls < 16U && fake->state_script[test_receive_calls].apply != 0U) {
            const TestXHardwareState_t *state = &fake->state_script[test_receive_calls];
            fake->status = state->status;
            fake->stop_reason = state->stop_reason;
            fake->start_limit_active = state->start_limit_active;
        }
    }
    ++test_receive_calls;
    test_last_receive_timeout = timeout_ms;
    if (test_receive_script_position < test_receive_script_count) {
        const TestQueueRead_t *read = &test_receive_script[test_receive_script_position++];
        if (read->status == osOK) {
            *command = read->command;
        }
        return read->status;
    }
    if (test_block_when_empty != 0U && test_jump_active != 0U) {
        longjmp(test_jump_buffer, 2);
    }
    return osErrorResource;
}

/* The plan buffer mutex is exercised in its own tests; here it only has to succeed. */
static uint8_t test_plan_buffer_mutex;

osMutexId_t osMutexNew(const osMutexAttr_t *attr) {
    (void)attr;
    return &test_plan_buffer_mutex;
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

void Error_Handler(void) {}

static uint32_t test_block_ready_notifications;
static CalibrationStoreStatus_t test_calibration_load_status;
static MachineCalibration_t test_stored_calibration;

CalibrationStoreStatus_t calibration_store_load(MachineCalibration_t *calibration) {
    if (test_calibration_load_status == CALIBRATION_STORE_STATUS_OK) {
        *calibration = test_stored_calibration;
    }
    return test_calibration_load_status;
}

void step_generator_task_notify_block_ready(void) { ++test_block_ready_notifications; }

static uint32_t test_generator_halts;
static uint32_t test_generator_resumes;
static uint32_t test_queue_discards;
/* After this many 1 ms waits the X axis reports READY; 0 keeps it moving. */
static uint32_t test_axis_settles_after_delays;

static uint8_t test_generator_busy;

uint8_t step_generator_task_is_busy(void) { return test_generator_busy; }

void step_generator_task_halt(void) { ++test_generator_halts; }

void step_generator_task_resume_after_stop(void) { ++test_generator_resumes; }

void command_queue_discard_all(void) { ++test_queue_discards; }

uint32_t osDelay(uint32_t milliseconds) {
    ++test_delay_calls;
    test_last_delay_ms = milliseconds;
    if (test_axis_settles_after_delays != 0U && test_delay_calls == test_axis_settles_after_delays) {
        fake_x_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    }
    if (test_jump_active != 0U && test_jump_on_delay != 0U) {
        longjmp(test_jump_buffer, 3);
    }
    return 0U;
}

MotionAxisHardwareStatus_t motion_axis_hardware_init(MotionAxis_t axis, const MotionAxisHardwareConfig_t *config) {
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->init_calls;
    fake->init_receive_calls = test_receive_calls;
    fake->status = config != NULL ? MOTION_AXIS_HARDWARE_STATUS_READY : MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG;
    return fake->status;
}

static uint32_t test_event_source_registrations[MOTION_AXIS_HARDWARE_COUNT];
static void *test_event_source_contexts[MOTION_AXIS_HARDWARE_COUNT];
static uint32_t test_executor_underruns[MOTION_AXIS_HARDWARE_COUNT];

MotionAxisHardwareStatus_t motion_axis_hardware_set_event_source(MotionAxis_t axis, const MotionEventSource_t *source) {
    ++test_event_source_registrations[axis];
    test_event_source_contexts[axis] = source->context;
    return fake_axes[axis].status;
}

MotionEventSource_t step_executor_event_source(StepExecutor_t *executor) {
    return (MotionEventSource_t){.context = executor};
}

/* motion_task keeps one executor per axis in an array; the X source is bound to its first element. */
uint32_t step_executor_get_underrun_count(const StepExecutor_t *executor) {
    const StepExecutor_t *first = test_event_source_contexts[MOTION_AXIS_X];
    assert(first != NULL && executor >= first && executor < first + MOTION_AXIS_HARDWARE_COUNT);
    return test_executor_underruns[executor - first];
}

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) { return fake_axes[axis].status; }

MotionAxisHardwareStopReason_t motion_axis_hardware_get_last_stop_reason(MotionAxis_t axis) {
    return fake_axes[axis].stop_reason;
}

void motion_axis_hardware_request_stop(MotionAxis_t axis) { ++fake_axes[axis].stop_requests; }

MotionAxisHardwareStatus_t motion_axis_hardware_clear_fault(MotionAxis_t axis) {
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->clear_fault_calls;
    if (fake->status >= MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT &&
        fake->status <= MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING) {
        fake->status = MOTION_AXIS_HARDWARE_STATUS_READY;
    }
    return fake->status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_ready_after_home(MotionAxis_t axis) {
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->ready_after_home_calls;
    if (fake->status == MOTION_AXIS_HARDWARE_STATUS_STOPPED && fake->start_limit_active != 0U) {
        fake->status = MOTION_AXIS_HARDWARE_STATUS_READY;
    }
    return fake->status;
}

uint8_t motion_axis_hardware_start_limit_is_active(MotionAxis_t axis) { return fake_axes[axis].start_limit_active; }

uint8_t motion_axis_hardware_end_limit_is_active(MotionAxis_t axis) { return fake_axes[axis].end_limit_active; }

void motion_axis_hardware_arm_start_limit_release_stop(MotionAxis_t axis) { ++fake_axes[axis].release_stop_arms; }

void motion_axis_hardware_disarm_start_limit_release_stop(MotionAxis_t axis) { (void)axis; }

void motion_axis_hardware_arm_start_limit_hook(MotionAxis_t axis, MotionAxisHardwareStartLimitHook_t hook) {
    fake_axes[axis].start_limit_hook = hook;
}

void motion_axis_hardware_reset_timing_peaks(MotionAxis_t axis) { ++fake_axes[axis].timing_peak_resets; }

void encoder_ox_reset_coordinate(void) {}

int32_t encoder_ox_get_coordinate(void) { return fake_axes[MOTION_AXIS_X].encoder_um; }

void encoder_oy_reset_coordinate(void) {}

int32_t encoder_oy_get_coordinate(void) { return fake_axes[MOTION_AXIS_Y].encoder_um; }

void encoder_oz_reset_coordinate(void) {}

int32_t encoder_oz_get_coordinate(void) { return fake_axes[MOTION_AXIS_Z].encoder_um; }

/* The calibration procedure has its own tests; here it only reports scripted outcomes. */
static AxisCalibrationRunResult_t fake_calibration_begin_result;
static AxisCalibrationRunResult_t fake_calibration_poll_result;
static AxisCalibration_t fake_calibration_measured;
static uint32_t fake_calibration_begin_calls;
static uint32_t fake_calibration_poll_calls;
static uint32_t fake_calibration_abort_calls;
static MotionAxis_t fake_calibration_last_axis;

static const AxisCalibrationRunSettings_t *fake_calibration_last_settings;

AxisCalibrationRunResult_t axis_calibration_run_begin(
    AxisCalibrationRun_t *run,
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *config,
    const AxisCalibrationRunSettings_t *settings
) {
    (void)config;
    fake_calibration_last_settings = settings;
    ++fake_calibration_begin_calls;
    fake_calibration_last_axis = axis;
    run->axis = axis;
    run->phase = fake_calibration_begin_result == AXIS_CALIBRATION_RUN_RESULT_RUNNING
                     ? AXIS_CALIBRATION_RUN_PHASE_SEEK_END
                     : AXIS_CALIBRATION_RUN_PHASE_IDLE;
    return fake_calibration_begin_result;
}

AxisCalibrationRunResult_t axis_calibration_run_poll(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    AxisCalibration_t *result
) {
    (void)config;
    ++fake_calibration_poll_calls;
    if (fake_calibration_poll_result != AXIS_CALIBRATION_RUN_RESULT_RUNNING) {
        run->phase = AXIS_CALIBRATION_RUN_PHASE_IDLE;
    }
    if (fake_calibration_poll_result == AXIS_CALIBRATION_RUN_RESULT_OK) {
        *result = fake_calibration_measured;
    }
    return fake_calibration_poll_result;
}

void axis_calibration_run_abort(AxisCalibrationRun_t *run) {
    ++fake_calibration_abort_calls;
    run->phase = AXIS_CALIBRATION_RUN_PHASE_IDLE;
}

uint8_t axis_calibration_run_is_active(const AxisCalibrationRun_t *run) {
    return run->phase != AXIS_CALIBRATION_RUN_PHASE_IDLE ? 1U : 0U;
}

AxisBenchStatus_t axis_bench_start_profile(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionAxisHardwareStatus_t *hardware_status
) {
    (void)hardware_config;
    FakeAxis_t *fake = &fake_axes[axis];
    fake->bench_last_initial_rate_hz = initial_step_rate_hz;
    fake->bench_last_target_rate_hz = target_step_rate_hz;
    fake->bench_last_acceleration = acceleration_steps_per_s2;
    ++fake->bench_starts;
    /* Both axes are configured with positive motion toward the end switch. */
    fake->bench_last_signed_steps = direction == AXIS_BENCH_TOWARD_END ? (int32_t)steps : -(int32_t)steps;
    if (fake->bench_starts == fake->failing_start) {
        fake->status = fake->failed_start_status;
        *hardware_status = fake->status;
        return AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }
    if (fake->status != MOTION_AXIS_HARDWARE_STATUS_READY && fake->status != MOTION_AXIS_HARDWARE_STATUS_STOPPED &&
        fake->status != MOTION_AXIS_HARDWARE_STATUS_COMPLETE) {
        *hardware_status = fake->status;
        return AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }
    fake->status = MOTION_AXIS_HARDWARE_STATUS_STARTING;
    fake->stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    *hardware_status = fake->status;
    return AXIS_BENCH_STATUS_STARTED;
}

static void reset_receive_script(void) {
    (void)memset(fake_axes, 0, sizeof(fake_axes));
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        fake_axes[axis].init_receive_calls = UINT32_MAX;
    }
    test_jump_active = 0U;
    test_jump_on_delay = 0U;
    test_block_when_empty = 0U;
    test_delay_calls = 0U;
    test_last_delay_ms = 0U;
    test_receive_calls = 0U;
    test_last_receive_timeout = 0U;
    test_receive_script_count = 0U;
    test_receive_script_position = 0U;
    (void)memset(test_receive_script, 0, sizeof(test_receive_script));

    const MotionTaskDiagnostics diagnostics = {0};
    motion_task_test_seed_diagnostics(NULL);
    motion_task_test_seed_diagnostics(&diagnostics);
    motion_task_test_reset_planner();
    plan_buffer_test_reset();
    (void)plan_buffer_init();
    test_block_ready_notifications = 0U;
    test_generator_halts = 0U;
    test_generator_resumes = 0U;
    test_queue_discards = 0U;
    test_axis_settles_after_delays = 0U;
    test_calibration_load_status = CALIBRATION_STORE_STATUS_EMPTY;
    test_stored_calibration = (MachineCalibration_t){0};
    fake_calibration_begin_result = AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    fake_calibration_measured = (AxisCalibration_t){0};
    fake_calibration_begin_calls = 0U;
    fake_calibration_poll_calls = 0U;
    fake_calibration_abort_calls = 0U;
    fake_calibration_last_axis = MOTION_AXIS_X;
}

static void script_read(uint32_t index, osStatus_t status, const CommandDTO_t *command) {
    test_receive_script[index].status = status;
    if (command != NULL) {
        test_receive_script[index].command = *command;
    }
    if (test_receive_script_count <= index) {
        test_receive_script_count = index + 1U;
    }
}

static MotionTaskContext_t valid_motion_context(void) {
    MotionTaskContext_t context = {
        .machine_state =
            {
                .position_um = {0, 0, 0},
                .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
            },
        .planner_config =
            {
                .max_step_event_rate_hz = 50000U,
            },
    };
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        context.planner_config.axis[axis] = (MotionPlannerAxisConfig_t){
            .min_position_um = -100000,
            .max_position_um = 100000,
            .steps_per_um_numerator = 80U,
            .steps_per_um_denominator = 1000U,
            .max_speed_um_per_s = 100000U,
        };
    }
    return context;
}

static int run_motion_task_until_scripted_stop(MotionTaskContext_t *context) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        MotionTask(context);
    }
    test_jump_active = 0U;
    return jumped;
}

static void test_successful_receive_marks_dequeue(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 101U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1234,
                .y = 0,
                .z = 0,
                .speed = 12000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    expect_u32((uint32_t)run_motion_task_until_scripted_stop(&context), 3U, "queue error enters bounded retry path");
    expect_u32(test_last_receive_timeout, osWaitForever, "consumer waits forever for a queue item");
    expect_u32(test_delay_calls, 1U, "queue error applies one retry delay");
    expect_u32(test_last_delay_ms, MOTION_TASK_ERROR_RETRY_DELAY_MS, "retry delay is bounded");

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(NULL);
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, 1U, "successfully received command is counted as dequeued");
    expect_u32(diagnostics.queue_receive_errors, 1U, "queue receive error is counted");
    expect_u32(diagnostics.last_sequence, 101U, "last dequeued sequence is recorded");
    expect_u32(diagnostics.last_command_code, COMMAND_CODE_MOTION_OPERATION, "last dequeued command code is recorded");
    expect_u32(diagnostics.planner_calls, 1U, "planner call is counted");
    expect_u32(diagnostics.planner_blocks_created, 1U, "successful planner result is counted as a block");
    expect_u32(diagnostics.planner_no_motion, 0U, "moving command is not counted as no-motion");
    expect_u32(diagnostics.planner_errors, 0U, "successful planning is not counted as an error");
    expect_u32(diagnostics.last_planner_status, MOTION_PLANNER_STATUS_OK, "last planner status is recorded");
    expect_u32(diagnostics.last_planner_sequence, 101U, "last planner sequence is recorded");

    const MotionPlannerResult_t planner_result = motion_task_test_get_last_planner_result();
    expect_u32(motion_task_test_get_planner_call_count(), 1U, "XYZ command invokes MotionPlanner once");
    expect_u32(planner_result.status, MOTION_PLANNER_STATUS_OK, "valid dequeued XYZ command produces a plan");
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_OK,
        "last planner status is retained for callers"
    );
    expect_u32(planner_result.block.sequence, 101U, "planner receives the command sequence");
    expect_i64(planner_result.block.step_delta[0], 99, "planner receives the X target");
    expect_i64(planner_result.block.step_delta[1], 0, "Y stays in place");
    expect_i64(planner_result.block.step_delta[2], 0, "Z stays in place");
    expect_u32(planner_result.block.feedrate_um_per_s, 12000U, "planner receives the XYZ feedrate");

    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    expect_u32(
        plan_buffer_get_diagnostics(&plan_buffer_diagnostics),
        PLAN_BUFFER_STATUS_OK,
        "plan buffer diagnostics are available"
    );
    expect_u32(plan_buffer_diagnostics.queued, 1U, "successful plan is queued");
    expect_u32(plan_buffer_diagnostics.enqueued_total, 1U, "successful plan enqueue is counted");
    expect_u32(test_block_ready_notifications, 1U, "step generator is woken for the queued plan");
    expect_u32((uint32_t)context.machine_state.position_um[0], 1234U, "planned X becomes the next start");
    expect_u32((uint32_t)context.machine_state.position_um[1], 0U, "planned Y is unchanged");
    expect_u32((uint32_t)context.machine_state.position_um[2], 0U, "planned Z is unchanged");
    MotionPlanBlock_t buffered_block = {0};
    expect_u32(
        plan_buffer_try_pop(&buffered_block),
        PLAN_BUFFER_STATUS_OK,
        "plan is available to the next pipeline block"
    );
    expect_u32(buffered_block.sequence, 101U, "plan buffer preserves source command sequence");
    expect_i64(buffered_block.step_delta[0], 99, "plan buffer preserves planner X output");
    expect_i64(buffered_block.step_delta[1], 0, "plan buffer preserves planner Y output");
    expect_i64(buffered_block.step_delta[2], 0, "plan buffer preserves planner Z output");
}

static void test_fifo_commands_are_dequeued_once(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 90U};
    expect_u32(plan_buffer_try_push(&pending_plan), PLAN_BUFFER_STATUS_OK, "pending plan exists before STOP");
    const CommandDTO_t commands[] = {
        {.sequence = 102U, .code = COMMAND_CODE_HOME, .parameters.home = {.axes = 0x08U}},
        {.sequence = 103U, .code = COMMAND_CODE_STOP},
        {
            .sequence = 104U,
            .code = COMMAND_CODE_SET_TEMPERATURE,
            .parameters.set_temperature = {.heater_id = 2U, .temperature = 215U},
        },
        {.sequence = 105U, .code = COMMAND_CODE_SET_OUTPUT, .parameters.set_output = {.output_id = 4U, .state = 1U}},
        {.sequence = 106U, .code = COMMAND_CODE_CHANGE_TOOL, .parameters.change_tool = {.tool_id = 7U}},
    };
    for (uint32_t index = 0U; index < sizeof(commands) / sizeof(commands[0]); ++index) {
        script_read(index, osOK, &commands[index]);
    }
    test_block_when_empty = 1U;

    MotionTaskContext_t context = {0};
    expect_u32(
        (uint32_t)run_motion_task_until_scripted_stop(&context),
        2U,
        "consumer blocks again after draining scripted commands"
    );
    expect_u32(test_receive_calls, 6U, "consumer performs one receive per command then waits again");
    expect_u32(test_receive_script_position, 5U, "each scripted queue item is removed exactly once");
    expect_u32(test_last_receive_timeout, osWaitForever, "each consumer read blocks indefinitely");

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, 5U, "all valid queue items are counted as dequeued");
    expect_u32(diagnostics.queue_receive_errors, 0U, "blocking on an empty queue is not a receive error");
    expect_u32(diagnostics.last_sequence, 106U, "last dequeued sequence is retained");
    expect_u32(diagnostics.last_command_code, COMMAND_CODE_CHANGE_TOOL, "last dequeued command code is retained");
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "non-XYZ commands do not invoke MotionPlanner");
    expect_u32(diagnostics.planner_calls, 0U, "non-XYZ commands do not increment planner calls");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 0U, "STOP clears pending plans");
    expect_u32(plan_buffer_diagnostics.cancelled_blocks, 1U, "STOP counts cancelled plans");
}

static void test_receive_error_does_not_count_as_dequeue(void) {
    reset_receive_script();
    MotionTaskContext_t context = {0};
    script_read(0U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    expect_u32((uint32_t)run_motion_task_until_scripted_stop(&context), 3U, "receive error retries after delay");
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, 0U, "failed receive does not count as a dequeued command");
    expect_u32(diagnostics.queue_receive_errors, 1U, "failed receive increments error count");
    expect_u32(diagnostics.last_sequence, 0U, "failed receive does not change last sequence");
}

static void test_diagnostic_counters_saturate(void) {
    reset_receive_script();
    const MotionTaskDiagnostics saturated = {
        .commands_dequeued = UINT32_MAX,
        .queue_receive_errors = UINT32_MAX,
        .planner_calls = UINT32_MAX,
        .planner_blocks_created = UINT32_MAX,
        .planner_no_motion = UINT32_MAX,
        .planner_errors = UINT32_MAX,
    };
    motion_task_test_seed_diagnostics(&saturated);

    MotionTaskContext_t valid_context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 107U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1000,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&valid_context);

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, UINT32_MAX, "dequeued counter saturates");
    expect_u32(diagnostics.queue_receive_errors, UINT32_MAX, "queue receive error counter saturates");
    expect_u32(diagnostics.planner_calls, UINT32_MAX, "planner call counter saturates");
    expect_u32(diagnostics.planner_blocks_created, UINT32_MAX, "planner block counter saturates");
    expect_u32(diagnostics.planner_no_motion, UINT32_MAX, "planner no-motion counter remains saturated");
    expect_u32(diagnostics.planner_errors, UINT32_MAX, "planner error counter remains saturated");
}

static void test_xyz_command_reports_unconfigured_planner_inputs(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 91U};
    expect_u32(plan_buffer_try_push(&pending_plan), PLAN_BUFFER_STATUS_OK, "pending plan exists before planner error");
    MotionTaskContext_t context = {0};
    const CommandDTO_t command = {
        .sequence = 108U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1000,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(motion_task_test_get_planner_call_count(), 1U, "XYZ command reaches MotionPlanner with runtime context");
    expect_u32(
        motion_task_test_get_last_planner_result().status,
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "zero-initialized runtime configuration rejects planning safely"
    );
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "task exposes the runtime planner rejection status"
    );
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_calls, 1U, "unconfigured XYZ command increments planner calls");
    expect_u32(diagnostics.planner_blocks_created, 0U, "unconfigured XYZ command creates no block");
    expect_u32(diagnostics.planner_errors, 1U, "unconfigured XYZ command increments planner errors");
    expect_u32(
        diagnostics.last_planner_status,
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "invalid configuration status is recorded"
    );
    expect_u32(diagnostics.last_planner_sequence, 108U, "failed planner sequence is recorded");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 0U, "planner error clears pending plans");
    expect_u32(plan_buffer_diagnostics.cancelled_blocks, 1U, "planner error counts cancelled plans");
}

static void test_xyz_command_without_position_change_counts_no_motion(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 92U};
    expect_u32(
        plan_buffer_try_push(&pending_plan),
        PLAN_BUFFER_STATUS_OK,
        "pending plan exists before no-motion command"
    );
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 111U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 0,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_calls, 1U, "stationary XYZ command invokes planner");
    expect_u32(diagnostics.planner_blocks_created, 0U, "stationary XYZ command creates no block");
    expect_u32(diagnostics.planner_no_motion, 1U, "stationary XYZ command increments no-motion count");
    expect_u32(diagnostics.planner_errors, 0U, "no movement is not counted as a planner error");
    expect_u32(diagnostics.last_planner_status, MOTION_PLANNER_STATUS_NO_MOVEMENT, "no-movement status is recorded");
    expect_u32(diagnostics.last_planner_sequence, 111U, "no-movement sequence is recorded");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 1U, "no-motion result preserves earlier plans");
}

static void test_full_plan_buffer_rejects_new_block_without_losing_existing_plans(void) {
    reset_receive_script();
    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = {.sequence = sequence};
        expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "plan buffer fills before integration check");
    }

    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 112U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1234,
                .y = 0,
                .z = 0,
                .speed = 12000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    MotionTaskDiagnostics motion_diagnostics = {0};
    motion_task_get_diagnostics(&motion_diagnostics);
    expect_u32(motion_diagnostics.planner_blocks_created, 1U, "planner output remains distinct from buffer acceptance");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, PLAN_BUFFER_CAPACITY, "full buffer retains existing plans");
    expect_u32(
        plan_buffer_diagnostics.enqueued_total,
        PLAN_BUFFER_CAPACITY,
        "rejected plan is not counted as enqueued"
    );
    expect_u32(plan_buffer_diagnostics.full_rejections, 1U, "full plan buffer rejection is visible");
    expect_u32(test_block_ready_notifications, 0U, "rejected plan does not wake the step generator");

    MotionPlanBlock_t first_block = {0};
    expect_u32(
        plan_buffer_try_pop(&first_block),
        PLAN_BUFFER_STATUS_OK,
        "existing plans remain consumable after full rejection"
    );
    expect_u32(first_block.sequence, 1U, "full rejection does not replace earlier plans");
}

static void test_xyz_command_with_null_context_reaches_planner_as_invalid_input(void) {
    reset_receive_script();
    const CommandDTO_t command = {
        .sequence = 109U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1000,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(NULL);
    expect_u32(
        motion_task_test_get_planner_call_count(),
        1U,
        "MotionPlanner is called even when task context is absent"
    );
    expect_u32(
        motion_task_test_get_last_planner_result().status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "missing motion context is rejected by MotionPlanner"
    );
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_errors, 1U, "missing motion context increments planner errors");
    expect_u32(
        diagnostics.last_planner_status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "missing context status is recorded"
    );
    expect_u32(diagnostics.last_planner_sequence, 109U, "missing context sequence is recorded");
}

static void test_motion_operation_without_xyz_flag_does_not_call_planner(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 110U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = 0x04U,
                .spindle_pwm = 5000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "non-XYZ operation is not sent to MotionPlanner");
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "planner status is unavailable before the first planner call"
    );
}

static void test_x_hardware_is_initialized_before_first_command(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_read(0U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_init_calls, 1U, "X hardware is initialized once at task start");
    expect_u32(fake_x_init_receive_calls, 0U, "X hardware is initialized before the first queue read");
    expect_u32(fake_x_bench_starts, 0U, "initialization does not start motion");
}

static void test_stream_event_source_is_registered_for_stream_axes_only(void) {
    memset(test_event_source_registrations, 0, sizeof(test_event_source_registrations));
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_read(0U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(test_event_source_registrations[MOTION_AXIS_X], 1U, "X gets an event source");
    expect_u32(test_event_source_registrations[MOTION_AXIS_Y], 0U, "Y has no planned stream yet");
    expect_u32(test_event_source_registrations[MOTION_AXIS_Z], 0U, "Z has no planned stream yet");
    expect_u32(test_event_source_contexts[MOTION_AXIS_X] != NULL, 1U, "the source is bound to an executor instance");

    test_executor_underruns[MOTION_AXIS_X] = 3U;
    test_executor_underruns[MOTION_AXIS_Y] = 4U;
    expect_u32(motion_task_get_stream_underrun_count(), 7U, "underruns are summed over axes");
}

static void run_single_stop(MotionTaskContext_t *context, MotionAxisHardwareStatus_t status_after_init) {
    const CommandDTO_t stop = {
        .sequence = 120U,
        .code = COMMAND_CODE_STOP,
    };
    script_read(0U, osOK, &stop);
    test_block_when_empty = 1U;
    /* MotionTask initializes the hardware itself, so the state is injected on the first queue read. */
    script_x_state(0U, status_after_init, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    (void)run_motion_task_until_scripted_stop(context);
}

static const CommandDTO_t test_home_x_command = {
    .sequence = 130U,
    .code = COMMAND_CODE_HOME,
    .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK,
};

/* Read 0 delivers HOME; every later read is a 1 ms poll timeout preceded by the scripted hardware state. */
static void script_home_polls(uint32_t poll_count) {
    script_read(0U, osOK, &test_home_x_command);
    for (uint32_t index = 1U; index <= poll_count; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
}

static void test_home_fast_seek_moves_toward_start_and_backs_off_on_pf10(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(1U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "HOME starts fast seek and then backoff");
    expect_u32(fake_x_release_stop_arms, 1U, "backoff arms the PF10 release stop");
    expect_i64(fake_x_bench_last_signed_steps, 1000000, "backoff moves away from PF10 (positive with this config)");
    expect_u32(fake_x_stop_requests, 0U, "a PF10 stop is not treated as an abort");
    expect_u32(test_last_receive_timeout, 1U, "active homing polls the queue every millisecond");
}

static void test_home_fast_seek_aborts_on_end_limit(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    script_home_polls(1U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "PF9 stop does not start a backoff");
    expect_i64(fake_x_bench_last_signed_steps, -1000000, "fast seek moves toward PF10 (negative with this config)");
    expect_u32(fake_x_stop_requests, 1U, "PF9 stop aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "aborted homing stops polling");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "aborted homing leaves X unhomed"
    );
}

/* The three hardware states of a successful homing, applied on reads first..first+2:
 * fast seek stopped by PF10, backoff stopped by the PF10 release, approach stopped by PF10. */
static void script_successful_home_states(uint32_t first) {
    script_x_state(first, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_x_state(
        first + 1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
        0U
    );
    script_x_state(first + 2U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
}

/* Drives HOME through the fast seek and the backoff, then applies the given approach result. */
static void run_home_to_approach_result(
    MotionTaskContext_t *context,
    MotionAxisHardwareStopReason_t approach_reason,
    uint8_t start_limit_active
) {
    reset_receive_script();
    *context = valid_motion_context();
    context->machine_state.position_um[0] = 12345;
    script_successful_home_states(1U);
    script_x_state(3U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, approach_reason, start_limit_active);
    script_home_polls(3U);
    (void)run_motion_task_until_scripted_stop(context);
}

static const CommandDTO_t test_calibrate_x_command = {
    .sequence = 140U,
    .code = COMMAND_CODE_CALIBRATE,
    .parameters.calibrate.axes = MOTION_PLANNER_AXIS_X_MASK,
};

/* CALIBRATE, homing on reads 1..3, the measurement finishes on read 4, then `polls` more reads. */
static void run_calibration_with_final_home_polls(MotionTaskContext_t *context, uint32_t polls) {
    script_successful_home_states(1U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 4U + polls; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_calibration_runs_after_homing_and_applies_result(void) {
    MotionTaskContext_t context;
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 10U, .distance_um = 12U, .travel_um = 301000U};
    fake_encoder_x_um = -300000;
    context = valid_motion_context();
    context.machine_state.position_um[0] = 777;
    script_successful_home_states(5U);
    run_calibration_with_final_home_polls(&context, 3U);

    expect_u32(fake_calibration_begin_calls, 1U, "calibration starts once homing has finished");
    expect_u32(fake_calibration_poll_calls, 1U, "calibration is polled while it runs");
    expect_u32(fake_x_bench_starts, 6U, "calibration homes X before and after the measurement");
    expect_i64(fake_x_bench_last_signed_steps, -1000000, "the last move is the approach of the final homing");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "X is marked calibrated");
    expect_u32(context.calibration.axis[0].distance_um, 12U, "measured distance is kept in RAM");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 10U, "planner uses the measured steps");
    expect_u32(context.planner_config.axis[0].steps_per_um_denominator, 12U, "planner uses the measured distance");
    expect_u32(context.planner_config.axis[0].max_speed_um_per_s, 60000U, "speed limit follows from the step rate");
    expect_i64(context.machine_state.position_um[0], 0, "calibration ends with X at home");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished calibration stops polling");

    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, NULL);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_OK, "diagnostics report the calibration result");
    expect_u32(axis.calibrated, 1U, "diagnostics report X as calibrated");
    expect_u32(axis.calibration_steps, 10U, "diagnostics report the measured steps");
    expect_u32(axis.calibration_distance_um, 12U, "diagnostics report the measured distance");
    expect_u32(axis.travel_um, 301000U, "diagnostics report the travel");
    expect_u32((uint32_t)axis.max_position_um, 301000U, "diagnostics report the usable travel");
    expect_u32(axis.max_speed_um_per_s, 60000U, "diagnostics report the speed limit");
    expect_u32(axis.homed, 1U, "diagnostics report X as homed");
    expect_u32((uint32_t)axis.planned_position_um, 0U, "diagnostics report the planned position");
    expect_u32((uint32_t)axis.scale_position_um, (uint32_t)-300000, "diagnostics read the scale live");

    /* While the final homing runs the calibration is still busy and other X commands wait. */
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 10U, .distance_um = 12U, .travel_um = 301000U};
    context = valid_motion_context();
    script_x_state(5U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_calibration_with_final_home_polls(&context, 1U);
    expect_u32(test_last_receive_timeout, 1U, "calibration keeps polling during the final homing");
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(
        axis.calibration_result,
        AXIS_CALIBRATION_RUN_RESULT_RUNNING,
        "calibration is not done before X is home"
    );

    /* The final homing fails: the measurement is kept, X is left unhomed. */
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 10U, .distance_um = 12U, .travel_um = 301000U};
    context = valid_motion_context();
    script_x_state(5U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    run_calibration_with_final_home_polls(&context, 1U);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED, "failed final homing is reported");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "measurement survives it");
    expect_u32(axis.homed, 0U, "X is unhomed after the failed final homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed final homing ends the calibration");

    /* STOP during the final homing aborts the calibration as well. */
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    context = valid_motion_context();
    script_successful_home_states(1U);
    script_x_state(5U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 4U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    const CommandDTO_t stop = {.sequence = 143U, .code = COMMAND_CODE_STOP};
    script_read(5U, osOK, &stop);
    /* Delay 1 is the settle pause of the first homing; the axis is at rest on the STOP wait that follows. */
    test_axis_settles_after_delays = 2U;
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_ABORTED, "STOP aborts the final homing");
    expect_u32(axis.homed, 0U, "aborted final homing leaves X unhomed");
}

static void test_calibration_failures(void) {
    MotionTaskContext_t context;

    reset_receive_script();
    context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osErrorTimeout, NULL);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_calibration_begin_calls, 0U, "calibration does not start when homing fails");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED, "failed homing is reported");

    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    expect_u32(fake_calibration_begin_calls, 0U, "HOME alone does not calibrate");

    reset_receive_script();
    fake_calibration_begin_result = AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED;
    context = valid_motion_context();
    script_successful_home_states(1U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 3U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED, "calibration that cannot start fails");
    expect_u32(context.calibration.calibrated_axes_mask, 0U, "failed calibration marks nothing");

    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE;
    context = valid_motion_context();
    script_successful_home_states(1U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 4U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE, "measurement failure is reported");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "failed calibration keeps the limits");
}

/* Both switches active at once can only be a wiring or sensor fault, so nothing may move. */
static void test_both_limits_active_refuse_homing_and_calibration(void) {
    MotionTaskContext_t context;
    MotionAxisDiagnostics axis = {0};

    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    fake_x_end_limit_active = 1U;
    script_read(0U, osOK, &test_calibrate_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_LIMITS_CONFLICT, "CALIBRATE reports the conflict");
    expect_u32(fake_x_bench_starts, 0U, "CALIBRATE with both switches active does not move");
    expect_u32(test_last_receive_timeout, osWaitForever, "refused calibration does not poll");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    fake_x_end_limit_active = 1U;
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "HOME with both switches active does not move");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "refused HOME leaves X unhomed"
    );

    /* Starting on PF10 alone is a normal start: the fast seek is blocked at once and the backoff follows. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "HOME on PF10 alone starts the seek");
}

static void test_calibration_is_cancelled_and_guarded(void) {
    MotionTaskContext_t context;
    const CommandDTO_t stop = {.sequence = 141U, .code = COMMAND_CODE_STOP};

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osOK, &test_calibrate_x_command);
    script_read(2U, osOK, &test_home_x_command);
    script_read(3U, osOK, &stop);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "CALIBRATE and HOME are ignored while calibration homes");
    expect_u32(fake_calibration_abort_calls, 1U, "STOP aborts the calibration");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_ABORTED, "aborted calibration is reported");

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osErrorParameter, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_calibration_abort_calls, 1U, "queue failure aborts the calibration");

    reset_receive_script();
    context = valid_motion_context();
    const CommandDTO_t calibrate_unknown = {
        .sequence = 142U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_ALL_AXES_MASK | 0x08U,
    };
    script_read(0U, osOK, &calibrate_unknown);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "a mask with an unknown axis is refused as a whole");
}

static void test_home_completes_only_on_pf10_during_approach(void) {
    MotionTaskContext_t context;
    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    expect_u32(fake_x_bench_starts, 3U, "fast seek, backoff and approach are started");
    expect_i64(fake_x_bench_last_signed_steps, -1000000, "the approach heads back toward PF10");
    expect_u32(test_delay_calls, 1U, "the approach waits once after the backoff");
    expect_u32(test_last_delay_ms, 10U, "the belt gets 10 ms to come to rest");
    expect_u32(
        fake_x_start_limit_hook == encoder_ox_reset_coordinate ? 1U : 0U,
        1U,
        "the approach lets the PF10 interrupt zero the scale"
    );
    expect_u32(fake_x_stop_requests, 0U, "a successful homing requests no stop");
    expect_u32(fake_x_ready_after_home_calls, 1U, "successful home readies the X hardware");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "successful homing marks X homed");
    expect_i64(context.machine_state.position_um[0], 0, "successful homing zeroes X");

    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 1U);
    expect_u32(fake_x_stop_requests, 1U, "approach ended by PF9 aborts homing");
    expect_u32(fake_x_start_limit_hook == NULL ? 1U : 0U, 1U, "aborted homing disarms the scale reset");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "approach ended by PF9 leaves X unhomed"
    );
    expect_i64(context.machine_state.position_um[0], 12345, "aborted homing keeps the old X position");
}

static void test_home_backoff_aborts_when_stopped_by_end_limit(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_x_state(2U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    script_home_polls(2U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "backoff stopped by PF9 does not start the approach");
    expect_u32(fake_x_stop_requests, 1U, "backoff stopped by PF9 aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "aborted homing stops polling");
}

static void test_home_backoff_continues_while_pf10_flickers(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    /* The release interrupt fired, but PF10 is active again once the axis has stopped. */
    script_x_state(2U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE, 1U);
    script_home_polls(2U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 3U, "a flickering PF10 restarts the backoff");
    expect_u32(fake_x_release_stop_arms, 2U, "the restarted backoff arms the release stop again");
    expect_i64(fake_x_bench_last_signed_steps, 1000000, "the restarted backoff keeps moving away from PF10");
    expect_u32(test_delay_calls, 0U, "no approach is started while PF10 is active");
    expect_u32(test_last_receive_timeout, 1U, "homing stays active");
}

static void test_stop_clears_runtime_fault_and_x_homing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);

    expect_u32(fake_x_clear_fault_calls, 1U, "STOP acknowledges an X runtime fault");
    expect_u32(fake_x_status, MOTION_AXIS_HARDWARE_STATUS_READY, "cleared fault returns X hardware to READY");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "X position is no longer trusted after a fault"
    );
}

static void test_stop_request_and_pipeline_reset(void) {
    reset_receive_script();
    motion_task_request_stop();
    expect_u32(test_generator_halts, 1U, "STOP halts the step generator at once");
    expect_u32(fake_x_stop_requests, 1U, "STOP stops the pulses at once");
    expect_u32(test_queue_discards, 1U, "STOP drops the commands accepted before it");

    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[0] = 5000;
    fake_encoder_x_um = -1234;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_u32(test_generator_resumes, 1U, "STOP restarts the generator once the axis is at rest");
    expect_i64(context.machine_state.position_um[0], 1234, "after STOP X continues from the scale reading");

    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    context.machine_state.position_um[0] = 5000;
    fake_encoder_x_um = 777;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_i64(context.machine_state.position_um[0], 5000, "unhomed X keeps its position after STOP");

    reset_receive_script();
    context = valid_motion_context();
    fake_encoder_x_um = 42;
    test_axis_settles_after_delays = 3U;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_RUNNING);
    expect_u32(test_delay_calls, 3U, "STOP waits for the axis to come to rest");
    expect_u32(test_generator_resumes, 1U, "generator restarts after the axis settled");
    expect_i64(context.machine_state.position_um[0], 42, "positive scale reading is used as is");

    reset_receive_script();
    context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_RUNNING);
    expect_u32(test_delay_calls, MOTION_TASK_STOP_SETTLE_TIMEOUT_MS, "STOP gives up after the settle timeout");
    expect_u32(test_generator_resumes, 0U, "generator stays halted while the axis still moves");
}

static void test_stop_without_fault_keeps_homing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);

    expect_u32(fake_x_stop_requests, 1U, "STOP requests an X hardware stop");
    expect_u32(fake_x_clear_fault_calls, 0U, "STOP without a fault does not clear anything");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "homing survives a plain STOP");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(fake_axes[axis].timing_peak_resets, 1U, "STOP starts a new step-timing measurement on every axis");
    }

    reset_receive_script();
    context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_STOPPED);
    expect_u32(fake_x_clear_fault_calls, 0U, "an axis stopped by a switch is not a fault");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "homing survives a switch stop");
}

typedef enum { TEST_HOME_PHASE_FAST_SEEK = 0, TEST_HOME_PHASE_BACKOFF, TEST_HOME_PHASE_APPROACH } TestHomePhase_t;

typedef struct {
    const char *name;
    TestHomePhase_t phase;
    TestXHardwareState_t state;
    uint8_t expect_active;
    uint32_t expect_starts;
} TestHomePhaseCase_t;

/* Scripts the hardware states that walk HOME into `phase` (poll N moves it into phase N). */
static void script_home_prefix(TestHomePhase_t phase) {
    if (phase >= TEST_HOME_PHASE_BACKOFF) {
        script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    }
    if (phase >= TEST_HOME_PHASE_APPROACH) {
        script_x_state(
            2U,
            MOTION_AXIS_HARDWARE_STATUS_STOPPED,
            MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
            0U
        );
    }
}

#define TEST_X_STATE(status_, reason_, limit_)                                                                         \
    {.apply = 1U,                                                                                                      \
        .status = MOTION_AXIS_HARDWARE_STATUS_##status_,                                                               \
        .stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_##reason_,                                                     \
        .start_limit_active = (limit_)}

static const TestHomePhaseCase_t test_home_phase_cases[] = {
    {"fast seek fault aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(FAULT_TIMING, NONE, 0U), 0U, 1U},
    {"fast seek completing without PF10 aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(COMPLETE, NONE, 0U), 0U, 1U},
    {"fast seek with idle hardware aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(READY, NONE, 0U), 0U, 1U},
    {"fast seek stopped on request aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(STOPPED, REQUEST, 0U), 0U, 1U},
    {"fast seek keeps waiting while STEP is pending",
        TEST_HOME_PHASE_FAST_SEEK,
        TEST_X_STATE(WAITING, NONE, 0U),
        1U,
        1U},

    {"backoff fault aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(FAULT_TIMING, NONE, 1U), 0U, 2U},
    {"backoff completing without the PF10 release aborts",
        TEST_HOME_PHASE_BACKOFF,
        TEST_X_STATE(COMPLETE, NONE, 1U),
        0U,
        2U},
    {"backoff with idle hardware aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(READY, NONE, 0U), 0U, 2U},
    {"backoff stopped on request aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(STOPPED, REQUEST, 0U), 0U, 2U},
    {"backoff keeps running while PF10 is active", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(RUNNING, NONE, 1U), 1U, 2U},
    {"backoff stopped by the PF10 release starts the approach",
        TEST_HOME_PHASE_BACKOFF,
        TEST_X_STATE(STOPPED, START_LIMIT_RELEASE, 0U),
        1U,
        3U},

    {"approach PF10 stop with released switch aborts",
        TEST_HOME_PHASE_APPROACH,
        TEST_X_STATE(STOPPED, START_LIMIT, 0U),
        0U,
        3U},
    {"approach keeps running", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(RUNNING, NONE, 0U), 1U, 3U},
    {"approach fault aborts", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(FAULT_TIMING, NONE, 0U), 0U, 3U},
    {"approach completing without PF10 aborts", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(COMPLETE, NONE, 0U), 0U, 3U},
    {"approach with idle hardware aborts", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(READY, NONE, 0U), 0U, 3U},
};

static void test_home_phase_outcomes(void) {
    for (size_t index = 0U; index < sizeof(test_home_phase_cases) / sizeof(test_home_phase_cases[0]); ++index) {
        const TestHomePhaseCase_t *test_case = &test_home_phase_cases[index];
        reset_receive_script();
        MotionTaskContext_t context = valid_motion_context();
        const uint32_t final_poll = (uint32_t)test_case->phase + 1U;
        script_home_prefix(test_case->phase);
        test_x_state_script[final_poll] = test_case->state;
        script_home_polls(final_poll);

        (void)run_motion_task_until_scripted_stop(&context);
        expect_u32(fake_x_bench_starts, test_case->expect_starts, test_case->name);
        expect_u32(test_last_receive_timeout, test_case->expect_active != 0U ? 1U : osWaitForever, test_case->name);
        expect_u32(
            context.machine_state.homed_axes_mask,
            MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
            "unfinished homing leaves X unhomed"
        );
    }
}

static void test_home_start_failures(void) {
    MotionTaskContext_t context;

    /* Already on PF10: the seek reports STOPPED immediately, which still counts as a started seek. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    fake_x_failed_start_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    /* The hardware refuses to move into a pressed PF10 and records it as the stop reason. */
    script_x_state(0U, MOTION_AXIS_HARDWARE_STATUS_READY, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(0U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "seek blocked by a pressed PF10 continues with the backoff");
    expect_u32(test_last_receive_timeout, 1U, "homing stays active after a blocked seek");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    fake_x_failed_start_status = MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING;
    script_home_polls(0U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_stop_requests, 1U, "failed seek start aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed seek start ends homing");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 2U;
    fake_x_failed_start_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(2U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "backoff start is attempted");
    expect_u32(fake_x_stop_requests, 1U, "a backoff blocked at its start aborts homing on the next poll");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed backoff start ends homing");
}

static const CommandDTO_t test_x_jog_command = {
    .sequence = 140U,
    .code = COMMAND_CODE_X_JOG,
    .parameters.x_jog = {.delta_um = 2000, .speed_um_per_s = 1000U},
};

static const CommandDTO_t test_xyz_move_command = {
    .sequence = 141U,
    .code = COMMAND_CODE_MOTION_OPERATION,
    .parameters.motion_operation =
        {
            .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
            .x = 1000,
            .speed = 1000U,
        },
};

static const CommandDTO_t test_stop_command = {
    .sequence = 142U,
    .code = COMMAND_CODE_STOP,
};

/* HOME, then `command` arrives while the fast seek is running. */
static void run_command_during_home(MotionTaskContext_t *context, const CommandDTO_t *command) {
    reset_receive_script();
    *context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, command);
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_commands_during_home(void) {
    MotionTaskContext_t context;

    run_command_during_home(&context, &test_home_x_command);
    expect_u32(fake_x_bench_starts, 1U, "repeated HOME does not restart an active homing");
    expect_u32(test_last_receive_timeout, 1U, "repeated HOME keeps homing active");

    run_command_during_home(&context, &test_x_jog_command);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "X_JOG is ignored while homing");
    expect_u32(test_last_receive_timeout, 1U, "X_JOG does not interrupt homing");

    run_command_during_home(&context, &test_xyz_move_command);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "XYZ move is not planned while homing");

    run_command_during_home(&context, &test_stop_command);
    expect_u32(fake_x_stop_requests, 1U, "STOP aborts active homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "STOP ends homing");

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_stop_requests, 1U, "queue failure aborts active homing");
}

static void test_home_is_refused_when_it_cannot_start(void) {
    MotionTaskContext_t context;

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_x_state(0U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "HOME is ignored while the axis is moving");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "ignored HOME keeps X homed");

    reset_receive_script();
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(NULL);
    expect_u32(fake_x_bench_starts, 0U, "HOME without a task context is ignored");
}

static void test_home_and_calibrate_wait_for_the_planned_stream(void) {
    MotionTaskContext_t context;

    reset_receive_script();
    context = valid_motion_context();
    test_generator_busy = 1U;
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "HOME is ignored while the planned stream is busy");

    reset_receive_script();
    script_read(0U, osOK, &test_calibrate_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "CALIBRATE is ignored while the planned stream is busy");

    test_generator_busy = 0U;
}

static void test_step_timing_comes_from_the_axis_config(void) {
    uint32_t pulse_ns = 1U;
    uint32_t low_ns = 2U;
    motion_task_get_step_timing(MOTION_AXIS_X, &pulse_ns, &low_ns);
    expect_u32(pulse_ns, 5000U, "X STEP pulse width");
    expect_u32(low_ns, MOTION_AXIS_HARDWARE_DM556_MIN_STEP_LOW_NS, "X STEP low time");

    pulse_ns = 1U;
    low_ns = 2U;
    motion_task_get_step_timing(MOTION_AXIS_HARDWARE_COUNT, &pulse_ns, &low_ns);
    motion_task_get_step_timing(MOTION_AXIS_X, NULL, &low_ns);
    motion_task_get_step_timing(MOTION_AXIS_X, &pulse_ns, NULL);
    expect_u32(pulse_ns, 1U, "unknown axis leaves the pulse width");
    expect_u32(low_ns, 2U, "unknown axis leaves the low time");
}

static void run_x_jog(MotionTaskContext_t *context, int32_t delta_um) {
    CommandDTO_t jog = test_x_jog_command;
    jog.parameters.x_jog.delta_um = delta_um;
    script_read(0U, osOK, &jog);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_x_jog_is_a_relative_planned_move(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[0] = 1000;
    context.machine_state.position_um[1] = 500;
    run_x_jog(&context, 2000);
    MotionPlannerResult_t result = motion_task_test_get_last_planner_result();
    expect_u32(result.status, MOTION_PLANNER_STATUS_OK, "X_JOG is planned like any other move");
    expect_u32(result.block.sequence, 140U, "X_JOG keeps its sequence");
    expect_i64(result.block.step_delta[0], 160, "X_JOG moves X by its distance");
    expect_i64(result.block.step_delta[1], 0, "X_JOG leaves Y in place");
    expect_u32(result.block.feedrate_um_per_s, 1000U, "X_JOG uses its speed");
    expect_i64(context.machine_state.position_um[0], 3000, "the next move starts at the jog target");
    expect_i64(context.machine_state.position_um[1], 500, "Y position is unchanged");
    expect_u32(test_block_ready_notifications, 1U, "step generator is woken for the jog");
    expect_u32(fake_x_bench_starts, 0U, "X_JOG no longer drives the axis directly");

    reset_receive_script();
    context = valid_motion_context();
    run_x_jog(&context, -200000);
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
        "X_JOG beyond the travel is rejected"
    );
    expect_i64(context.machine_state.position_um[0], 0, "rejected X_JOG keeps the position");
    expect_u32(test_block_ready_notifications, 0U, "rejected X_JOG moves nothing");

    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    run_x_jog(&context, 2000);
    expect_u32(motion_task_get_last_planner_status(), MOTION_PLANNER_STATUS_NOT_HOMED, "X_JOG needs a homed X");

    /* Targets beyond int32 are clamped instead of wrapping around. */
    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.position_um[0] = INT32_MAX;
    run_x_jog(&context, 1);
    expect_i64(motion_task_test_get_last_planner_result().block.step_delta[0], 0, "positive overflow is clamped");
    expect_i64(context.machine_state.position_um[0], INT32_MAX, "clamped jog leaves X at the int32 limit");

    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.position_um[0] = INT32_MIN;
    run_x_jog(&context, -1);
    expect_i64(context.machine_state.position_um[0], INT32_MIN, "negative overflow is clamped");

    reset_receive_script();
    run_x_jog(NULL, 2000);
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "X_JOG without a task context is rejected by the planner"
    );
}

static void test_stop_fault_acknowledgement_edges(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG);
    expect_u32(fake_x_clear_fault_calls, 1U, "STOP tries to clear a configuration fault");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK,
        "uncleared configuration fault leaves homing untouched"
    );

    reset_receive_script();
    run_single_stop(NULL, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
    expect_u32(fake_x_clear_fault_calls, 1U, "STOP clears a runtime fault without a task context");
    expect_u32(fake_x_status, MOTION_AXIS_HARDWARE_STATUS_READY, "fault is cleared without a task context");
}

static void test_stored_calibration_sets_planner_limits(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const MotionPlannerAxisConfig_t default_y = context.planner_config.axis[1];
    test_calibration_load_status = CALIBRATION_STORE_STATUS_OK;
    test_stored_calibration.calibrated_axes_mask = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK;
    test_stored_calibration.axis[0] =
        (AxisCalibration_t){.steps = 100000U, .distance_um = 50000U, .travel_um = 301000U};
    /* Z is marked but its travel does not fit the planner, so it keeps the default limits. */
    test_stored_calibration.axis[2] =
        (AxisCalibration_t){.steps = 1U, .distance_um = 1U, .travel_um = (uint32_t)INT32_MAX + 1U};
    const MotionPlannerAxisConfig_t default_z = context.planner_config.axis[2];
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 100000U, "X steps come from the calibration");
    expect_u32(
        context.planner_config.axis[0].steps_per_um_denominator,
        50000U,
        "X micrometres come from the calibration"
    );
    expect_u32(
        (uint32_t)context.planner_config.axis[0].max_position_um,
        301000U,
        "X travel comes from the calibration"
    );
    expect_u32(context.planner_config.axis[0].max_speed_um_per_s, 25000U, "X speed follows from the step-rate limit");
    expect_u32(
        context.planner_config.axis[1].steps_per_um_numerator,
        default_y.steps_per_um_numerator,
        "uncalibrated Y keeps its defaults"
    );
    expect_u32(
        context.planner_config.axis[2].steps_per_um_numerator,
        default_z.steps_per_um_numerator,
        "unusable Z calibration is ignored"
    );

    reset_receive_script();
    context = valid_motion_context();
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "missing calibration keeps defaults");
}

static const CommandDTO_t test_home_y_command = {
    .sequence = 150U,
    .code = COMMAND_CODE_HOME,
    .parameters.home.axes = MOTION_PLANNER_AXIS_Y_MASK,
};

/* The three hardware states of a successful homing of `axis`, applied on reads first..first+2. */
static void script_axis_home_success(MotionAxis_t axis, uint32_t first) {
    script_axis_state(
        axis,
        first,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
        1U
    );
    script_axis_state(
        axis,
        first + 1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
        0U
    );
    script_axis_state(
        axis,
        first + 2U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
        1U
    );
}

/* `command` on read 0, then `polls` poll timeouts. */
static void run_command_with_polls(MotionTaskContext_t *context, const CommandDTO_t *command, uint32_t polls) {
    script_read(0U, osOK, command);
    for (uint32_t index = 1U; index <= polls; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_y_home_runs_on_its_own_hardware(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[1] = 4321;
    const MotionPlanBlock_t planned_x_move = {.sequence = 93U};
    expect_u32(plan_buffer_try_push(&planned_x_move), PLAN_BUFFER_STATUS_OK, "an X move is queued before HOME Y");
    script_axis_home_success(MOTION_AXIS_Y, 1U);
    run_command_with_polls(&context, &test_home_y_command, 3U);

    expect_u32(fake_axes[MOTION_AXIS_Y].init_calls, 1U, "Y hardware is initialized at task start");
    expect_u32(fake_y.bench_starts, 3U, "Y runs fast seek, backoff and approach");
    expect_u32(fake_x_bench_starts, 0U, "HOME Y does not move X");
    expect_u32(fake_y.release_stop_arms, 1U, "the Y backoff arms the Y release stop");
    expect_i64(fake_y.bench_last_signed_steps, -1000000, "the Y approach heads toward the Y start switch");
    expect_u32(
        fake_y.start_limit_hook == encoder_oy_reset_coordinate ? 1U : 0U,
        1U,
        "the Y approach zeroes the Y scale"
    );
    expect_u32(fake_y.ready_after_home_calls, 1U, "successful homing readies the Y hardware");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "Y is homed");
    expect_i64(context.machine_state.position_um[1], 0, "Y is at zero");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 1U, "Y homing keeps the planned X moves");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished homing stops polling");

    /* A failed Y homing leaves only Y unhomed. */
    reset_receive_script();
    context = valid_motion_context();
    script_axis_state(
        MOTION_AXIS_Y,
        1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
        0U
    );
    run_command_with_polls(&context, &test_home_y_command, 1U);
    expect_u32(fake_y.stop_requests, 1U, "the end switch aborts the Y homing");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_Y_MASK,
        "aborted Y homing leaves only Y unhomed"
    );
}

static void test_x_and_y_home_together(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    const CommandDTO_t home_xy = {
        .sequence = 151U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
    };
    script_axis_home_success(MOTION_AXIS_X, 1U);
    script_axis_home_success(MOTION_AXIS_Y, 1U);
    run_command_with_polls(&context, &home_xy, 3U);

    expect_u32(fake_x_bench_starts, 3U, "X homes");
    expect_u32(fake_y.bench_starts, 3U, "Y homes at the same time");
    expect_u32(fake_y.bench_last_target_rate_hz, 100U, "the belt approach tops out at 100 Hz");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
        "both axes are homed"
    );
    expect_u32(test_last_receive_timeout, osWaitForever, "polling ends once both are home");
}

static void test_home_and_calibrate_masks(void) {
    MotionTaskContext_t context;
    const CommandDTO_t home_none = {.sequence = 152U, .code = COMMAND_CODE_HOME, .parameters.home.axes = 0U};
    const CommandDTO_t calibrate_none = {
        .sequence = 153U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = 0U,
    };
    const CommandDTO_t home_xy = {
        .sequence = 154U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
    };

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &home_none);
    script_read(1U, osOK, &calibrate_none);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts + fake_y.bench_starts, 0U, "HOME and CALIBRATE without axes move nothing");

    /* An idle axis can home while another one is busy... */
    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &test_home_y_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "X starts homing");
    expect_u32(fake_y.bench_starts, 1U, "Y starts homing while X homes");

    /* ...but a command naming a busy axis is refused as a whole. */
    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &home_xy);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "X is not restarted");
    expect_u32(fake_y.bench_starts, 0U, "Y does not start with a refused command");
}

static void test_y_calibration_reports_on_y(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 50000U, .distance_um = 23300U, .travel_um = 401000U};
    fake_y.encoder_um = 1234;
    script_axis_home_success(MOTION_AXIS_Y, 1U);
    script_axis_home_success(MOTION_AXIS_Y, 5U);
    const CommandDTO_t calibrate_y = {
        .sequence = 155U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_AXIS_Y_MASK,
    };
    run_command_with_polls(&context, &calibrate_y, 7U);

    expect_u32(fake_calibration_last_axis, MOTION_AXIS_Y, "the procedure runs on Y");
    expect_u32(fake_x_bench_starts, 0U, "CALIBRATE Y does not move X");
    expect_u32(fake_y.bench_starts, 6U, "Y is homed before and after the measurement");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_Y_MASK, "Y is marked calibrated");
    expect_u32(context.calibration.axis[1].distance_um, 23300U, "the Y measurement is kept for Y");
    expect_u32(context.planner_config.axis[1].steps_per_um_numerator, 50000U, "Y limits follow the measurement");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "X limits are untouched");

    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_OK, "Y diagnostics report the result");
    expect_u32(axis.calibrated, 1U, "Y diagnostics report Y as calibrated");
    expect_u32(axis.travel_um, 401000U, "Y diagnostics report the Y travel");
    expect_u32(axis.homed, 1U, "Y diagnostics report Y as homed");
    expect_u32((uint32_t)axis.scale_position_um, 1234U, "Y diagnostics read the Y scale");
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibrated, 0U, "X stays uncalibrated");
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NONE, "X has no calibration result");
}

static void test_z_home_and_calibration(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    const CommandDTO_t home_all = {
        .sequence = 156U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_ALL_AXES_MASK,
    };
    script_axis_home_success(MOTION_AXIS_X, 1U);
    script_axis_home_success(MOTION_AXIS_Y, 1U);
    script_axis_home_success(MOTION_AXIS_Z, 1U);
    run_command_with_polls(&context, &home_all, 3U);
    expect_u32(fake_z.bench_starts, 3U, "Z homes together with X and Y");
    expect_u32(
        fake_z.start_limit_hook == encoder_oz_reset_coordinate ? 1U : 0U,
        1U,
        "the Z approach zeroes the Z scale"
    );
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "all three axes are homed");

    reset_receive_script();
    context = valid_motion_context();
    context.planner_config.max_step_event_rate_hz = 150000U;
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 50000U, .distance_um = 25000U, .travel_um = 50300U};
    fake_z.encoder_um = -42;
    script_axis_home_success(MOTION_AXIS_Z, 1U);
    script_axis_home_success(MOTION_AXIS_Z, 5U);
    const CommandDTO_t calibrate_z = {
        .sequence = 157U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_AXIS_Z_MASK,
    };
    run_command_with_polls(&context, &calibrate_z, 7U);
    expect_u32(fake_calibration_last_axis, MOTION_AXIS_Z, "the procedure runs on Z");
    expect_u32(fake_calibration_last_settings->measure_steps, 233600U, "Z measures 12 mm of its screw");
    expect_u32(fake_calibration_last_settings->seek.target_step_rate_hz, 80000U, "Z seeks its end at 80 kHz");
    expect_u32(fake_calibration_last_settings->seek.acceleration_steps_per_s2, 320000U, "Z ramps up its seek");
    expect_u32(fake_z.bench_last_target_rate_hz, 4800U, "Z approaches its home switch slowly");
    expect_u32(fake_z.bench_last_initial_rate_hz, 800U, "Z starts its approach below the target rate");
    expect_u32(fake_z.bench_last_acceleration, 32000U, "Z ramps its approach");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_Z_MASK, "Z is marked calibrated");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Z, &axis);
    expect_u32(axis.travel_um, 50300U, "Z diagnostics report the Z travel");
    expect_u32(axis.max_speed_um_per_s, 40000U, "Z speed follows from its own 80 kHz motor limit");
    expect_u32((uint32_t)axis.scale_position_um, (uint32_t)-42, "Z diagnostics read the Z scale");
}

static void test_axis_diagnostics_edges(void) {
    MotionAxisDiagnostics axis = {.travel_um = 77U};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, NULL);
    motion_task_get_axis_diagnostics(MOTION_AXIS_HARDWARE_COUNT, &axis);
    expect_u32(axis.travel_um, 77U, "an unknown axis leaves the result untouched");
}

static void test_planned_moves_of_other_axes_are_refused(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    CommandDTO_t move = test_xyz_move_command;
    move.parameters.motion_operation.y = 1000;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_NOT_HOMED,
        "a homed Y is still refused until it has its own step stream"
    );
    expect_u32(test_block_ready_notifications, 0U, "nothing is queued");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "the refusal unhomes nothing");
}

static void test_stop_reaches_every_axis(void) {
    reset_receive_script();
    motion_task_request_stop();
    expect_u32(fake_y.stop_requests, 1U, "STOP stops the Y pulses at once");

    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_axis_state(
        MOTION_AXIS_Y,
        0U,
        MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING,
        MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
        0U
    );
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_u32(fake_y.clear_fault_calls, 1U, "STOP acknowledges a Y fault");
    expect_u32(fake_x_clear_fault_calls, 0U, "X without a fault is left alone");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_Y_MASK,
        "only Y loses its homing"
    );

    reset_receive_script();
    context = valid_motion_context();
    fake_y.encoder_um = -2500;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_i64(context.machine_state.position_um[1], 2500, "after STOP Y continues from its scale reading");
}

int main(void) {
    test_stored_calibration_sets_planner_limits();
    test_calibration_runs_after_homing_and_applies_result();
    test_calibration_failures();
    test_calibration_is_cancelled_and_guarded();
    test_x_hardware_is_initialized_before_first_command();
    test_stream_event_source_is_registered_for_stream_axes_only();
    test_home_phase_outcomes();
    test_home_start_failures();
    test_commands_during_home();
    test_home_is_refused_when_it_cannot_start();
    test_home_and_calibrate_wait_for_the_planned_stream();
    test_step_timing_comes_from_the_axis_config();
    test_x_jog_is_a_relative_planned_move();
    test_stop_fault_acknowledgement_edges();
    test_stop_clears_runtime_fault_and_x_homing();
    test_stop_without_fault_keeps_homing();
    test_stop_request_and_pipeline_reset();
    test_home_fast_seek_moves_toward_start_and_backs_off_on_pf10();
    test_home_fast_seek_aborts_on_end_limit();
    test_home_completes_only_on_pf10_during_approach();
    test_home_backoff_aborts_when_stopped_by_end_limit();
    test_home_backoff_continues_while_pf10_flickers();
    test_both_limits_active_refuse_homing_and_calibration();
    test_successful_receive_marks_dequeue();
    test_fifo_commands_are_dequeued_once();
    test_receive_error_does_not_count_as_dequeue();
    test_diagnostic_counters_saturate();
    test_xyz_command_reports_unconfigured_planner_inputs();
    test_xyz_command_without_position_change_counts_no_motion();
    test_full_plan_buffer_rejects_new_block_without_losing_existing_plans();
    test_xyz_command_with_null_context_reaches_planner_as_invalid_input();
    test_motion_operation_without_xyz_flag_does_not_call_planner();
    test_y_home_runs_on_its_own_hardware();
    test_x_and_y_home_together();
    test_home_and_calibrate_masks();
    test_y_calibration_reports_on_y();
    test_axis_diagnostics_edges();
    test_z_home_and_calibration();
    test_planned_moves_of_other_axes_are_refused();
    test_stop_reaches_every_axis();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Motion task unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Motion task unit tests passed.\n");
    return 0;
}

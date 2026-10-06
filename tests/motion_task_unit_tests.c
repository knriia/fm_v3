#include "command/command_queue.h"
#include "motion/plan_buffer.h"
#include "motion/x_axis_bench.h"
#include "motion_hardware_x.h"
#include "motion_task.h"

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
static TestQueueRead_t test_receive_script[8];

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
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lld, got %lld)\n",
            message,
            (long long)expected,
            (long long)actual
        );
    }
}

typedef struct {
    uint8_t apply;
    MotionHardwareXStatus_t status;
    MotionHardwareXStopReason_t stop_reason;
    uint8_t start_limit_active;
    uint32_t jog_steps_completed;
    uint8_t start_limit_release_event;
} TestXHardwareState_t;

static MotionHardwareXStatus_t fake_x_status;
static MotionHardwareXStopReason_t fake_x_stop_reason;
static uint8_t fake_x_start_limit_active;
static uint32_t fake_x_jog_steps_completed;
static uint8_t fake_x_start_limit_release_event;
/* Hardware state applied just before the matching scripted queue read, i.e. while MotionTask is blocked. */
static TestXHardwareState_t test_x_state_script[8];

static void script_x_state(
    uint32_t index,
    MotionHardwareXStatus_t status,
    MotionHardwareXStopReason_t stop_reason,
    uint8_t start_limit_active,
    uint32_t jog_steps_completed
) {
    test_x_state_script[index] = (TestXHardwareState_t){
        .apply = 1U,
        .status = status,
        .stop_reason = stop_reason,
        .start_limit_active = start_limit_active,
        .jog_steps_completed = jog_steps_completed,
    };
}

osStatus_t command_queue_receive(CommandDTO_t *command, uint32_t timeout_ms) {
    if (test_receive_calls < 8U && test_x_state_script[test_receive_calls].apply != 0U) {
        const TestXHardwareState_t *state = &test_x_state_script[test_receive_calls];
        fake_x_status = state->status;
        fake_x_stop_reason = state->stop_reason;
        fake_x_start_limit_active = state->start_limit_active;
        fake_x_jog_steps_completed = state->jog_steps_completed;
        fake_x_start_limit_release_event = state->start_limit_release_event;
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

uint32_t osDelay(uint32_t milliseconds) {
    ++test_delay_calls;
    test_last_delay_ms = milliseconds;
    if (test_jump_active != 0U && test_jump_on_delay != 0U) {
        longjmp(test_jump_buffer, 3);
    }
    return 0U;
}

static uint32_t fake_x_init_calls;
static uint32_t fake_x_init_receive_calls;
static uint32_t fake_x_stop_requests;
static uint32_t fake_x_clear_fault_calls;
static uint32_t fake_x_bench_starts;
static uint32_t fake_x_jog_starts;
static int32_t fake_x_bench_last_signed_steps;
static uint32_t fake_x_release_stop_arms;
/* 1-based index of the profile start that fails, leaving the hardware in fake_x_failed_start_status. */
static uint32_t fake_x_failing_start;
static MotionHardwareXStatus_t fake_x_failed_start_status;

MotionHardwareXStatus_t motion_hardware_x_init(const MotionHardwareXConfig_t *config) {
    ++fake_x_init_calls;
    fake_x_init_receive_calls = test_receive_calls;
    fake_x_status = config != NULL ? MOTION_HARDWARE_X_STATUS_READY : MOTION_HARDWARE_X_STATUS_FAULT_CONFIG;
    return fake_x_status;
}

MotionHardwareXStatus_t motion_hardware_x_get_status(void) { return fake_x_status; }

MotionHardwareXStopReason_t motion_hardware_x_get_last_stop_reason(void) { return fake_x_stop_reason; }

void motion_hardware_x_request_stop(void) { ++fake_x_stop_requests; }

MotionHardwareXStatus_t motion_hardware_x_clear_fault(void) {
    ++fake_x_clear_fault_calls;
    if (fake_x_status >= MOTION_HARDWARE_X_STATUS_FAULT_LIMIT &&
        fake_x_status <= MOTION_HARDWARE_X_STATUS_FAULT_TIMING) {
        fake_x_status = MOTION_HARDWARE_X_STATUS_READY;
    }
    return fake_x_status;
}

uint8_t motion_hardware_x_start_limit_is_active(void) { return fake_x_start_limit_active; }

uint8_t motion_hardware_x_take_start_limit_release_event(void) {
    const uint8_t event = fake_x_start_limit_release_event;
    fake_x_start_limit_release_event = 0U;
    return event;
}

uint32_t motion_hardware_x_get_last_jog_steps_completed(void) { return fake_x_jog_steps_completed; }

void motion_hardware_x_arm_start_limit_release_stop(void) { ++fake_x_release_stop_arms; }

void motion_hardware_x_disarm_start_limit_release_stop(void) {}

void encoder_ox_reset_coordinate(void) {}

XAxisBenchStatus_t x_axis_bench_start(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t step_rate_hz,
    MotionHardwareXStatus_t *hardware_status
) {
    (void)hardware_config;
    (void)step_rate_hz;
    ++fake_x_jog_starts;
    fake_x_bench_last_signed_steps = signed_steps;
    *hardware_status = fake_x_status;
    return X_AXIS_BENCH_STATUS_HARDWARE_ERROR;
}

XAxisBenchStatus_t x_axis_bench_start_profile(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionHardwareXStatus_t *hardware_status
) {
    (void)hardware_config;
    (void)initial_step_rate_hz;
    (void)target_step_rate_hz;
    (void)acceleration_steps_per_s2;
    ++fake_x_bench_starts;
    fake_x_bench_last_signed_steps = signed_steps;
    if (fake_x_bench_starts == fake_x_failing_start) {
        fake_x_status = fake_x_failed_start_status;
        *hardware_status = fake_x_status;
        return X_AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }
    if (fake_x_status != MOTION_HARDWARE_X_STATUS_READY && fake_x_status != MOTION_HARDWARE_X_STATUS_STOPPED &&
        fake_x_status != MOTION_HARDWARE_X_STATUS_COMPLETE) {
        *hardware_status = fake_x_status;
        return X_AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }
    fake_x_status = MOTION_HARDWARE_X_STATUS_STARTING;
    fake_x_stop_reason = MOTION_HARDWARE_X_STOP_REASON_NONE;
    *hardware_status = fake_x_status;
    return X_AXIS_BENCH_STATUS_STARTED;
}

static void reset_receive_script(void) {
    fake_x_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
    fake_x_stop_reason = MOTION_HARDWARE_X_STOP_REASON_NONE;
    fake_x_start_limit_active = 0U;
    fake_x_jog_steps_completed = 0U;
    fake_x_start_limit_release_event = 0U;
    (void)memset(test_x_state_script, 0, sizeof(test_x_state_script));
    fake_x_bench_last_signed_steps = 0;
    fake_x_jog_starts = 0U;
    fake_x_failing_start = 0U;
    fake_x_failed_start_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
    fake_x_release_stop_arms = 0U;
    fake_x_init_calls = 0U;
    fake_x_init_receive_calls = UINT32_MAX;
    fake_x_stop_requests = 0U;
    fake_x_clear_fault_calls = 0U;
    fake_x_bench_starts = 0U;
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
        .machine_state = {
            .position_um = {0, 0, 0},
            .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
        },
        .planner_config = {
            .max_step_event_rate_hz = 50000U,
        },
    };
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        context.planner_config.axis[axis] = (MotionPlannerAxisConfig_t){
            .min_position_um = -100000,
            .max_position_um = 100000,
            .steps_per_mm_numerator = 80U,
            .steps_per_mm_denominator = 1U,
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
        .parameters.motion_operation = {
            .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
            .x = 1234,
            .y = -567,
            .z = 890,
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
    expect_u32(motion_task_get_last_planner_status(), MOTION_PLANNER_STATUS_OK, "last planner status is retained for callers");
    expect_u32(planner_result.block.sequence, 101U, "planner receives the command sequence");
    expect_i64(planner_result.block.step_delta[0], 99, "planner receives the X target");
    expect_i64(planner_result.block.step_delta[1], -45, "planner receives the signed Y target");
    expect_i64(planner_result.block.step_delta[2], 71, "planner receives the Z target");
    expect_u32(planner_result.block.feedrate_um_per_s, 12000U, "planner receives the XYZ feedrate");

    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    expect_u32(
        plan_buffer_get_diagnostics(&plan_buffer_diagnostics),
        PLAN_BUFFER_STATUS_OK,
        "plan buffer diagnostics are available"
    );
    expect_u32(plan_buffer_diagnostics.queued, 1U, "successful plan is queued");
    expect_u32(plan_buffer_diagnostics.enqueued_total, 1U, "successful plan enqueue is counted");
    MotionPlanBlock_t buffered_block = {0};
    expect_u32(plan_buffer_try_pop(&buffered_block), PLAN_BUFFER_STATUS_OK, "plan is available to the next pipeline block");
    expect_u32(buffered_block.sequence, 101U, "plan buffer preserves source command sequence");
    expect_i64(buffered_block.step_delta[0], 99, "plan buffer preserves planner X output");
    expect_i64(buffered_block.step_delta[1], -45, "plan buffer preserves planner Y output");
    expect_i64(buffered_block.step_delta[2], 71, "plan buffer preserves planner Z output");
}

static void test_fifo_commands_are_dequeued_once(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 90U};
    expect_u32(plan_buffer_try_push(&pending_plan), PLAN_BUFFER_STATUS_OK, "pending plan exists before STOP");
    const CommandDTO_t commands[] = {
        {.sequence = 102U, .code = COMMAND_CODE_HOME, .parameters.home = {.axes = 5U}},
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
    expect_u32((uint32_t)run_motion_task_until_scripted_stop(&context), 2U, "consumer blocks again after draining scripted commands");
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
        .parameters.motion_operation = {
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
        .parameters.motion_operation = {
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
    expect_u32(diagnostics.last_planner_status, MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "invalid configuration status is recorded");
    expect_u32(diagnostics.last_planner_sequence, 108U, "failed planner sequence is recorded");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 0U, "planner error clears pending plans");
    expect_u32(plan_buffer_diagnostics.cancelled_blocks, 1U, "planner error counts cancelled plans");
}

static void test_xyz_command_without_position_change_counts_no_motion(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 92U};
    expect_u32(plan_buffer_try_push(&pending_plan), PLAN_BUFFER_STATUS_OK, "pending plan exists before no-motion command");
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 111U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation = {
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
        .parameters.motion_operation = {
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
    expect_u32(plan_buffer_diagnostics.enqueued_total, PLAN_BUFFER_CAPACITY, "rejected plan is not counted as enqueued");
    expect_u32(plan_buffer_diagnostics.full_rejections, 1U, "full plan buffer rejection is visible");

    MotionPlanBlock_t first_block = {0};
    expect_u32(plan_buffer_try_pop(&first_block), PLAN_BUFFER_STATUS_OK, "existing plans remain consumable after full rejection");
    expect_u32(first_block.sequence, 1U, "full rejection does not replace earlier plans");
}

static void test_xyz_command_with_null_context_reaches_planner_as_invalid_input(void) {
    reset_receive_script();
    const CommandDTO_t command = {
        .sequence = 109U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation = {
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
    expect_u32(motion_task_test_get_planner_call_count(), 1U, "MotionPlanner is called even when task context is absent");
    expect_u32(
        motion_task_test_get_last_planner_result().status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "missing motion context is rejected by MotionPlanner"
    );
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_errors, 1U, "missing motion context increments planner errors");
    expect_u32(diagnostics.last_planner_status, MOTION_PLANNER_STATUS_INVALID_ARGUMENT, "missing context status is recorded");
    expect_u32(diagnostics.last_planner_sequence, 109U, "missing context sequence is recorded");
}

static void test_motion_operation_without_xyz_flag_does_not_call_planner(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 110U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation = {
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

static void run_single_stop(MotionTaskContext_t *context, MotionHardwareXStatus_t status_after_init) {
    const CommandDTO_t stop = {
        .sequence = 120U,
        .code = COMMAND_CODE_STOP,
    };
    script_read(0U, osOK, &stop);
    test_block_when_empty = 1U;
    /* MotionTask initializes the hardware itself, so the state is injected on the first queue read. */
    script_x_state(0U, status_after_init, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 0U);
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
    script_x_state(1U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U, 0U);
    script_home_polls(1U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "HOME starts fast seek and then backoff");
    expect_u32(fake_x_release_stop_arms, 1U, "backoff arms the PF10 release stop");
    expect_i64(fake_x_bench_last_signed_steps, 1000, "backoff moves away from PF10 (positive with this config)");
    expect_u32(fake_x_stop_requests, 0U, "a PF10 stop is not treated as an abort");
    expect_u32(test_last_receive_timeout, 1U, "active homing polls the queue every millisecond");
}

static void test_home_fast_seek_aborts_on_end_limit(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_END_LIMIT, 0U, 0U);
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

/* Drives HOME through every phase up to the fine seek, then applies the given fine-seek result. */
static void run_home_to_fine_seek_result(
    MotionTaskContext_t *context,
    MotionHardwareXStopReason_t fine_seek_reason,
    uint8_t start_limit_active
) {
    reset_receive_script();
    *context = valid_motion_context();
    context->machine_state.position_um[0] = 12345;
    /* Fast seek hits PF10. */
    script_x_state(1U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U, 0U);
    /* Backoff is running and PF10 is released: the task requests the stop itself. */
    script_x_state(2U, MOTION_HARDWARE_X_STATUS_RUNNING, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 0U);
    script_x_state(3U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_REQUEST, 0U, 300U);
    /* Extra backoff completes exactly. */
    script_x_state(4U, MOTION_HARDWARE_X_STATUS_COMPLETE, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 100U);
    /* Fine seek result. */
    script_x_state(5U, MOTION_HARDWARE_X_STATUS_STOPPED, fine_seek_reason, start_limit_active, 0U);
    script_home_polls(5U);
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_home_completes_only_on_pf10_during_fine_seek(void) {
    MotionTaskContext_t context;
    run_home_to_fine_seek_result(&context, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U);
    expect_u32(fake_x_bench_starts, 4U, "fast seek, backoff, extra backoff and fine seek are started");
    expect_i64(fake_x_bench_last_signed_steps, -400, "fine seek returns toward PF10 by backoff plus extra steps");
    expect_u32(fake_x_stop_requests, 1U, "only the backoff stop is requested");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "successful homing marks X homed");
    expect_i64(context.machine_state.position_um[0], 0, "successful homing zeroes X");

    run_home_to_fine_seek_result(&context, MOTION_HARDWARE_X_STOP_REASON_END_LIMIT, 1U);
    expect_u32(fake_x_stop_requests, 2U, "fine seek ended by PF9 aborts homing");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "fine seek ended by PF9 leaves X unhomed"
    );
    expect_i64(context.machine_state.position_um[0], 12345, "aborted homing keeps the old X position");
}

static void test_home_backoff_aborts_when_stopped_by_end_limit(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U, 0U);
    script_x_state(2U, MOTION_HARDWARE_X_STATUS_RUNNING, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 0U);
    script_x_state(3U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_END_LIMIT, 0U, 300U);
    script_home_polls(3U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "backoff stopped by PF9 does not start the extra backoff");
    expect_u32(fake_x_stop_requests, 2U, "backoff stop request and abort are both issued");
    expect_u32(test_last_receive_timeout, osWaitForever, "aborted homing stops polling");
}

static void test_stop_clears_runtime_fault_and_x_homing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_HARDWARE_X_STATUS_FAULT_TIMING);

    expect_u32(fake_x_clear_fault_calls, 1U, "STOP acknowledges an X runtime fault");
    expect_u32(fake_x_status, MOTION_HARDWARE_X_STATUS_READY, "cleared fault returns X hardware to READY");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "X position is no longer trusted after a fault"
    );
}

static void test_stop_without_fault_keeps_homing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_HARDWARE_X_STATUS_READY);

    expect_u32(fake_x_stop_requests, 1U, "STOP requests an X hardware stop");
    expect_u32(fake_x_clear_fault_calls, 0U, "STOP without a fault does not clear anything");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "homing survives a plain STOP");
}

typedef enum {
    TEST_HOME_PHASE_FAST_SEEK = 0,
    TEST_HOME_PHASE_BACKOFF,
    TEST_HOME_PHASE_BACKOFF_STOPPING,
    TEST_HOME_PHASE_BACKOFF_EXTRA,
    TEST_HOME_PHASE_FINE_SEEK
} TestHomePhase_t;

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
        script_x_state(1U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U, 0U);
    }
    if (phase >= TEST_HOME_PHASE_BACKOFF_STOPPING) {
        script_x_state(2U, MOTION_HARDWARE_X_STATUS_RUNNING, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 0U);
    }
    if (phase >= TEST_HOME_PHASE_BACKOFF_EXTRA) {
        script_x_state(3U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_REQUEST, 0U, 300U);
    }
    if (phase >= TEST_HOME_PHASE_FINE_SEEK) {
        script_x_state(4U, MOTION_HARDWARE_X_STATUS_COMPLETE, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 100U);
    }
}

#define TEST_X_STATE(status_, reason_, limit_, steps_, release_)                                                       \
    {.apply = 1U,                                                                                                      \
     .status = MOTION_HARDWARE_X_STATUS_##status_,                                                                     \
     .stop_reason = MOTION_HARDWARE_X_STOP_REASON_##reason_,                                                           \
     .start_limit_active = (limit_),                                                                                   \
     .jog_steps_completed = (steps_),                                                                                  \
     .start_limit_release_event = (release_)}

static const TestHomePhaseCase_t test_home_phase_cases[] = {
    {"fast seek fault aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(FAULT_TIMING, NONE, 0U, 0U, 0U), 0U, 1U},
    {"fast seek completing without PF10 aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(COMPLETE, NONE, 0U, 0U, 0U), 0U, 1U},
    {"fast seek with idle hardware aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(READY, NONE, 0U, 0U, 0U), 0U, 1U},
    {"fast seek keeps waiting while STEP is pending", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(WAITING, NONE, 0U, 0U, 0U), 1U, 1U},

    {"backoff fault aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(FAULT_TIMING, NONE, 1U, 0U, 0U), 0U, 2U},
    {"backoff stopped by PF10 release starts extra backoff", TEST_HOME_PHASE_BACKOFF,
     TEST_X_STATE(STOPPED, START_LIMIT_RELEASE, 0U, 300U, 1U), 1U, 3U},
    {"backoff completing after PF10 release starts extra backoff", TEST_HOME_PHASE_BACKOFF,
     TEST_X_STATE(COMPLETE, NONE, 0U, 1000U, 1U), 1U, 3U},
    {"backoff release with idle hardware aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(READY, NONE, 0U, 0U, 1U), 0U, 2U},
    {"backoff completing on PF10 aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(COMPLETE, NONE, 1U, 1000U, 0U), 0U, 2U},
    {"backoff stopped on PF10 aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(STOPPED, REQUEST, 1U, 10U, 0U), 0U, 2U},
    {"backoff with idle hardware aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(READY, NONE, 0U, 0U, 0U), 0U, 2U},
    {"backoff keeps running while PF10 is pressed", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(RUNNING, NONE, 1U, 0U, 0U), 1U, 2U},

    {"backoff stopping fault aborts", TEST_HOME_PHASE_BACKOFF_STOPPING, TEST_X_STATE(FAULT_TIMING, NONE, 0U, 0U, 0U), 0U, 2U},
    {"backoff completing before the stop starts extra backoff", TEST_HOME_PHASE_BACKOFF_STOPPING,
     TEST_X_STATE(COMPLETE, NONE, 0U, 1000U, 0U), 1U, 3U},
    {"backoff stopped by PF10 release starts extra backoff", TEST_HOME_PHASE_BACKOFF_STOPPING,
     TEST_X_STATE(STOPPED, START_LIMIT_RELEASE, 0U, 300U, 0U), 1U, 3U},
    {"backoff stop still in progress keeps waiting", TEST_HOME_PHASE_BACKOFF_STOPPING, TEST_X_STATE(RUNNING, NONE, 0U, 0U, 0U), 1U, 2U},
    {"backoff stopping with idle hardware aborts", TEST_HOME_PHASE_BACKOFF_STOPPING, TEST_X_STATE(READY, NONE, 0U, 0U, 0U), 0U, 2U},
    {"backoff stopped on a pressed PF10 aborts", TEST_HOME_PHASE_BACKOFF_STOPPING, TEST_X_STATE(STOPPED, REQUEST, 1U, 300U, 0U), 0U, 2U},
    {"backoff without measured steps aborts", TEST_HOME_PHASE_BACKOFF_STOPPING, TEST_X_STATE(STOPPED, REQUEST, 0U, 0U, 0U), 0U, 2U},
    {"backoff longer than its bound aborts", TEST_HOME_PHASE_BACKOFF_STOPPING, TEST_X_STATE(STOPPED, REQUEST, 0U, 1001U, 0U), 0U, 2U},

    {"extra backoff fault aborts", TEST_HOME_PHASE_BACKOFF_EXTRA, TEST_X_STATE(FAULT_TIMING, NONE, 0U, 0U, 0U), 0U, 3U},
    {"extra backoff reaching PF10 aborts", TEST_HOME_PHASE_BACKOFF_EXTRA, TEST_X_STATE(RUNNING, NONE, 1U, 0U, 0U), 0U, 3U},
    {"extra backoff stopped aborts", TEST_HOME_PHASE_BACKOFF_EXTRA, TEST_X_STATE(STOPPED, END_LIMIT, 0U, 50U, 0U), 0U, 3U},
    {"extra backoff with a short move aborts", TEST_HOME_PHASE_BACKOFF_EXTRA, TEST_X_STATE(COMPLETE, NONE, 0U, 50U, 0U), 0U, 3U},
    {"extra backoff keeps running", TEST_HOME_PHASE_BACKOFF_EXTRA, TEST_X_STATE(RUNNING, NONE, 0U, 0U, 0U), 1U, 3U},
    {"extra backoff with idle hardware aborts", TEST_HOME_PHASE_BACKOFF_EXTRA, TEST_X_STATE(READY, NONE, 0U, 0U, 0U), 0U, 3U},

    {"fine seek PF10 stop with released switch aborts", TEST_HOME_PHASE_FINE_SEEK, TEST_X_STATE(STOPPED, START_LIMIT, 0U, 0U, 0U), 0U, 4U},
    {"fine seek keeps running", TEST_HOME_PHASE_FINE_SEEK, TEST_X_STATE(RUNNING, NONE, 0U, 0U, 0U), 1U, 4U},
    {"fine seek fault aborts", TEST_HOME_PHASE_FINE_SEEK, TEST_X_STATE(FAULT_TIMING, NONE, 0U, 0U, 0U), 0U, 4U},
    {"fine seek completing without PF10 aborts", TEST_HOME_PHASE_FINE_SEEK, TEST_X_STATE(COMPLETE, NONE, 0U, 400U, 0U), 0U, 4U},
    {"fine seek with idle hardware aborts", TEST_HOME_PHASE_FINE_SEEK, TEST_X_STATE(READY, NONE, 0U, 0U, 0U), 0U, 4U},
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
        expect_u32(
            test_last_receive_timeout,
            test_case->expect_active != 0U ? 1U : osWaitForever,
            test_case->name
        );
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
    fake_x_failed_start_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    /* The hardware refuses to move into a pressed PF10 and records it as the stop reason. */
    script_x_state(0U, MOTION_HARDWARE_X_STATUS_READY, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U, 0U);
    script_home_polls(0U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "seek blocked by a pressed PF10 continues with the backoff");
    expect_u32(test_last_receive_timeout, 1U, "homing stays active after a blocked seek");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    fake_x_failed_start_status = MOTION_HARDWARE_X_STATUS_FAULT_TIMING;
    script_home_polls(0U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_stop_requests, 1U, "failed seek start aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed seek start ends homing");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 2U;
    fake_x_failed_start_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    script_x_state(1U, MOTION_HARDWARE_X_STATUS_STOPPED, MOTION_HARDWARE_X_STOP_REASON_START_LIMIT, 1U, 0U);
    script_home_polls(1U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "backoff start is attempted");
    expect_u32(fake_x_stop_requests, 1U, "failed backoff start aborts homing even when the axis reports STOPPED");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed backoff start ends homing");
}

static const CommandDTO_t test_x_jog_command = {
    .sequence = 140U,
    .code = COMMAND_CODE_X_JOG,
    .parameters.x_jog = {.signed_steps = -200, .step_rate_hz = 1000U},
};

static const CommandDTO_t test_xyz_move_command = {
    .sequence = 141U,
    .code = COMMAND_CODE_MOTION_OPERATION,
    .parameters.motion_operation = {
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
    script_x_state(1U, MOTION_HARDWARE_X_STATUS_RUNNING, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 0U);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_commands_during_home(void) {
    MotionTaskContext_t context;

    run_command_during_home(&context, &test_home_x_command);
    expect_u32(fake_x_bench_starts, 1U, "repeated HOME does not restart an active homing");
    expect_u32(test_last_receive_timeout, 1U, "repeated HOME keeps homing active");

    run_command_during_home(&context, &test_x_jog_command);
    expect_u32(fake_x_jog_starts, 0U, "X_JOG is ignored while homing");
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
    script_x_state(0U, MOTION_HARDWARE_X_STATUS_RUNNING, MOTION_HARDWARE_X_STOP_REASON_NONE, 0U, 0U);
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

static void test_x_jog_starts_bench_move_when_idle(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_read(0U, osOK, &test_x_jog_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_jog_starts, 1U, "X_JOG starts a bench move");
    expect_i64(fake_x_bench_last_signed_steps, -200, "X_JOG passes its signed step count");
}

static void test_stop_fault_acknowledgement_edges(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_HARDWARE_X_STATUS_FAULT_CONFIG);
    expect_u32(fake_x_clear_fault_calls, 1U, "STOP tries to clear a configuration fault");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK,
        "uncleared configuration fault leaves homing untouched"
    );

    reset_receive_script();
    run_single_stop(NULL, MOTION_HARDWARE_X_STATUS_FAULT_TIMING);
    expect_u32(fake_x_clear_fault_calls, 1U, "STOP clears a runtime fault without a task context");
    expect_u32(fake_x_status, MOTION_HARDWARE_X_STATUS_READY, "fault is cleared without a task context");
}

int main(void) {
    test_x_hardware_is_initialized_before_first_command();
    test_home_phase_outcomes();
    test_home_start_failures();
    test_commands_during_home();
    test_home_is_refused_when_it_cannot_start();
    test_x_jog_starts_bench_move_when_idle();
    test_stop_fault_acknowledgement_edges();
    test_stop_clears_runtime_fault_and_x_homing();
    test_stop_without_fault_keeps_homing();
    test_home_fast_seek_moves_toward_start_and_backs_off_on_pf10();
    test_home_fast_seek_aborts_on_end_limit();
    test_home_completes_only_on_pf10_during_fine_seek();
    test_home_backoff_aborts_when_stopped_by_end_limit();
    test_successful_receive_marks_dequeue();
    test_fifo_commands_are_dequeued_once();
    test_receive_error_does_not_count_as_dequeue();
    test_diagnostic_counters_saturate();
    test_xyz_command_reports_unconfigured_planner_inputs();
    test_xyz_command_without_position_change_counts_no_motion();
    test_full_plan_buffer_rejects_new_block_without_losing_existing_plans();
    test_xyz_command_with_null_context_reaches_planner_as_invalid_input();
    test_motion_operation_without_xyz_flag_does_not_call_planner();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Motion task unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Motion task unit tests passed.\n");
    return 0;
}

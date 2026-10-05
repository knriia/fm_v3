#include "motion_task.h"

#include "command/command_queue.h"
#include "motion/plan_buffer.h"

#include "cmsis_os.h"

#include <stddef.h>

static volatile MotionTaskDiagnostics motion_task_diagnostics = {
    .last_planner_status = MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
};

#ifdef FM_V3_ENABLE_TEST_HOOKS
static uint32_t motion_task_test_planner_call_count;
static MotionPlannerResult_t motion_task_test_last_planner_result;
#endif

static void motion_task_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static int motion_task_is_xyz_move(const CommandDTO_t *command) {
    return command->code == COMMAND_CODE_MOTION_OPERATION &&
           (command->parameters.motion_operation.operation_flags & COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE) != 0U;
}

static MotionPlannerResult_t
motion_task_plan_xyz_move(const CommandDTO_t *command, const MotionTaskContext_t *context) {
    const CommandMotionOperationParameters_t *motion = &command->parameters.motion_operation;
    const MotionPlannerRequest_t request = {
        .sequence = command->sequence,
        .target_um = {motion->x, motion->y, motion->z},
        .feedrate_um_per_s = motion->speed,
    };

    return motion_planner_plan_linear(
        &request,
        context == NULL ? NULL : &context->machine_state,
        context == NULL ? NULL : &context->planner_config
    );
}

void MotionTask(void *argument) {
    MotionTaskContext_t *context = argument;

    for (;;) {
        CommandDTO_t command = {0};
        const osStatus_t receive_status = command_queue_receive(&command, osWaitForever);
        if (receive_status != osOK) {
            motion_task_counter_increment(&motion_task_diagnostics.queue_receive_errors);
            (void)osDelay(MOTION_TASK_ERROR_RETRY_DELAY_MS);
            continue;
        }

        motion_task_counter_increment(&motion_task_diagnostics.commands_dequeued);
        motion_task_diagnostics.last_sequence = command.sequence;
        motion_task_diagnostics.last_command_code = (uint32_t)command.code;

        if (command.code == COMMAND_CODE_STOP) {
            (void)plan_buffer_clear();
            continue;
        }

        if (motion_task_is_xyz_move(&command)) {
            motion_task_counter_increment(&motion_task_diagnostics.planner_calls);
            const MotionPlannerResult_t result = motion_task_plan_xyz_move(&command, context);
            motion_task_diagnostics.last_planner_status = (uint32_t)result.status;
            motion_task_diagnostics.last_planner_sequence = command.sequence;
#ifdef FM_V3_ENABLE_TEST_HOOKS
            ++motion_task_test_planner_call_count;
            motion_task_test_last_planner_result = result;
#endif
            if (result.status == MOTION_PLANNER_STATUS_OK) {
                motion_task_counter_increment(&motion_task_diagnostics.planner_blocks_created);
                const PlanBufferStatus_t enqueue_status = plan_buffer_try_push(&result.block);
                if (enqueue_status != PLAN_BUFFER_STATUS_OK) {
                    /* The rejected block is dropped; PlanBuffer records why it was not accepted. */
                    continue;
                }
            } else if (result.status == MOTION_PLANNER_STATUS_NO_MOVEMENT) {
                motion_task_counter_increment(&motion_task_diagnostics.planner_no_motion);
            } else {
                motion_task_counter_increment(&motion_task_diagnostics.planner_errors);
                (void)plan_buffer_clear();
            }
        }
    }
}

void motion_task_get_diagnostics(MotionTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->commands_dequeued = motion_task_diagnostics.commands_dequeued;
    diagnostics->queue_receive_errors = motion_task_diagnostics.queue_receive_errors;
    diagnostics->last_sequence = motion_task_diagnostics.last_sequence;
    diagnostics->last_command_code = motion_task_diagnostics.last_command_code;
    diagnostics->planner_calls = motion_task_diagnostics.planner_calls;
    diagnostics->planner_blocks_created = motion_task_diagnostics.planner_blocks_created;
    diagnostics->planner_no_motion = motion_task_diagnostics.planner_no_motion;
    diagnostics->planner_errors = motion_task_diagnostics.planner_errors;
    diagnostics->last_planner_status = motion_task_diagnostics.last_planner_status;
    diagnostics->last_planner_sequence = motion_task_diagnostics.last_planner_sequence;
}

MotionPlannerStatus_t motion_task_get_last_planner_status(void) {
    if (motion_task_diagnostics.planner_calls == 0U) {
        return MOTION_PLANNER_STATUS_INVALID_ARGUMENT;
    }
    return (MotionPlannerStatus_t)motion_task_diagnostics.last_planner_status;
}

#ifdef FM_V3_ENABLE_TEST_HOOKS
void motion_task_test_seed_diagnostics(const MotionTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    motion_task_diagnostics = *diagnostics;
}

void motion_task_test_reset_planner(void) {
    motion_task_test_planner_call_count = 0U;
    motion_task_test_last_planner_result = (MotionPlannerResult_t){0};
    motion_task_diagnostics.last_planner_status = MOTION_PLANNER_STATUS_INVALID_ARGUMENT;
    motion_task_diagnostics.last_planner_sequence = 0U;
}

uint32_t motion_task_test_get_planner_call_count(void) { return motion_task_test_planner_call_count; }

MotionPlannerResult_t motion_task_test_get_last_planner_result(void) { return motion_task_test_last_planner_result; }
#endif

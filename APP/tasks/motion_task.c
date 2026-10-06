#include "motion_task.h"

#include "command/command_queue.h"
#include "encoder.h"
#include "motion/plan_buffer.h"
#include "motion/x_axis_bench.h"

#include "cmsis_os.h"

#include <stddef.h>

#define MOTION_TASK_X_HOME_AXIS_MASK MOTION_PLANNER_AXIS_X_MASK
#define MOTION_TASK_X_HOME_POLL_INTERVAL_MS 1U
#define MOTION_TASK_X_HOME_FAST_STEP_RATE_HZ 50000U
#define MOTION_TASK_X_HOME_CALIBRATION_STEP_RATE_HZ 100U
#define MOTION_TASK_X_HOME_CALIBRATION_INITIAL_STEP_RATE_HZ 10U
#define MOTION_TASK_X_HOME_INITIAL_STEP_RATE_HZ 1000U
#define MOTION_TASK_X_HOME_ACCELERATION_STEPS_PER_S2 100000U
/* Safety bound only: the backoff ends on PF10 release, not after this fixed distance. */
#define MOTION_TASK_X_HOME_CALIBRATION_MAX_STEPS 1000U
#define MOTION_TASK_X_HOME_CALIBRATION_EXTRA_STEPS 100U
#define MOTION_TASK_X_HOME_CALIBRATION_ACCELERATION_STEPS_PER_S2 500U
#define MOTION_TASK_X_HOME_CALIBRATION_RETURN_MAX_STEPS                                                                \
    (MOTION_TASK_X_HOME_CALIBRATION_MAX_STEPS + MOTION_TASK_X_HOME_CALIBRATION_EXTRA_STEPS)

static volatile MotionTaskDiagnostics motion_task_diagnostics = {
    .last_planner_status = MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
};

/* Bring-up defaults: switches are active-high with pull-downs; positive X points to PF9. */
static const MotionHardwareXConfig_t motion_task_x_hardware_config = {
    .step_active_high = 1U,
    .enable_active_high = 0U,
    .positive_direction_high = 1U,
    .positive_moves_toward_end = 1U,
    .start_limit_active_high = 1U,
    .end_limit_active_high = 1U,
    .limit_pull = MOTION_HARDWARE_X_LIMIT_PULL_DOWN,
    .step_pulse_width_ns = 5000U,
    .step_low_time_ns = MOTION_HARDWARE_X_DM556_MIN_STEP_LOW_NS,
    .direction_setup_ns = MOTION_HARDWARE_X_DM556_MIN_DIRECTION_SETUP_NS,
    .direction_hold_ns = 0U,
    .enable_setup_ns = MOTION_HARDWARE_X_DM556_MIN_ENABLE_LEAD_NS,
    .enable_hold_ns = 0U,
};

typedef enum {
    MOTION_TASK_X_HOME_IDLE = 0,
    MOTION_TASK_X_HOME_FAST_SEEK,
    MOTION_TASK_X_HOME_BACKOFF,
    MOTION_TASK_X_HOME_BACKOFF_STOPPING,
    MOTION_TASK_X_HOME_BACKOFF_EXTRA,
    MOTION_TASK_X_HOME_FINE_SEEK
} MotionTaskXHomePhase_t;

typedef struct {
    MotionTaskXHomePhase_t phase;
    uint32_t backoff_release_steps;
    uint8_t backoff_release_detected;
} MotionTaskXHomeContext_t;

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

/* Homing seeks the start switch (PF10); the step sign follows the configured motor direction. */
static int32_t motion_task_x_home_toward_start(uint32_t steps) {
    return motion_task_x_hardware_config.positive_moves_toward_end != 0U ? -(int32_t)steps : (int32_t)steps;
}

static int32_t motion_task_x_home_away_from_start(uint32_t steps) { return -motion_task_x_home_toward_start(steps); }

/* Homing state is owned by MotionTask; an active phase implies a non-NULL task context. */
static uint8_t motion_task_x_home_is_active(const MotionTaskXHomeContext_t *homing) {
    return homing->phase != MOTION_TASK_X_HOME_IDLE;
}

static uint8_t motion_task_x_home_hardware_is_active(MotionHardwareXStatus_t status) {
    return status == MOTION_HARDWARE_X_STATUS_STARTING || status == MOTION_HARDWARE_X_STATUS_WAITING ||
           status == MOTION_HARDWARE_X_STATUS_RUNNING;
}

static uint8_t motion_task_x_home_status_is_fault(MotionHardwareXStatus_t status) {
    return status >= MOTION_HARDWARE_X_STATUS_FAULT_LIMIT && status <= MOTION_HARDWARE_X_STATUS_FAULT_CLOCK;
}

static void motion_task_x_home_abort(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    motion_hardware_x_request_stop();
    motion_hardware_x_disarm_start_limit_release_stop();
    context->machine_state.homed_axes_mask &= (uint8_t)~MOTION_TASK_X_HOME_AXIS_MASK;
    *homing = (MotionTaskXHomeContext_t){0};
}

static void motion_task_x_home_complete(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    motion_hardware_x_disarm_start_limit_release_stop();
    encoder_ox_reset_coordinate();
    context->machine_state.position_um[0] = 0;
    context->machine_state.homed_axes_mask |= MOTION_TASK_X_HOME_AXIS_MASK;
    *homing = (MotionTaskXHomeContext_t){0};
}

static uint8_t motion_task_x_home_start_jog(
    MotionTaskXHomeContext_t *homing,
    MotionTaskContext_t *context,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionTaskXHomePhase_t phase,
    uint8_t allow_switch_already_active
) {
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
    const XAxisBenchStatus_t start_status = x_axis_bench_start_profile(
        &motion_task_x_hardware_config,
        signed_steps,
        initial_step_rate_hz,
        target_step_rate_hz,
        acceleration_steps_per_s2,
        &hardware_status
    );

    if (start_status == X_AXIS_BENCH_STATUS_STARTED ||
        (allow_switch_already_active != 0U && hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED)) {
        homing->phase = phase;
        return 1U;
    }

    motion_task_x_home_abort(homing, context);
    return 0U;
}

static void motion_task_x_home_start_backoff(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    motion_hardware_x_arm_start_limit_release_stop();
    (void)motion_task_x_home_start_jog(
        homing,
        context,
        motion_task_x_home_away_from_start(MOTION_TASK_X_HOME_CALIBRATION_MAX_STEPS),
        MOTION_TASK_X_HOME_CALIBRATION_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_CALIBRATION_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_CALIBRATION_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_BACKOFF,
        0U
    );
}

/* The caller has bounded backoff_steps to 1..MOTION_TASK_X_HOME_CALIBRATION_RETURN_MAX_STEPS. */
static void motion_task_x_home_start_fine_seek(
    MotionTaskXHomeContext_t *homing,
    MotionTaskContext_t *context,
    uint32_t backoff_steps
) {
    (void)motion_task_x_home_start_jog(
        homing,
        context,
        motion_task_x_home_toward_start(backoff_steps),
        MOTION_TASK_X_HOME_CALIBRATION_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_CALIBRATION_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_CALIBRATION_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_FINE_SEEK,
        1U
    );
}

/* Called once PF10 release was seen and the axis is STOPPED or COMPLETE. */
static void motion_task_x_home_finish_backoff(
    MotionTaskXHomeContext_t *homing,
    MotionTaskContext_t *context,
    MotionHardwareXStatus_t hardware_status
) {
    /* Backoff may end by the armed PF10-release stop or by this task's own stop request, never by a limit. */
    const MotionHardwareXStopReason_t stop_reason = motion_hardware_x_get_last_stop_reason();
    const uint8_t stop_is_expected = hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE ||
                                     stop_reason == MOTION_HARDWARE_X_STOP_REASON_START_LIMIT_RELEASE ||
                                     stop_reason == MOTION_HARDWARE_X_STOP_REASON_REQUEST;
    if (stop_is_expected == 0U || motion_hardware_x_start_limit_is_active() != 0U) {
        motion_task_x_home_abort(homing, context);
        return;
    }
    homing->backoff_release_steps = motion_hardware_x_get_last_jog_steps_completed();
    if (homing->backoff_release_steps == 0U ||
        homing->backoff_release_steps > MOTION_TASK_X_HOME_CALIBRATION_MAX_STEPS) {
        motion_task_x_home_abort(homing, context);
        return;
    }
    (void)motion_task_x_home_start_jog(
        homing,
        context,
        motion_task_x_home_away_from_start(MOTION_TASK_X_HOME_CALIBRATION_EXTRA_STEPS),
        MOTION_TASK_X_HOME_CALIBRATION_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_CALIBRATION_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_CALIBRATION_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_BACKOFF_EXTRA,
        0U
    );
}

static void motion_task_x_home_begin(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    if (context == NULL || motion_task_x_home_is_active(homing) != 0U) {
        return;
    }
    if (motion_task_x_home_hardware_is_active(motion_hardware_x_get_status()) != 0U) {
        return;
    }

    *homing = (MotionTaskXHomeContext_t){0};
    context->machine_state.homed_axes_mask &= (uint8_t)~MOTION_TASK_X_HOME_AXIS_MASK;
    (void)plan_buffer_clear();
    (void)motion_task_x_home_start_jog(
        homing,
        context,
        motion_task_x_home_toward_start(X_AXIS_BENCH_MAX_STEPS),
        MOTION_TASK_X_HOME_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_FAST_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_FAST_SEEK,
        1U
    );
}

static void motion_task_x_home_poll(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    const MotionHardwareXStatus_t hardware_status = motion_hardware_x_get_status();
    switch (homing->phase) {
    case MOTION_TASK_X_HOME_FAST_SEEK:
        if (hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED) {
            /* Only PF10 may end the seek; PF9 or any other stop means wrong wiring or direction. */
            if (motion_hardware_x_get_last_stop_reason() == MOTION_HARDWARE_X_STOP_REASON_START_LIMIT) {
                motion_task_x_home_start_backoff(homing, context);
            } else {
                motion_task_x_home_abort(homing, context);
            }
            break;
        }
        if (motion_task_x_home_status_is_fault(hardware_status) != 0U ||
            hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE ||
            motion_task_x_home_hardware_is_active(hardware_status) == 0U) {
            motion_task_x_home_abort(homing, context);
        }
        break;

    case MOTION_TASK_X_HOME_BACKOFF:
        if (motion_task_x_home_status_is_fault(hardware_status) != 0U) {
            motion_task_x_home_abort(homing, context);
            return;
        }
        if (motion_hardware_x_take_start_limit_release_event() != 0U) {
            homing->backoff_release_detected = 1U;
        }
        if (motion_hardware_x_start_limit_is_active() == 0U &&
            motion_task_x_home_hardware_is_active(hardware_status) != 0U) {
            homing->backoff_release_detected = 1U;
        }
        if (homing->backoff_release_detected != 0U) {
            if (motion_task_x_home_hardware_is_active(hardware_status) != 0U) {
                motion_hardware_x_request_stop();
                homing->phase = MOTION_TASK_X_HOME_BACKOFF_STOPPING;
                break;
            }
            if (hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED ||
                hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE) {
                motion_task_x_home_finish_backoff(homing, context, hardware_status);
                break;
            }
            motion_task_x_home_abort(homing, context);
            return;
        }
        if (hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE) {
            motion_task_x_home_abort(homing, context);
            break;
        }
        if (hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED) {
            motion_task_x_home_abort(homing, context);
            return;
        }
        if (motion_task_x_home_hardware_is_active(hardware_status) == 0U) {
            motion_task_x_home_abort(homing, context);
        }
        break;

    case MOTION_TASK_X_HOME_BACKOFF_STOPPING:
        if (motion_task_x_home_status_is_fault(hardware_status) != 0U) {
            motion_task_x_home_abort(homing, context);
            return;
        }
        if (hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED ||
            hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE) {
            motion_task_x_home_finish_backoff(homing, context, hardware_status);
            break;
        }
        if (motion_task_x_home_hardware_is_active(hardware_status) == 0U) {
            motion_task_x_home_abort(homing, context);
        }
        break;

    case MOTION_TASK_X_HOME_BACKOFF_EXTRA:
        if (motion_task_x_home_status_is_fault(hardware_status) != 0U ||
            motion_hardware_x_start_limit_is_active() != 0U || hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED) {
            motion_task_x_home_abort(homing, context);
            return;
        }
        if (hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE) {
            /* backoff_release_steps is already bounded by CALIBRATION_MAX_STEPS, so the sum fits RETURN_MAX. */
            const uint32_t extra_steps = motion_hardware_x_get_last_jog_steps_completed();
            if (extra_steps != MOTION_TASK_X_HOME_CALIBRATION_EXTRA_STEPS) {
                motion_task_x_home_abort(homing, context);
                return;
            }
            motion_task_x_home_start_fine_seek(homing, context, homing->backoff_release_steps + extra_steps);
            break;
        }
        if (motion_task_x_home_hardware_is_active(hardware_status) == 0U) {
            motion_task_x_home_abort(homing, context);
        }
        break;

    case MOTION_TASK_X_HOME_FINE_SEEK:
        if (hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED) {
            if (motion_hardware_x_get_last_stop_reason() != MOTION_HARDWARE_X_STOP_REASON_START_LIMIT ||
                motion_hardware_x_start_limit_is_active() == 0U) {
                motion_task_x_home_abort(homing, context);
                return;
            }
            motion_task_x_home_complete(homing, context);
            return;
        }
        if (motion_task_x_home_status_is_fault(hardware_status) != 0U ||
            hardware_status == MOTION_HARDWARE_X_STATUS_COMPLETE ||
            motion_task_x_home_hardware_is_active(hardware_status) == 0U) {
            motion_task_x_home_abort(homing, context);
        }
        break;

    default:
        /* MOTION_TASK_X_HOME_IDLE: nothing to supervise. */
        break;
    }
}

static void motion_task_x_stop(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    (void)plan_buffer_clear();
    if (motion_task_x_home_is_active(homing) != 0U) {
        motion_task_x_home_abort(homing, context);
    } else {
        motion_hardware_x_request_stop();
    }

    /* STOP is also the operator's fault acknowledgement; the X position is untrusted after a fault. */
    if (motion_task_x_home_status_is_fault(motion_hardware_x_get_status()) != 0U &&
        motion_hardware_x_clear_fault() == MOTION_HARDWARE_X_STATUS_READY && context != NULL) {
        context->machine_state.homed_axes_mask &= (uint8_t)~MOTION_TASK_X_HOME_AXIS_MASK;
    }
}

void MotionTask(void *argument) {
    MotionTaskContext_t *context = argument;
    MotionTaskXHomeContext_t x_homing = {0};

    /* Drive STEP/DIR/ENABLE to defined levels before any command can arrive. */
    (void)motion_hardware_x_init(&motion_task_x_hardware_config);

    for (;;) {
        CommandDTO_t command = {0};
        const uint32_t receive_timeout_ms =
            motion_task_x_home_is_active(&x_homing) != 0U ? MOTION_TASK_X_HOME_POLL_INTERVAL_MS : osWaitForever;
        const osStatus_t receive_status = command_queue_receive(&command, receive_timeout_ms);
        if (receive_status != osOK) {
            /* Timeouts only occur while homing polls; the poll is a no-op once homing has ended. */
            if (receive_status == osErrorTimeout) {
                motion_task_x_home_poll(&x_homing, context);
                continue;
            }
            if (motion_task_x_home_is_active(&x_homing) != 0U) {
                motion_task_x_home_abort(&x_homing, context);
            }
            motion_task_counter_increment(&motion_task_diagnostics.queue_receive_errors);
            (void)osDelay(MOTION_TASK_ERROR_RETRY_DELAY_MS);
            continue;
        }

        motion_task_counter_increment(&motion_task_diagnostics.commands_dequeued);
        motion_task_diagnostics.last_sequence = command.sequence;
        motion_task_diagnostics.last_command_code = (uint32_t)command.code;

        if (command.code == COMMAND_CODE_STOP) {
            motion_task_x_stop(&x_homing, context);
            continue;
        }

        if (command.code == COMMAND_CODE_HOME) {
            if (command.parameters.home.axes == MOTION_TASK_X_HOME_AXIS_MASK) {
                motion_task_x_home_begin(&x_homing, context);
            }
        } else if (command.code == COMMAND_CODE_X_JOG) {
            if (motion_task_x_home_is_active(&x_homing) == 0U) {
                MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
                (void)x_axis_bench_start(
                    &motion_task_x_hardware_config,
                    command.parameters.x_jog.signed_steps,
                    command.parameters.x_jog.step_rate_hz,
                    &hardware_status
                );
            }
        } else if (motion_task_is_xyz_move(&command) && motion_task_x_home_is_active(&x_homing) == 0U) {
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

        motion_task_x_home_poll(&x_homing, context);
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

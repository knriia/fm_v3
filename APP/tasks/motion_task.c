#include "motion_task.h"

#include "command/command_queue.h"
#include "encoder.h"
#include "motion/calibration_store.h"
#include "motion/plan_buffer.h"
#include "motion/x_axis_bench.h"
#include "motion/x_axis_calibration.h"
#include "step_generator_task.h"

#include "cmsis_os.h"

#include <stddef.h>

#define MOTION_TASK_X_HOME_AXIS_MASK MOTION_PLANNER_AXIS_X_MASK
#define MOTION_TASK_X_HOME_POLL_INTERVAL_MS 1U
#define MOTION_TASK_X_HOME_FAST_STEP_RATE_HZ 50000U
#define MOTION_TASK_X_HOME_INITIAL_STEP_RATE_HZ 1000U
#define MOTION_TASK_X_HOME_ACCELERATION_STEPS_PER_S2 100000U
/* The fast seek brakes past PF10, so the backoff has up to 1 mm to cover; its speed does not affect the zero. */
#define MOTION_TASK_X_HOME_BACKOFF_STEP_RATE_HZ 2000U
#define MOTION_TASK_X_HOME_BACKOFF_INITIAL_STEP_RATE_HZ 100U
#define MOTION_TASK_X_HOME_BACKOFF_ACCELERATION_STEPS_PER_S2 2000U
/* The approach runs slowly so PF10 stops the axis within a step of its edge. */
#define MOTION_TASK_X_HOME_SLOW_STEP_RATE_HZ 100U
#define MOTION_TASK_X_HOME_SLOW_INITIAL_STEP_RATE_HZ 10U
#define MOTION_TASK_X_HOME_SLOW_ACCELERATION_STEPS_PER_S2 500U
/* Lets the belt come to rest between the backoff and the approach. */
#define MOTION_TASK_X_HOME_SETTLE_MS 10U
/* PF10, not a step count, ends every homing move; a switch that never changes is a separate open task. */
#define MOTION_TASK_X_HOME_MOVE_STEPS X_AXIS_BENCH_MAX_STEPS

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
    /* Brakes a 50 kHz seek within about 2100 steps, i.e. under 1 mm at 1/51200. */
    .limit_deceleration_steps_per_s2 = 600000U,
    .position_reader = encoder_ox_get_coordinate,
};

typedef enum {
    MOTION_TASK_X_HOME_IDLE = 0,
    /* Toward PF10 at full speed; skipped in effect when PF10 is already active. */
    MOTION_TASK_X_HOME_FAST_SEEK,
    /* Away from PF10 until it turns off. */
    MOTION_TASK_X_HOME_BACKOFF,
    /* Back toward PF10 until it turns on; that interrupt stops the axis and zeroes the scale. */
    MOTION_TASK_X_HOME_APPROACH
} MotionTaskXHomePhase_t;

typedef struct {
    MotionTaskXHomePhase_t phase;
} MotionTaskXHomeContext_t;

/* CALIBRATE homes X, runs the calibration procedure and homes X again, so every move starts from home. */
typedef struct {
    XAxisCalibration_t procedure;
    uint8_t waiting_for_home;
    uint8_t returning_home;
    XAxisCalibrationResult_t result;
} MotionTaskXCalibrationContext_t;

/* Written by MotionTask, read by the diagnostic task; each field is a single 32-bit store. */
static volatile MotionAxisDiagnostics motion_task_x_axis_diagnostics;

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

/* Plans one absolute move and hands it to the step generator; a move the planner rejects is dropped. */
static void motion_task_queue_move(MotionTaskContext_t *context, const MotionPlannerRequest_t *request) {
    motion_task_counter_increment(&motion_task_diagnostics.planner_calls);
    const MotionPlannerResult_t result = motion_planner_plan_linear(
        request,
        context == NULL ? NULL : &context->machine_state,
        context == NULL ? NULL : &context->planner_config
    );
    motion_task_diagnostics.last_planner_status = (uint32_t)result.status;
    motion_task_diagnostics.last_planner_sequence = request->sequence;
#ifdef FM_V3_ENABLE_TEST_HOOKS
    ++motion_task_test_planner_call_count;
    motion_task_test_last_planner_result = result;
#endif
    if (result.status == MOTION_PLANNER_STATUS_OK) {
        motion_task_counter_increment(&motion_task_diagnostics.planner_blocks_created);
        if (plan_buffer_try_push(&result.block) != PLAN_BUFFER_STATUS_OK) {
            /* The rejected block is dropped; PlanBuffer records why it was not accepted. */
            return;
        }
        /* The next move starts where this one ends, not where the axis is right now. */
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            context->machine_state.position_um[axis] = request->target_um[axis];
        }
        step_generator_task_notify_block_ready();
    } else if (result.status == MOTION_PLANNER_STATUS_NO_MOVEMENT) {
        motion_task_counter_increment(&motion_task_diagnostics.planner_no_motion);
    } else {
        motion_task_counter_increment(&motion_task_diagnostics.planner_errors);
        (void)plan_buffer_clear();
    }
}

static void motion_task_queue_xyz_move(MotionTaskContext_t *context, const CommandDTO_t *command) {
    const CommandMotionOperationParameters_t *motion = &command->parameters.motion_operation;
    const MotionPlannerRequest_t request = {
        .sequence = command->sequence,
        .target_um = {motion->x, motion->y, motion->z},
        .feedrate_um_per_s = motion->speed,
    };
    motion_task_queue_move(context, &request);
}

/* X_JOG is a move relative to the planned X position; Y and Z stay where they are. */
static void motion_task_queue_x_jog(MotionTaskContext_t *context, const CommandDTO_t *command) {
    MotionPlannerRequest_t request = {
        .sequence = command->sequence,
        .feedrate_um_per_s = command->parameters.x_jog.speed_um_per_s,
    };
    if (context != NULL) {
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            request.target_um[axis] = context->machine_state.position_um[axis];
        }
        /* A target beyond int32 is clamped; it is far outside any axis, so the planner rejects it. */
        const int64_t target_um =
            (int64_t)context->machine_state.position_um[0] + (int64_t)command->parameters.x_jog.delta_um;
        request.target_um[0] = target_um > INT32_MAX   ? INT32_MAX
                               : target_um < INT32_MIN ? INT32_MIN
                                                       : (int32_t)target_um;
    }
    motion_task_queue_move(context, &request);
}

/* Calibrated axes get their planner limits from the measured values; the others keep the context defaults. */
static void motion_task_apply_calibration(MotionTaskContext_t *context, const MachineCalibration_t *calibration) {
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        if ((calibration->calibrated_axes_mask & (uint8_t)(1U << axis)) == 0U) {
            continue;
        }
        MotionPlannerAxisConfig_t axis_config = {0};
        if (axis_calibration_to_planner_axis(
                &calibration->axis[axis],
                context->planner_config.max_step_event_rate_hz,
                &axis_config
            ) == AXIS_CALIBRATION_STATUS_OK) {
            context->planner_config.axis[axis] = axis_config;
        }
    }
}

static void motion_task_load_calibration(MotionTaskContext_t *context) {
    MachineCalibration_t calibration = {0};
    if (context != NULL && calibration_store_load(&calibration) == CALIBRATION_STORE_STATUS_OK) {
        motion_task_apply_calibration(context, &calibration);
    }
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

/* PF10 and PF9 can never be covered at once, so both active means a wiring or sensor fault. */
static uint8_t motion_task_x_limits_conflict(void) {
    return motion_hardware_x_start_limit_is_active() != 0U && motion_hardware_x_end_limit_is_active() != 0U ? 1U : 0U;
}

static void motion_task_x_home_abort(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    motion_hardware_x_request_stop();
    motion_hardware_x_disarm_start_limit_release_stop();
    motion_hardware_x_arm_start_limit_hook(NULL);
    context->machine_state.homed_axes_mask &= (uint8_t)~MOTION_TASK_X_HOME_AXIS_MASK;
    *homing = (MotionTaskXHomeContext_t){0};
}

static void motion_task_x_home_complete(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    /* The PF10 interrupt that stopped the approach has already zeroed the scale at the switch edge. */
    context->machine_state.position_um[0] = 0;
    context->machine_state.homed_axes_mask |= MOTION_TASK_X_HOME_AXIS_MASK;
    (void)motion_hardware_x_ready_after_home();
    *homing = (MotionTaskXHomeContext_t){0};
}

/* A move that a switch blocks at once leaves the hardware STOPPED with that switch as the reason; the next poll
 * judges it like any other stop, e.g. a seek that starts on PF10 goes straight on to the backoff. */
static void motion_task_x_home_start_move(
    MotionTaskXHomeContext_t *homing,
    MotionTaskContext_t *context,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionTaskXHomePhase_t phase
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

    if (start_status == X_AXIS_BENCH_STATUS_STARTED || hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED) {
        homing->phase = phase;
        return;
    }
    motion_task_x_home_abort(homing, context);
}

static void motion_task_x_home_start_backoff(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    /* The PF10 release interrupt stops the backoff. */
    motion_hardware_x_arm_start_limit_release_stop();
    motion_task_x_home_start_move(
        homing,
        context,
        motion_task_x_home_away_from_start(MOTION_TASK_X_HOME_MOVE_STEPS),
        MOTION_TASK_X_HOME_BACKOFF_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_BACKOFF_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_BACKOFF_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_BACKOFF
    );
}

static void motion_task_x_home_start_approach(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    /* The PF10 interrupt that stops the approach zeroes the scale in the same interrupt. */
    motion_hardware_x_arm_start_limit_hook(encoder_ox_reset_coordinate);
    motion_task_x_home_start_move(
        homing,
        context,
        motion_task_x_home_toward_start(MOTION_TASK_X_HOME_MOVE_STEPS),
        MOTION_TASK_X_HOME_SLOW_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_SLOW_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_SLOW_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_APPROACH
    );
}

/* Callers start homing only while X is idle, so no homing or calibration is running here. */
static void motion_task_x_home_begin(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    if (context == NULL) {
        return;
    }
    if (motion_task_x_home_hardware_is_active(motion_hardware_x_get_status()) != 0U) {
        return;
    }

    *homing = (MotionTaskXHomeContext_t){0};
    context->machine_state.homed_axes_mask &= (uint8_t)~MOTION_TASK_X_HOME_AXIS_MASK;
    (void)plan_buffer_clear();
    if (motion_task_x_limits_conflict() != 0U) {
        return;
    }
    motion_task_x_home_start_move(
        homing,
        context,
        motion_task_x_home_toward_start(MOTION_TASK_X_HOME_MOVE_STEPS),
        MOTION_TASK_X_HOME_INITIAL_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_FAST_STEP_RATE_HZ,
        MOTION_TASK_X_HOME_ACCELERATION_STEPS_PER_S2,
        MOTION_TASK_X_HOME_FAST_SEEK
    );
}

static void motion_task_x_home_poll(MotionTaskXHomeContext_t *homing, MotionTaskContext_t *context) {
    if (motion_task_x_home_is_active(homing) == 0U) {
        return;
    }
    const MotionHardwareXStatus_t hardware_status = motion_hardware_x_get_status();
    if (motion_task_x_home_hardware_is_active(hardware_status) != 0U) {
        return;
    }
    /* Every homing move is meant to end on PF10; running out of steps or any other stop is a failure. */
    const MotionHardwareXStopReason_t stop_reason = hardware_status == MOTION_HARDWARE_X_STATUS_STOPPED
                                                        ? motion_hardware_x_get_last_stop_reason()
                                                        : MOTION_HARDWARE_X_STOP_REASON_NONE;

    switch (homing->phase) {
    case MOTION_TASK_X_HOME_FAST_SEEK:
        if (stop_reason == MOTION_HARDWARE_X_STOP_REASON_START_LIMIT) {
            motion_task_x_home_start_backoff(homing, context);
            return;
        }
        break;
    case MOTION_TASK_X_HOME_BACKOFF:
        if (stop_reason == MOTION_HARDWARE_X_STOP_REASON_START_LIMIT_RELEASE) {
            if (motion_hardware_x_start_limit_is_active() != 0U) {
                /* PF10 flickered on its edge; keep backing off until it stays off. */
                motion_task_x_home_start_backoff(homing, context);
                return;
            }
            (void)osDelay(MOTION_TASK_X_HOME_SETTLE_MS);
            motion_task_x_home_start_approach(homing, context);
            return;
        }
        break;
    default:
        /* MOTION_TASK_X_HOME_APPROACH */
        if (stop_reason == MOTION_HARDWARE_X_STOP_REASON_START_LIMIT &&
            motion_hardware_x_start_limit_is_active() != 0U) {
            motion_task_x_home_complete(homing, context);
            return;
        }
        break;
    }
    motion_task_x_home_abort(homing, context);
}

static uint8_t motion_task_x_calibration_is_busy(const MotionTaskXCalibrationContext_t *calibration) {
    return calibration->waiting_for_home != 0U || calibration->returning_home != 0U ||
                   x_axis_calibration_is_active(&calibration->procedure) != 0U
               ? 1U
               : 0U;
}

static void motion_task_x_calibration_begin(
    MotionTaskXCalibrationContext_t *calibration,
    MotionTaskXHomeContext_t *homing,
    MotionTaskContext_t *context
) {
    if (motion_task_x_limits_conflict() != 0U) {
        calibration->result = X_AXIS_CALIBRATION_RESULT_LIMITS_CONFLICT;
        return;
    }
    calibration->result = X_AXIS_CALIBRATION_RESULT_RUNNING;
    calibration->waiting_for_home = 1U;
    /* A homing that cannot start leaves X unhomed, and the next poll reports it. */
    motion_task_x_home_begin(homing, context);
}

static void motion_task_x_calibration_cancel(MotionTaskXCalibrationContext_t *calibration) {
    if (motion_task_x_calibration_is_busy(calibration) != 0U) {
        x_axis_calibration_abort(&calibration->procedure);
        *calibration = (MotionTaskXCalibrationContext_t){.result = X_AXIS_CALIBRATION_RESULT_ABORTED};
    }
}

static void motion_task_x_calibration_poll(
    MotionTaskXCalibrationContext_t *calibration,
    MotionTaskXHomeContext_t *homing,
    MotionTaskContext_t *context
) {
    if (calibration->waiting_for_home != 0U || calibration->returning_home != 0U) {
        if (motion_task_x_home_is_active(homing) != 0U) {
            return;
        }
        const uint8_t returning_home = calibration->returning_home;
        calibration->waiting_for_home = 0U;
        calibration->returning_home = 0U;
        if ((context->machine_state.homed_axes_mask & MOTION_TASK_X_HOME_AXIS_MASK) == 0U) {
            /* After the final homing the measured values are kept; X stays unhomed until HOME or CALIBRATE. */
            calibration->result = X_AXIS_CALIBRATION_RESULT_HOME_FAILED;
            return;
        }
        calibration->result = returning_home != 0U
                                  ? X_AXIS_CALIBRATION_RESULT_OK
                                  : x_axis_calibration_begin(&calibration->procedure, &motion_task_x_hardware_config);
        return;
    }
    if (x_axis_calibration_is_active(&calibration->procedure) == 0U) {
        return;
    }

    AxisCalibration_t measured = {0};
    const XAxisCalibrationResult_t result =
        x_axis_calibration_poll(&calibration->procedure, &motion_task_x_hardware_config, &measured);
    if (result != X_AXIS_CALIBRATION_RESULT_OK) {
        calibration->result = result;
        return;
    }
    context->calibration.axis[0] = measured;
    context->calibration.calibrated_axes_mask |= MOTION_TASK_X_HOME_AXIS_MASK;
    motion_task_apply_calibration(context, &context->calibration);
    /* Calibration reports OK only once X is back home; a homing that cannot start is reported on the next poll. */
    calibration->returning_home = 1U;
    motion_task_x_home_begin(homing, context);
}

static void
motion_task_publish_x_axis(const MotionTaskXCalibrationContext_t *calibration, const MotionTaskContext_t *context) {
    motion_task_x_axis_diagnostics.calibration_phase = (uint32_t)calibration->procedure.phase;
    motion_task_x_axis_diagnostics.calibration_result = (uint32_t)calibration->result;
    if (context == NULL) {
        return;
    }
    const AxisCalibration_t *measured = &context->calibration.axis[0];
    const MotionPlannerAxisConfig_t *limits = &context->planner_config.axis[0];
    motion_task_x_axis_diagnostics.calibrated =
        (context->calibration.calibrated_axes_mask & MOTION_TASK_X_HOME_AXIS_MASK) != 0U ? 1U : 0U;
    motion_task_x_axis_diagnostics.calibration_steps = measured->steps;
    motion_task_x_axis_diagnostics.calibration_distance_um = measured->distance_um;
    motion_task_x_axis_diagnostics.travel_um = measured->travel_um;
    motion_task_x_axis_diagnostics.min_position_um = limits->min_position_um;
    motion_task_x_axis_diagnostics.max_position_um = limits->max_position_um;
    motion_task_x_axis_diagnostics.max_speed_um_per_s = limits->max_speed_um_per_s;
    motion_task_x_axis_diagnostics.homed =
        (context->machine_state.homed_axes_mask & MOTION_TASK_X_HOME_AXIS_MASK) != 0U ? 1U : 0U;
    motion_task_x_axis_diagnostics.planned_position_um = context->machine_state.position_um[0];
}

void motion_task_request_stop(void) {
    step_generator_task_halt();
    motion_hardware_x_request_stop();
    command_queue_discard_all();
}

/* Second half of STOP: once the axis is at rest the step pipeline is emptied and restarted. If it does not come to
 * rest in time the generator stays halted, so nothing moves until the next STOP succeeds. */
static void motion_task_finish_stop(MotionTaskContext_t *context) {
    for (
        uint32_t waited_ms = 0U; motion_task_x_home_hardware_is_active(motion_hardware_x_get_status()) != 0U;
        ++waited_ms
    ) {
        if (waited_ms == MOTION_TASK_STOP_SETTLE_TIMEOUT_MS) {
            return;
        }
        (void)osDelay(1U);
    }
    /* Blocks still queued were cut short, so the next move starts from where the scale says X is. */
    if (context != NULL && (context->machine_state.homed_axes_mask & MOTION_TASK_X_HOME_AXIS_MASK) != 0U) {
        const int32_t scale_um = encoder_ox_get_coordinate();
        context->machine_state.position_um[0] = scale_um < 0 ? -scale_um : scale_um;
    }
    step_generator_task_resume_after_stop();
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
    MotionTaskXCalibrationContext_t x_calibration = {0};

    /* Drive STEP/DIR/ENABLE to defined levels before any command can arrive. */
    (void)motion_hardware_x_init(&motion_task_x_hardware_config);
    motion_task_load_calibration(context);
    motion_task_publish_x_axis(&x_calibration, context);

    for (;;) {
        CommandDTO_t command = {0};
        const uint8_t x_busy =
            motion_task_x_home_is_active(&x_homing) != 0U || motion_task_x_calibration_is_busy(&x_calibration) != 0U
                ? 1U
                : 0U;
        const uint32_t receive_timeout_ms = x_busy != 0U ? MOTION_TASK_X_HOME_POLL_INTERVAL_MS : osWaitForever;
        const osStatus_t receive_status = command_queue_receive(&command, receive_timeout_ms);
        if (receive_status != osOK) {
            /* Timeouts only occur while homing polls; the poll is a no-op once homing has ended. */
            if (receive_status == osErrorTimeout) {
                motion_task_x_home_poll(&x_homing, context);
                motion_task_x_calibration_poll(&x_calibration, &x_homing, context);
                motion_task_publish_x_axis(&x_calibration, context);
                continue;
            }
            if (motion_task_x_home_is_active(&x_homing) != 0U) {
                motion_task_x_home_abort(&x_homing, context);
            }
            motion_task_x_calibration_cancel(&x_calibration);
            motion_task_publish_x_axis(&x_calibration, context);
            motion_task_counter_increment(&motion_task_diagnostics.queue_receive_errors);
            (void)osDelay(MOTION_TASK_ERROR_RETRY_DELAY_MS);
            continue;
        }

        motion_task_counter_increment(&motion_task_diagnostics.commands_dequeued);
        motion_task_diagnostics.last_sequence = command.sequence;
        motion_task_diagnostics.last_command_code = (uint32_t)command.code;

        if (command.code == COMMAND_CODE_STOP) {
            motion_task_x_calibration_cancel(&x_calibration);
            motion_task_x_stop(&x_homing, context);
            motion_task_finish_stop(context);
            motion_task_publish_x_axis(&x_calibration, context);
            continue;
        }

        if (command.code == COMMAND_CODE_HOME) {
            if (command.parameters.home.axes == MOTION_TASK_X_HOME_AXIS_MASK && x_busy == 0U) {
                motion_task_x_home_begin(&x_homing, context);
            }
        } else if (command.code == COMMAND_CODE_CALIBRATE) {
            if (command.parameters.calibrate.axes == MOTION_TASK_X_HOME_AXIS_MASK && x_busy == 0U) {
                motion_task_x_calibration_begin(&x_calibration, &x_homing, context);
            }
        } else if (command.code == COMMAND_CODE_X_JOG && x_busy == 0U) {
            motion_task_queue_x_jog(context, &command);
        } else if (motion_task_is_xyz_move(&command) && x_busy == 0U) {
            motion_task_queue_xyz_move(context, &command);
        }

        motion_task_x_home_poll(&x_homing, context);
        motion_task_x_calibration_poll(&x_calibration, &x_homing, context);
        motion_task_publish_x_axis(&x_calibration, context);
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

void motion_task_get_x_axis_diagnostics(MotionAxisDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    diagnostics->calibration_phase = motion_task_x_axis_diagnostics.calibration_phase;
    diagnostics->calibration_result = motion_task_x_axis_diagnostics.calibration_result;
    diagnostics->calibrated = motion_task_x_axis_diagnostics.calibrated;
    diagnostics->calibration_steps = motion_task_x_axis_diagnostics.calibration_steps;
    diagnostics->calibration_distance_um = motion_task_x_axis_diagnostics.calibration_distance_um;
    diagnostics->travel_um = motion_task_x_axis_diagnostics.travel_um;
    diagnostics->min_position_um = motion_task_x_axis_diagnostics.min_position_um;
    diagnostics->max_position_um = motion_task_x_axis_diagnostics.max_position_um;
    diagnostics->max_speed_um_per_s = motion_task_x_axis_diagnostics.max_speed_um_per_s;
    diagnostics->homed = motion_task_x_axis_diagnostics.homed;
    diagnostics->planned_position_um = motion_task_x_axis_diagnostics.planned_position_um;
    diagnostics->scale_position_um = encoder_ox_get_coordinate();
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

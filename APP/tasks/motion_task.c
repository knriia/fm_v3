#include "motion_task.h"

#include "command/command_queue.h"
#include "encoder.h"
#include "motion/axis_bench.h"
#include "motion/axis_calibration_run.h"
#include "motion/calibration_store.h"
#include "motion/home_block.h"
#include "motion/plan_buffer.h"
#include "motion/step_executor.h"
#include "step_generator_task.h"

#include "cmsis_os.h"

#include <stddef.h>

/* HOME and CALIBRATE accept any combination of these axes; the axes of one command run at the same time. */
#define MOTION_TASK_HOME_AXES_MASK MOTION_PLANNER_ALL_AXES_MASK
/* Axes the planned step stream can drive. Until each axis has its own stream, a planned move of any other axis is
 * refused as NOT_HOMED; homing and calibration move those axes directly. */
#define MOTION_TASK_STREAM_AXES_MASK MOTION_PLANNER_AXIS_X_MASK
#define MOTION_TASK_HOME_POLL_INTERVAL_MS 1U
/* Lets the belt come to rest between the backoff and the approach. */
#define MOTION_TASK_HOME_SETTLE_MS 10U
/* The start switch, not a step count, ends every homing move; a switch that never changes is a separate open
 * task. */
#define MOTION_TASK_HOME_MOVE_STEPS AXIS_BENCH_MAX_STEPS

static volatile MotionTaskDiagnostics motion_task_diagnostics = {
    .last_planner_status = MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
};

/* The planned step stream of each axis in MOTION_TASK_STREAM_AXES_MASK; the other entries stay unused. */
static StepExecutor_t motion_task_stream_executors[MOTION_AXIS_HARDWARE_COUNT];

/* The three homing moves; the switches, not step counts, end them. */
typedef struct {
    /* Toward the start switch. */
    AxisBenchProfile_t fast_seek;
    /* Away from the start switch until it turns off; its speed does not affect the zero. */
    AxisBenchProfile_t backoff;
    /* Back onto the start switch, slowly, so it stops the axis within a step of its edge. */
    AxisBenchProfile_t approach;
} MotionTaskHomeSettings_t;

typedef struct {
    MotionAxisHardwareConfig_t hardware;
    /* Zeroes the axis scale; runs in the start-switch interrupt that ends the homing approach. */
    MotionAxisHardwareStartLimitHook_t zero_scale;
    MotionTaskHomeSettings_t home;
    AxisCalibrationRunSettings_t calibration;
    /* Highest step rate the motor still follows; with the calibration it sets the axis speed limit. */
    uint32_t max_step_rate_hz;
} MotionTaskAxisConfig_t;

/* X and Y are belt axes at about 0.5 um per step. */
#define MOTION_TASK_BELT_HOME_SETTINGS                                                                                 \
    {                                                                                                                  \
        .fast_seek = {1000U, 50000U, 100000U},                                                                         \
        .backoff = {100U, 2000U, 2000U},                                                                               \
        .approach = {10U, 100U, 500U},                                                                                 \
    }

#define MOTION_TASK_BELT_CALIBRATION_SETTINGS                                                                          \
    {                                                                                                                  \
        .seek = {1000U, 50000U, 100000U},                                                                              \
        .measure = {100U, 2000U, 2000U},                                                                               \
        .take_up_steps = 2000U,                                                                                        \
        .measure_steps = 50000U,                                                                                       \
    }

/* Z drives a screw through a belt: about 335 um per motor turn, travel about 16.3 mm. The driver runs at 6400 pulses
 * per turn, about 0.052 um per step. Fast moves ramp from 8 kHz to 80 kHz (about 4.2 mm/s) within about 10000 steps
 * (0.5 mm). All step counts and rates here assume that driver setting: scale them together with its divider. */
#define MOTION_TASK_Z_FAST_PROFILE {8000U, 80000U, 320000U}

#define MOTION_TASK_Z_HOME_SETTINGS                                                                                    \
    {                                                                                                                  \
        .fast_seek = MOTION_TASK_Z_FAST_PROFILE,                                                                       \
        .backoff = {3200U, 16000U, 80000U},                                                                            \
        .approach = {800U, 4800U, 32000U},                                                                             \
    }

/* The take-up is about 0.08 mm and the measurement about 12.2 mm. */
#define MOTION_TASK_Z_CALIBRATION_SETTINGS                                                                             \
    {                                                                                                                  \
        .seek = MOTION_TASK_Z_FAST_PROFILE,                                                                            \
        .measure = {3200U, 32000U, 80000U},                                                                            \
        .take_up_steps = 1600U,                                                                                        \
        .measure_steps = 233600U,                                                                                      \
    }

/* Bring-up defaults, the same for every axis: switches are active-high with pull-downs; positive motion points to
 * the end switch. Both Y motors share one STEP/DIR/ENA wiring, so firmware sees one Y motor. */
#define MOTION_TASK_AXIS_HARDWARE_CONFIG(position_reader_, limit_deceleration_)                                        \
    {                                                                                                                  \
        .step_active_high = 1U,                                                                                        \
        .enable_active_high = 0U,                                                                                      \
        .positive_direction_high = 1U,                                                                                 \
        .positive_moves_toward_end = 1U,                                                                               \
        .start_limit_active_high = 1U,                                                                                 \
        .end_limit_active_high = 1U,                                                                                   \
        .limit_pull = MOTION_AXIS_HARDWARE_LIMIT_PULL_DOWN,                                                            \
        .step_pulse_width_ns = 5000U,                                                                                  \
        .step_low_time_ns = MOTION_AXIS_HARDWARE_DM556_MIN_STEP_LOW_NS,                                                \
        .direction_setup_ns = MOTION_AXIS_HARDWARE_DM556_MIN_DIRECTION_SETUP_NS,                                       \
        .direction_hold_ns = 0U,                                                                                       \
        .enable_setup_ns = MOTION_AXIS_HARDWARE_DM556_MIN_ENABLE_LEAD_NS,                                              \
        .enable_hold_ns = 0U,                                                                                          \
        .limit_deceleration_steps_per_s2 = (limit_deceleration_),                                                      \
        .position_reader = (position_reader_),                                                                         \
    }

static const MotionTaskAxisConfig_t motion_task_axis_configs[MOTION_AXIS_HARDWARE_COUNT] = {
    [MOTION_AXIS_X] =
        {
            /* Brakes a 50 kHz seek within about 2100 steps, i.e. under 1 mm at 1/51200. */
            .hardware = MOTION_TASK_AXIS_HARDWARE_CONFIG(encoder_ox_get_coordinate, 600000U),
            .zero_scale = encoder_ox_reset_coordinate,
            .home = MOTION_TASK_BELT_HOME_SETTINGS,
            .calibration = MOTION_TASK_BELT_CALIBRATION_SETTINGS,
            .max_step_rate_hz = MOTION_TASK_MAX_STEP_RATE_HZ,
        },
    [MOTION_AXIS_Y] =
        {
            .hardware = MOTION_TASK_AXIS_HARDWARE_CONFIG(encoder_oy_get_coordinate, 600000U),
            .zero_scale = encoder_oy_reset_coordinate,
            .home = MOTION_TASK_BELT_HOME_SETTINGS,
            .calibration = MOTION_TASK_BELT_CALIBRATION_SETTINGS,
            .max_step_rate_hz = MOTION_TASK_MAX_STEP_RATE_HZ,
        },
    [MOTION_AXIS_Z] =
        {
            /* Brakes an 80 kHz seek within about 2000 steps (0.1 mm) past the switch. */
            .hardware = MOTION_TASK_AXIS_HARDWARE_CONFIG(encoder_oz_get_coordinate, 1600000U),
            .zero_scale = encoder_oz_reset_coordinate,
            .home = MOTION_TASK_Z_HOME_SETTINGS,
            .calibration = MOTION_TASK_Z_CALIBRATION_SETTINGS,
            /* About 4.2 mm/s, the fast-move rate. */
            .max_step_rate_hz = 80000U,
        },
};

typedef enum {
    MOTION_TASK_HOME_IDLE = 0,
    /* Toward the start switch at full speed; skipped in effect when the switch is already active. */
    MOTION_TASK_HOME_FAST_SEEK,
    /* Away from the start switch until it turns off. */
    MOTION_TASK_HOME_BACKOFF,
    /* Back toward the start switch until it turns on; that interrupt stops the axis and zeroes the scale. */
    MOTION_TASK_HOME_APPROACH
} MotionTaskHomePhase_t;

/* CALIBRATE homes the axis, runs the calibration procedure and homes it again, so every move starts from home. */
typedef struct {
    AxisCalibrationRun_t procedure;
    uint8_t waiting_for_home;
    uint8_t returning_home;
    AxisCalibrationRunResult_t result;
} MotionTaskCalibrationContext_t;

/* Homing and calibration state of one axis, owned by MotionTask; an active phase implies a non-NULL task
 * context. */
typedef struct {
    MotionAxis_t axis;
    MotionTaskHomePhase_t home_phase;
    MotionTaskCalibrationContext_t calibration;
} MotionTaskAxisState_t;

/* Written by MotionTask, read by the diagnostic task; each field is a single 32-bit store. */
static volatile MotionAxisDiagnostics motion_task_axis_diagnostics[MOTION_AXIS_HARDWARE_COUNT];

#ifdef FM_V3_ENABLE_TEST_HOOKS
static uint32_t motion_task_test_planner_call_count;
static MotionPlannerResult_t motion_task_test_last_planner_result;
#endif

static void motion_task_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static uint8_t motion_task_axis_mask(MotionAxis_t axis) { return (uint8_t)(1U << (uint32_t)axis); }

static const MotionAxisHardwareConfig_t *motion_task_hardware_config(MotionAxis_t axis) {
    return &motion_task_axis_configs[axis].hardware;
}

static int motion_task_is_xyz_move(const CommandDTO_t *command) {
    return command->code == COMMAND_CODE_MOTION_OPERATION &&
           (command->parameters.motion_operation.operation_flags & COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE) != 0U;
}

/* Plans one absolute move and hands it to the step generator; a move the planner rejects is dropped. */
static void motion_task_queue_move(MotionTaskContext_t *context, const MotionPlannerRequest_t *request) {
    motion_task_counter_increment(&motion_task_diagnostics.planner_calls);
    MotionPlannerMachineState_t planner_state = {0};
    if (context != NULL) {
        planner_state = context->machine_state;
        planner_state.homed_axes_mask &= MOTION_TASK_STREAM_AXES_MASK;
    }
    const MotionPlannerResult_t result = motion_planner_plan_linear(
        request,
        context == NULL ? NULL : &planner_state,
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

_Static_assert(MOTION_PLANNER_AXIS_COUNT == MOTION_AXIS_HARDWARE_COUNT, "planner axes index the axis configs");

/* Calibrated axes get their planner limits from the measured values; the others keep the context defaults. The speed
 * limit uses the lower of the axis motor rate and the planner step rate. */
static void motion_task_apply_calibration(MotionTaskContext_t *context, const MachineCalibration_t *calibration) {
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        if ((calibration->calibrated_axes_mask & (uint8_t)(1U << axis)) == 0U) {
            continue;
        }
        const uint32_t axis_rate_hz = motion_task_axis_configs[axis].max_step_rate_hz;
        const uint32_t planner_rate_hz = context->planner_config.max_step_event_rate_hz;
        MotionPlannerAxisConfig_t axis_config = {0};
        if (axis_calibration_to_planner_axis(
                &calibration->axis[axis],
                axis_rate_hz < planner_rate_hz ? axis_rate_hz : planner_rate_hz,
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

static uint8_t motion_task_home_is_active(const MotionTaskAxisState_t *state) {
    return state->home_phase != MOTION_TASK_HOME_IDLE;
}

static uint8_t motion_task_hardware_is_active(MotionAxisHardwareStatus_t status) {
    return status == MOTION_AXIS_HARDWARE_STATUS_STARTING || status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
           status == MOTION_AXIS_HARDWARE_STATUS_RUNNING;
}

static uint8_t motion_task_hardware_status_is_fault(MotionAxisHardwareStatus_t status) {
    return status >= MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT && status <= MOTION_AXIS_HARDWARE_STATUS_FAULT_CLOCK;
}

/* The two switches of an axis can never be covered at once, so both active means a wiring or sensor fault. */
static uint8_t motion_task_limits_conflict(MotionAxis_t axis) {
    return motion_axis_hardware_start_limit_is_active(axis) != 0U &&
                   motion_axis_hardware_end_limit_is_active(axis) != 0U
               ? 1U
               : 0U;
}

static uint8_t motion_task_axis_is_streamed(MotionAxis_t axis) {
    return (motion_task_axis_mask(axis) & MOTION_TASK_STREAM_AXES_MASK) != 0U ? 1U : 0U;
}

/* Returns the step generator to its idle state: stops the stream, drops the queued blocks and waits until the task
 * has reset. Returns 0 when it does not reset in time; the generator then stays halted until the next STOP. */
static uint8_t motion_task_stream_flush(void) {
    step_generator_task_halt();
    (void)plan_buffer_clear();
    step_generator_task_resume_after_stop();
    for (uint32_t waited_ms = 0U; step_generator_task_is_busy() != 0U; ++waited_ms) {
        if (waited_ms == MOTION_TASK_STOP_SETTLE_TIMEOUT_MS) {
            return 0U;
        }
        (void)osDelay(1U);
    }
    return 1U;
}

/* The stop reason of a stream is only meant for the homing that waits for it; it is not left behind for later moves. */
static void motion_task_home_forget_stream_stop(MotionAxis_t axis) {
    if (motion_task_axis_is_streamed(axis) != 0U) {
        motion_axis_hardware_clear_stop_reason(axis);
    }
}

/* Ends the homing of the axis without touching the step generator; STOP leaves its reset to motion_task_finish_stop,
 * which waits until every axis is at rest. */
static void motion_task_home_cancel(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    motion_axis_hardware_request_stop(state->axis);
    motion_axis_hardware_disarm_start_limit_release_stop(state->axis);
    motion_axis_hardware_arm_start_limit_hook(state->axis, NULL);
    motion_task_home_forget_stream_stop(state->axis);
    context->machine_state.homed_axes_mask &= (uint8_t)~motion_task_axis_mask(state->axis);
    state->home_phase = MOTION_TASK_HOME_IDLE;
}

static void motion_task_home_abort(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    if (motion_task_axis_is_streamed(state->axis) != 0U) {
        /* A move still in the stream would run on without anyone watching its switches. */
        (void)motion_task_stream_flush();
    }
    motion_task_home_cancel(state, context);
}

static void motion_task_home_complete(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    motion_task_home_forget_stream_stop(state->axis);
    /* The start-switch interrupt that stopped the approach has already zeroed the scale at the switch edge. */
    context->machine_state.position_um[state->axis] = 0;
    context->machine_state.homed_axes_mask |= motion_task_axis_mask(state->axis);
    (void)motion_axis_hardware_ready_after_home(state->axis);
    state->home_phase = MOTION_TASK_HOME_IDLE;
}

/* A streamed axis moves with one plan block at the constant target rate of the profile, with no ramp. The stream, idle
 * at this point, plays it; a switch ends it. A switch that blocks the move at once ends it just the same. */
static uint8_t motion_task_stream_start_move(
    MotionAxis_t axis,
    AxisBenchDirection_t direction,
    uint32_t steps,
    const AxisBenchProfile_t *profile
) {
    MotionPlanBlock_t block;
    if (home_block_make(motion_task_hardware_config(axis), direction, steps, profile->target_step_rate_hz, &block) ==
        0U) {
        return 0U;
    }
    /* The stop of the previous move is still recorded; the next poll must wait for the stop of this one. */
    motion_axis_hardware_clear_stop_reason(axis);
    if (plan_buffer_try_push(&block) != PLAN_BUFFER_STATUS_OK) {
        return 0U;
    }
    step_generator_task_notify_block_ready();
    return 1U;
}

/* A move that a switch blocks at once leaves the hardware STOPPED with that switch as the reason; the next poll
 * judges it like any other stop, e.g. a seek that starts on the start switch goes straight on to the backoff. */
static void motion_task_home_start_move(
    MotionTaskAxisState_t *state,
    MotionTaskContext_t *context,
    AxisBenchDirection_t direction,
    const AxisBenchProfile_t *profile,
    MotionTaskHomePhase_t phase
) {
    if (motion_task_axis_is_streamed(state->axis) != 0U) {
        if (motion_task_stream_start_move(state->axis, direction, MOTION_TASK_HOME_MOVE_STEPS, profile) == 0U) {
            motion_task_home_abort(state, context);
            return;
        }
        state->home_phase = phase;
        return;
    }
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    const AxisBenchStatus_t start_status = axis_bench_start_profile(
        state->axis,
        motion_task_hardware_config(state->axis),
        direction,
        MOTION_TASK_HOME_MOVE_STEPS,
        profile->initial_step_rate_hz,
        profile->target_step_rate_hz,
        profile->acceleration_steps_per_s2,
        &hardware_status
    );

    if (start_status == AXIS_BENCH_STATUS_STARTED || hardware_status == MOTION_AXIS_HARDWARE_STATUS_STOPPED) {
        state->home_phase = phase;
        return;
    }
    motion_task_home_abort(state, context);
}

static void motion_task_home_start_backoff(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    /* The start-switch release interrupt stops the backoff. */
    motion_axis_hardware_arm_start_limit_release_stop(state->axis);
    motion_task_home_start_move(
        state,
        context,
        AXIS_BENCH_TOWARD_END,
        &motion_task_axis_configs[state->axis].home.backoff,
        MOTION_TASK_HOME_BACKOFF
    );
}

static void motion_task_home_start_approach(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    /* The backoff is over; a later move toward the end switch must not stop when the start switch turns off. */
    motion_axis_hardware_disarm_start_limit_release_stop(state->axis);
    /* The start-switch interrupt that stops the approach zeroes the scale in the same interrupt. */
    motion_axis_hardware_arm_start_limit_hook(state->axis, motion_task_axis_configs[state->axis].zero_scale);
    motion_task_home_start_move(
        state,
        context,
        AXIS_BENCH_TOWARD_START,
        &motion_task_axis_configs[state->axis].home.approach,
        MOTION_TASK_HOME_APPROACH
    );
}

/* Callers start homing only while the axis is idle, so no homing or calibration of it is running here. */
static void motion_task_home_begin(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    if (context == NULL) {
        return;
    }
    if (motion_task_hardware_is_active(motion_axis_hardware_get_status(state->axis)) != 0U) {
        return;
    }

    const uint8_t axis_mask = motion_task_axis_mask(state->axis);
    state->home_phase = MOTION_TASK_HOME_IDLE;
    context->machine_state.homed_axes_mask &= (uint8_t)~axis_mask;
    if ((axis_mask & MOTION_TASK_STREAM_AXES_MASK) != 0U) {
        /* Queued blocks were planned from the old position of this axis. */
        (void)plan_buffer_clear();
    }
    if (motion_task_limits_conflict(state->axis) != 0U) {
        return;
    }
    motion_task_home_start_move(
        state,
        context,
        AXIS_BENCH_TOWARD_START,
        &motion_task_axis_configs[state->axis].home.fast_seek,
        MOTION_TASK_HOME_FAST_SEEK
    );
}

/* 1 once the homing move of the axis has ended. The reason is NONE when it ended without a switch: it ran out of steps,
 * the generator stopped on a fault, or the stream did not reset. A streamed axis is also made ready for the next move
 * here, because a stop on a switch leaves its generator halted. A stream that does not reset blocks MotionTask for the
 * settle timeout here and for another one in the abort that follows. */
static uint8_t motion_task_move_has_ended(MotionAxis_t axis, MotionAxisHardwareStopReason_t *stop_reason) {
    if (motion_task_axis_is_streamed(axis) != 0U) {
        *stop_reason = motion_axis_hardware_get_last_stop_reason(axis);
        if (*stop_reason != MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
            if (motion_task_stream_flush() == 0U) {
                *stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
            }
            return 1U;
        }
        return step_generator_task_is_halted() != 0U || step_generator_task_is_busy() == 0U ? 1U : 0U;
    }
    const MotionAxisHardwareStatus_t hardware_status = motion_axis_hardware_get_status(axis);
    if (motion_task_hardware_is_active(hardware_status) != 0U) {
        return 0U;
    }
    *stop_reason = hardware_status == MOTION_AXIS_HARDWARE_STATUS_STOPPED
                       ? motion_axis_hardware_get_last_stop_reason(axis)
                       : MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    return 1U;
}

/* The moves of the calibration procedure for a streamed axis: one plan block each, like the homing moves. */
static uint8_t motion_task_stream_mover_start(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    const AxisBenchProfile_t *profile
) {
    (void)config;
    return motion_task_stream_start_move(axis, direction, steps, profile);
}

/* The stream stops itself only after the half holding the last record of the block has been played, so a stream that
 * ended with no stop reason and no halt has played every step of the block; that is why `steps` is not checked. A
 * stop on a switch has already been flushed by move_has_ended. A fault halts the generator with no reason; it is
 * flushed here, because nothing else resets it after a failed calibration. */
static AxisCalibrationRunMoveState_t motion_task_stream_mover_poll(MotionAxis_t axis, uint32_t steps) {
    (void)steps;
    MotionAxisHardwareStopReason_t stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (motion_task_move_has_ended(axis, &stop_reason) == 0U) {
        return AXIS_CALIBRATION_RUN_MOVE_RUNNING;
    }
    if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT) {
        return AXIS_CALIBRATION_RUN_MOVE_END_LIMIT;
    }
    if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        if (step_generator_task_is_halted() == 0U) {
            return AXIS_CALIBRATION_RUN_MOVE_COMPLETED;
        }
        (void)motion_task_stream_flush();
    }
    motion_task_home_forget_stream_stop(axis);
    return AXIS_CALIBRATION_RUN_MOVE_FAILED;
}

/* Like the cancel of a homing, this leaves the reset of the generator to whoever aborts: STOP resets it in
 * motion_task_finish_stop once every axis is at rest. */
static void motion_task_stream_mover_abort(MotionAxis_t axis) {
    step_generator_task_halt();
    motion_task_home_forget_stream_stop(axis);
}

static const AxisCalibrationRunMover_t motion_task_stream_mover = {
    .start = motion_task_stream_mover_start,
    .poll = motion_task_stream_mover_poll,
    .abort = motion_task_stream_mover_abort,
};

static const AxisCalibrationRunMover_t *motion_task_calibration_mover(MotionAxis_t axis) {
    return motion_task_axis_is_streamed(axis) != 0U ? &motion_task_stream_mover : &axis_calibration_run_jog_mover;
}

static void motion_task_home_poll(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    if (motion_task_home_is_active(state) == 0U) {
        return;
    }
    /* Every homing move is meant to end on the start switch; running out of steps or any other stop is a
     * failure. */
    MotionAxisHardwareStopReason_t stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (motion_task_move_has_ended(state->axis, &stop_reason) == 0U) {
        return;
    }

    switch (state->home_phase) {
    case MOTION_TASK_HOME_FAST_SEEK:
        if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT) {
            motion_task_home_start_backoff(state, context);
            return;
        }
        break;
    case MOTION_TASK_HOME_BACKOFF:
        if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE) {
            if (motion_axis_hardware_start_limit_is_active(state->axis) != 0U) {
                /* The switch flickered on its edge; keep backing off until it stays off. */
                motion_task_home_start_backoff(state, context);
                return;
            }
            (void)osDelay(MOTION_TASK_HOME_SETTLE_MS);
            motion_task_home_start_approach(state, context);
            return;
        }
        break;
    default:
        /* MOTION_TASK_HOME_APPROACH */
        if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT &&
            motion_axis_hardware_start_limit_is_active(state->axis) != 0U) {
            motion_task_home_complete(state, context);
            return;
        }
        break;
    }
    motion_task_home_abort(state, context);
}

static uint8_t motion_task_calibration_is_busy(const MotionTaskCalibrationContext_t *calibration) {
    return calibration->waiting_for_home != 0U || calibration->returning_home != 0U ||
                   axis_calibration_run_is_active(&calibration->procedure) != 0U
               ? 1U
               : 0U;
}

static uint8_t motion_task_axis_is_busy(const MotionTaskAxisState_t *state) {
    return motion_task_home_is_active(state) != 0U || motion_task_calibration_is_busy(&state->calibration) != 0U ? 1U
                                                                                                                 : 0U;
}

static void motion_task_calibration_begin(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    MotionTaskCalibrationContext_t *calibration = &state->calibration;
    if (motion_task_limits_conflict(state->axis) != 0U) {
        calibration->result = AXIS_CALIBRATION_RUN_RESULT_LIMITS_CONFLICT;
        return;
    }
    calibration->result = AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    calibration->waiting_for_home = 1U;
    /* A homing that cannot start leaves the axis unhomed, and the next poll reports it. */
    motion_task_home_begin(state, context);
}

static void motion_task_calibration_cancel(MotionTaskAxisState_t *state) {
    MotionTaskCalibrationContext_t *calibration = &state->calibration;
    if (motion_task_calibration_is_busy(calibration) != 0U) {
        axis_calibration_run_abort(&calibration->procedure);
        *calibration = (MotionTaskCalibrationContext_t){
            .procedure = {.axis = state->axis},
            .result = AXIS_CALIBRATION_RUN_RESULT_ABORTED,
        };
    }
}

static void motion_task_calibration_poll(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    MotionTaskCalibrationContext_t *calibration = &state->calibration;
    const uint8_t axis_mask = motion_task_axis_mask(state->axis);
    if (calibration->waiting_for_home != 0U || calibration->returning_home != 0U) {
        if (motion_task_home_is_active(state) != 0U) {
            return;
        }
        const uint8_t returning_home = calibration->returning_home;
        calibration->waiting_for_home = 0U;
        calibration->returning_home = 0U;
        if ((context->machine_state.homed_axes_mask & axis_mask) == 0U) {
            /* After the final homing the measured values are kept; the axis stays unhomed until HOME or
             * CALIBRATE. */
            calibration->result = AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED;
            return;
        }
        calibration->result = returning_home != 0U ? AXIS_CALIBRATION_RUN_RESULT_OK
                                                   : axis_calibration_run_begin(
                                                         &calibration->procedure,
                                                         state->axis,
                                                         motion_task_hardware_config(state->axis),
                                                         &motion_task_axis_configs[state->axis].calibration,
                                                         motion_task_calibration_mover(state->axis)
                                                     );
        return;
    }
    if (axis_calibration_run_is_active(&calibration->procedure) == 0U) {
        return;
    }

    AxisCalibration_t measured = {0};
    const AxisCalibrationRunResult_t result =
        axis_calibration_run_poll(&calibration->procedure, motion_task_hardware_config(state->axis), &measured);
    if (result != AXIS_CALIBRATION_RUN_RESULT_OK) {
        calibration->result = result;
        return;
    }
    context->calibration.axis[state->axis] = measured;
    context->calibration.calibrated_axes_mask |= axis_mask;
    motion_task_apply_calibration(context, &context->calibration);
    /* Calibration reports OK only once the axis is back home; a homing that cannot start is reported on the next
     * poll. */
    calibration->returning_home = 1U;
    motion_task_home_begin(state, context);
}

static void motion_task_publish_axis(const MotionTaskAxisState_t *state, const MotionTaskContext_t *context) {
    volatile MotionAxisDiagnostics *diagnostics = &motion_task_axis_diagnostics[state->axis];
    const uint8_t axis_mask = motion_task_axis_mask(state->axis);
    diagnostics->calibration_phase = (uint32_t)state->calibration.procedure.phase;
    diagnostics->calibration_result = (uint32_t)state->calibration.result;
    if (context == NULL) {
        return;
    }
    const AxisCalibration_t *measured = &context->calibration.axis[state->axis];
    const MotionPlannerAxisConfig_t *limits = &context->planner_config.axis[state->axis];
    diagnostics->calibrated = (context->calibration.calibrated_axes_mask & axis_mask) != 0U ? 1U : 0U;
    diagnostics->calibration_steps = measured->steps;
    diagnostics->calibration_distance_um = measured->distance_um;
    diagnostics->travel_um = measured->travel_um;
    diagnostics->min_position_um = limits->min_position_um;
    diagnostics->max_position_um = limits->max_position_um;
    diagnostics->max_speed_um_per_s = limits->max_speed_um_per_s;
    diagnostics->homed = (context->machine_state.homed_axes_mask & axis_mask) != 0U ? 1U : 0U;
    diagnostics->planned_position_um = context->machine_state.position_um[state->axis];
}

static void motion_task_publish_axes(const MotionTaskAxisState_t *states, const MotionTaskContext_t *context) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        motion_task_publish_axis(&states[axis], context);
    }
}

static void motion_task_poll_axes(MotionTaskAxisState_t *states, MotionTaskContext_t *context) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        motion_task_home_poll(&states[axis], context);
        motion_task_calibration_poll(&states[axis], context);
    }
}

static uint8_t motion_task_busy_axes(const MotionTaskAxisState_t *states) {
    uint8_t busy_mask = 0U;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (motion_task_axis_is_busy(&states[axis]) != 0U) {
            busy_mask |= motion_task_axis_mask(states[axis].axis);
        }
    }
    return busy_mask;
}

/* HOME and CALIBRATE name axes that can home and are idle; anything else refuses the whole command. */
static uint8_t motion_task_axes_can_start(uint8_t axes_mask, uint8_t busy_mask) {
    return axes_mask != 0U && (axes_mask & (uint8_t)~MOTION_TASK_HOME_AXES_MASK) == 0U && (axes_mask & busy_mask) == 0U
               ? 1U
               : 0U;
}

void motion_task_request_stop(void) {
    step_generator_task_halt();
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        motion_axis_hardware_request_stop((MotionAxis_t)axis);
    }
    command_queue_discard_all();
}

static uint8_t motion_task_any_hardware_is_active(void) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (motion_task_hardware_is_active(motion_axis_hardware_get_status((MotionAxis_t)axis)) != 0U) {
            return 1U;
        }
    }
    return 0U;
}

/* Second half of STOP: once every axis is at rest the step pipeline is emptied and restarted. If one does not come
 * to rest in time the generator stays halted, so nothing moves until the next STOP succeeds. */
static void motion_task_finish_stop(MotionTaskContext_t *context) {
    for (uint32_t waited_ms = 0U; motion_task_any_hardware_is_active() != 0U; ++waited_ms) {
        if (waited_ms == MOTION_TASK_STOP_SETTLE_TIMEOUT_MS) {
            return;
        }
        (void)osDelay(1U);
    }
    /* Blocks still queued were cut short, so the next move starts from where the scale says each axis is. The scale
     * counts away from zero in either direction, so only its magnitude is the position. */
    for (uint32_t axis = 0U; context != NULL && axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if ((context->machine_state.homed_axes_mask & motion_task_axis_mask((MotionAxis_t)axis)) != 0U) {
            const int32_t scale_um = motion_task_axis_configs[axis].hardware.position_reader();
            context->machine_state.position_um[axis] = scale_um < 0 ? -scale_um : scale_um;
        }
    }
    step_generator_task_resume_after_stop();
}

static void motion_task_stop_axis(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    if (motion_task_home_is_active(state) != 0U) {
        motion_task_home_cancel(state, context);
    } else {
        motion_axis_hardware_request_stop(state->axis);
    }

    /* STOP is also the operator's fault acknowledgement; the axis position is untrusted after a fault. */
    if (motion_task_hardware_status_is_fault(motion_axis_hardware_get_status(state->axis)) != 0U &&
        motion_axis_hardware_clear_fault(state->axis) == MOTION_AXIS_HARDWARE_STATUS_READY && context != NULL) {
        context->machine_state.homed_axes_mask &= (uint8_t)~motion_task_axis_mask(state->axis);
    }
    /* Each test run after a STOP gets its own step-timing peaks in the diagnostics. */
    motion_axis_hardware_reset_timing_peaks(state->axis);
}

static void motion_task_stop(MotionTaskAxisState_t *states, MotionTaskContext_t *context) {
    (void)plan_buffer_clear();
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        motion_task_calibration_cancel(&states[axis]);
        motion_task_stop_axis(&states[axis], context);
    }
    motion_task_finish_stop(context);
}

/* A queue failure leaves no way to supervise a running homing or calibration, so all of them are aborted. */
static void motion_task_abort_axes(MotionTaskAxisState_t *states, MotionTaskContext_t *context) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (motion_task_home_is_active(&states[axis]) != 0U) {
            motion_task_home_abort(&states[axis], context);
        }
        const uint8_t calibrating = motion_task_calibration_is_busy(&states[axis].calibration);
        motion_task_calibration_cancel(&states[axis]);
        if (calibrating != 0U && motion_task_axis_is_streamed((MotionAxis_t)axis) != 0U) {
            /* No STOP follows this abort, so the halted generator is reset here, as in the homing abort. */
            (void)motion_task_stream_flush();
        }
    }
}

static void motion_task_start_axes(
    MotionTaskAxisState_t *states,
    MotionTaskContext_t *context,
    uint8_t axes_mask,
    uint8_t calibrate
) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if ((axes_mask & motion_task_axis_mask((MotionAxis_t)axis)) == 0U) {
            continue;
        }
        if (calibrate != 0U) {
            motion_task_calibration_begin(&states[axis], context);
        } else {
            motion_task_home_begin(&states[axis], context);
        }
    }
}

void MotionTask(void *argument) {
    MotionTaskContext_t *context = argument;
    MotionTaskAxisState_t axes[MOTION_AXIS_HARDWARE_COUNT] = {0};

    /* Drive STEP/DIR/ENABLE of every axis to defined levels before any command can arrive. */
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        axes[axis].axis = (MotionAxis_t)axis;
        axes[axis].calibration.procedure.axis = (MotionAxis_t)axis;
        (void)motion_axis_hardware_init((MotionAxis_t)axis, motion_task_hardware_config((MotionAxis_t)axis));
        if (((1U << axis) & MOTION_TASK_STREAM_AXES_MASK) != 0U) {
            const MotionEventSource_t source = step_executor_event_source(&motion_task_stream_executors[axis]);
            (void)motion_axis_hardware_set_event_source((MotionAxis_t)axis, &source);
        }
    }
    motion_task_load_calibration(context);
    motion_task_publish_axes(axes, context);

    for (;;) {
        CommandDTO_t command = {0};
        const uint8_t busy_mask = motion_task_busy_axes(axes);
        const uint32_t receive_timeout_ms = busy_mask != 0U ? MOTION_TASK_HOME_POLL_INTERVAL_MS : osWaitForever;
        const osStatus_t receive_status = command_queue_receive(&command, receive_timeout_ms);
        if (receive_status != osOK) {
            /* Timeouts only occur while an axis homes or calibrates; the poll is a no-op once that has ended. */
            if (receive_status == osErrorTimeout) {
                motion_task_poll_axes(axes, context);
                motion_task_publish_axes(axes, context);
                continue;
            }
            motion_task_abort_axes(axes, context);
            motion_task_publish_axes(axes, context);
            motion_task_counter_increment(&motion_task_diagnostics.queue_receive_errors);
            (void)osDelay(MOTION_TASK_ERROR_RETRY_DELAY_MS);
            continue;
        }

        motion_task_counter_increment(&motion_task_diagnostics.commands_dequeued);
        motion_task_diagnostics.last_sequence = command.sequence;
        motion_task_diagnostics.last_command_code = (uint32_t)command.code;

        if (command.code == COMMAND_CODE_STOP) {
            motion_task_stop(axes, context);
            motion_task_publish_axes(axes, context);
            continue;
        }

        /* The planned stream owns X's timer from the first queued block to its end, so HOME and CALIBRATE are refused
         * meanwhile like any other command that finds the axis busy. */
        const uint8_t start_busy_mask =
            (uint8_t)(busy_mask | (step_generator_task_is_busy() != 0U ? MOTION_PLANNER_AXIS_X_MASK : 0U));
        if (command.code == COMMAND_CODE_HOME) {
            if (motion_task_axes_can_start(command.parameters.home.axes, start_busy_mask) != 0U) {
                motion_task_start_axes(axes, context, command.parameters.home.axes, 0U);
            }
        } else if (command.code == COMMAND_CODE_CALIBRATE) {
            if (motion_task_axes_can_start(command.parameters.calibrate.axes, start_busy_mask) != 0U) {
                motion_task_start_axes(axes, context, command.parameters.calibrate.axes, 1U);
            }
        } else if (command.code == COMMAND_CODE_X_JOG && busy_mask == 0U) {
            motion_task_queue_x_jog(context, &command);
        } else if (motion_task_is_xyz_move(&command) && busy_mask == 0U) {
            motion_task_queue_xyz_move(context, &command);
        }

        motion_task_poll_axes(axes, context);
        motion_task_publish_axes(axes, context);
    }
}

void motion_task_get_step_timing(MotionAxis_t axis, uint32_t *step_pulse_width_ns, uint32_t *step_low_time_ns) {
    if ((uint32_t)axis >= (uint32_t)MOTION_AXIS_HARDWARE_COUNT || step_pulse_width_ns == NULL ||
        step_low_time_ns == NULL) {
        return;
    }
    *step_pulse_width_ns = motion_task_axis_configs[axis].hardware.step_pulse_width_ns;
    *step_low_time_ns = motion_task_axis_configs[axis].hardware.step_low_time_ns;
}

uint32_t motion_task_get_limit_deceleration(MotionAxis_t axis) {
    if ((uint32_t)axis >= (uint32_t)MOTION_AXIS_HARDWARE_COUNT) {
        return 0U;
    }
    return motion_task_axis_configs[axis].hardware.limit_deceleration_steps_per_s2;
}

uint32_t motion_task_get_stream_underrun_count(void) {
    uint32_t underruns = 0U;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        underruns += step_executor_get_underrun_count(&motion_task_stream_executors[axis]);
    }
    return underruns;
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

void motion_task_get_axis_diagnostics(MotionAxis_t axis, MotionAxisDiagnostics *diagnostics) {
    if (diagnostics == NULL || (uint32_t)axis >= (uint32_t)MOTION_AXIS_HARDWARE_COUNT) {
        return;
    }
    const volatile MotionAxisDiagnostics *source = &motion_task_axis_diagnostics[axis];
    diagnostics->calibration_phase = source->calibration_phase;
    diagnostics->calibration_result = source->calibration_result;
    diagnostics->calibrated = source->calibrated;
    diagnostics->calibration_steps = source->calibration_steps;
    diagnostics->calibration_distance_um = source->calibration_distance_um;
    diagnostics->travel_um = source->travel_um;
    diagnostics->min_position_um = source->min_position_um;
    diagnostics->max_position_um = source->max_position_um;
    diagnostics->max_speed_um_per_s = source->max_speed_um_per_s;
    diagnostics->homed = source->homed;
    diagnostics->planned_position_um = source->planned_position_um;
    diagnostics->scale_position_um = motion_task_axis_configs[axis].hardware.position_reader();
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

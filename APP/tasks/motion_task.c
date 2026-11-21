#include "motion_task.h"

#include "command/command_queue.h"
#include "command/command_result_queue.h"
#include "encoder.h"
#include "motion/axis_bench.h"
#include "motion/axis_calibration_run.h"
#include "motion/calibration_store.h"
#include "motion/home_block.h"
#include "motion/move_result_tracker.h"
#include "motion/plan_buffer.h"
#include "motion/step_executor.h"
#include "step_generator_task.h"

#include "cmsis_os.h"

#include <stddef.h>

/* HOME and CALIBRATE accept any combination of these axes; the axes of one command run at the same time. */
#define MOTION_TASK_HOME_AXES_MASK MOTION_PLANNER_ALL_AXES_MASK
/* Axes the planned step stream drives: every axis has its own stream channel. */
#define MOTION_TASK_STREAM_AXES_MASK                                                                                   \
    (MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK | MOTION_PLANNER_AXIS_Z_MASK)
#define MOTION_TASK_HOME_POLL_INTERVAL_MS 1U
/* Thread flags of MotionTask: a command is queued, the step stream has progress to report, the plan buffer has room. */
#define MOTION_TASK_FLAG_COMMAND (1UL << 0)
#define MOTION_TASK_FLAG_PROGRESS (1UL << 1)
#define MOTION_TASK_FLAG_SPACE (1UL << 2)
#define MOTION_TASK_FLAGS_ALL (MOTION_TASK_FLAG_COMMAND | MOTION_TASK_FLAG_PROGRESS | MOTION_TASK_FLAG_SPACE)
_Static_assert(
    MOVE_RESULT_TRACKER_CAPACITY == COMMAND_RESULT_QUEUE_PENDING_MOVES,
    "the result queue has room for every move that waits for its result"
);
/* Lets the belt come to rest between the backoff and the approach. */
#define MOTION_TASK_HOME_SETTLE_MS 10U
/* The start switch, not a step count, ends every homing move; a switch that never changes is a separate open
 * task. */
#define MOTION_TASK_HOME_MOVE_STEPS AXIS_BENCH_MAX_STEPS

static volatile MotionTaskDiagnostics motion_task_diagnostics = {
    .last_planner_status = MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
};

/* The planned step stream of each axis. */
static StepExecutor_t motion_task_stream_executors[MOTION_AXIS_HARDWARE_COUNT];
/* The queue of the planned moves (the printing stream). Set once when the task starts. The homing and calibration moves
 * go into the queue of the generator instance of their axis instead. */
static PlanBuffer_t *motion_task_plan_buffer;
/* The moves of the printing stream that have been accepted and wait for their final result. */
static MoveResultTracker_t motion_task_move_tracker;
/* Written once when the task starts; the notify functions read it. */
static osThreadId_t volatile motion_task_thread_id;

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
        .approach = {7U, 67U, 333U},                                                                                   \
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
        .approach = {107U, 640U, 4267U},                                                                               \
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
            /* The ramp starts at 50 kHz: the 80 kHz seek drops to it at once at the switch, then stops within about
             * 780 steps (0.04 mm). */
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

/* What MotionTask decides about one command: DONE with a COMMAND_DONE_* code, ERROR with a COMMAND_ERROR_* code, or
 * to wait. A command that waits has changed nothing and gets no answer yet; it is decided again when MotionTask wakes
 * (room in a queue it needs, a STOP). */
typedef struct {
    uint8_t response_type;
    uint16_t code;
    uint8_t wait;
} MotionTaskDecision_t;

static MotionTaskDecision_t motion_task_done(uint16_t result_code) {
    return (MotionTaskDecision_t){.response_type = COMMAND_MESSAGE_TYPE_DONE, .code = result_code};
}

static MotionTaskDecision_t motion_task_wait(void) { return (MotionTaskDecision_t){.wait = 1U}; }

/* Error code 0 means the command was accepted. */
static MotionTaskDecision_t motion_task_decision_from_error(uint16_t error_code) {
    if (error_code == 0U) {
        return motion_task_done(COMMAND_DONE_ACCEPTED);
    }
    return (MotionTaskDecision_t){.response_type = COMMAND_MESSAGE_TYPE_ERROR, .code = error_code};
}

/* Plans one absolute move and hands it to the step generator; a move the planner rejects is dropped. A move to the
 * point where the axes already are is no refusal and is answered as such. The context is not NULL. */
static MotionTaskDecision_t
motion_task_queue_move(MotionTaskContext_t *context, const MotionPlannerRequest_t *request) {
    motion_task_counter_increment(&motion_task_diagnostics.planner_calls);
    MotionPlannerMachineState_t planner_state = context->machine_state;
    const MotionPlannerResult_t result = motion_planner_plan_linear(request, &planner_state, &context->planner_config);
    motion_task_diagnostics.last_planner_status = (uint32_t)result.status;
    motion_task_diagnostics.last_planner_sequence = request->sequence;
#ifdef FM_V3_ENABLE_TEST_HOOKS
    ++motion_task_test_planner_call_count;
    motion_task_test_last_planner_result = result;
#endif
    if (result.status == MOTION_PLANNER_STATUS_OK) {
        motion_task_counter_increment(&motion_task_diagnostics.planner_blocks_created);
        /* The tracker's capacity is what the step feeder can remember of started blocks. The move is decided only while
         * the tracker has room (motion_task_move_waits_for_room), and only this task adds to it, so the add succeeds.
         */
        MotionPlanBlock_t block = result.block;
        block.order = move_result_tracker_add(&motion_task_move_tracker, request->sequence);
        if (plan_buffer_try_push(motion_task_plan_buffer, &block) != PLAN_BUFFER_STATUS_OK) {
            /* The rejected block is dropped; PlanBuffer records why it was not accepted. */
            move_result_tracker_remove_last(&motion_task_move_tracker);
            return motion_task_decision_from_error(COMMAND_ERROR_PLAN_QUEUE_FULL);
        }
        /* The next move starts where this one ends, not where the axis is right now. */
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            context->machine_state.position_um[axis] = request->target_um[axis];
        }
        context->machine_state.syringe_step_remainder = result.syringe_step_remainder;
        step_generator_task_notify_block_ready();
        return motion_task_done(COMMAND_DONE_ACCEPTED);
    }
    if (result.status == MOTION_PLANNER_STATUS_NO_MOVEMENT) {
        motion_task_counter_increment(&motion_task_diagnostics.planner_no_motion);
        return motion_task_done(COMMAND_DONE_ALREADY_AT_TARGET);
    }
    motion_task_counter_increment(&motion_task_diagnostics.planner_errors);
    /* The caller treats the refusal as a malformed command and brings the machine to rest and empty. */
    return motion_task_decision_from_error((uint16_t)(COMMAND_ERROR_PLANNER_BASE + (uint32_t)result.status));
}

/* 1 for a refusal by the planner (COMMAND_ERROR_PLANNER_BASE + status, below the codes of MotionTask itself). */
static uint8_t motion_task_is_planner_refusal(const MotionTaskDecision_t *decision) {
    return decision->response_type == COMMAND_MESSAGE_TYPE_ERROR && decision->code > COMMAND_ERROR_PLANNER_BASE &&
                   decision->code < COMMAND_ERROR_AXIS_BUSY
               ? 1U
               : 0U;
}

/* A target beyond int32 is clamped; it is far outside any axis, so the planner rejects it. */
static int32_t motion_task_relative_target(int32_t position_um, int32_t delta_um) {
    const int64_t target_um = (int64_t)position_um + (int64_t)delta_um;
    return target_um > INT32_MAX ? INT32_MAX : target_um < INT32_MIN ? INT32_MIN : (int32_t)target_um;
}

/* Absolute operations carry the target; relative ones carry increments to the planned position. */
static MotionTaskDecision_t motion_task_queue_xyz_move(MotionTaskContext_t *context, const CommandDTO_t *command) {
    const CommandMotionOperationParameters_t *motion = &command->parameters.motion_operation;
    MotionPlannerRequest_t request = {
        .sequence = command->sequence,
        .target_um = {motion->x, motion->y, motion->z},
        .feedrate_um_per_s = motion->speed,
    };
    if ((motion->operation_flags & COMMAND_MOTION_OPERATION_FLAG_RELATIVE) != 0U) {
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            request.target_um[axis] =
                motion_task_relative_target(context->machine_state.position_um[axis], request.target_um[axis]);
        }
    }
    return motion_task_queue_move(context, &request);
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

/* The generator instance that plays the homing and calibration moves of an axis: one lane on that axis, with its own
 * queue, so the axes of one command move independently. The planned moves of the print go through the printing
 * instance, which this file reaches only through the plan buffer of its context. */
static StepGeneratorInstance_t *motion_task_generator_for_axis(MotionAxis_t axis) {
    return step_generator_task_get_axis_instance(axis);
}

/* Returns the generator instance to its idle state: stops its stream, drops its queued blocks and waits until the task
 * has reset it. Returns 0 when it does not reset in time; the instance then stays halted until the next STOP. */
static uint8_t motion_task_stream_flush(StepGeneratorInstance_t *generator) {
    step_generator_instance_halt(generator);
    (void)plan_buffer_clear(step_generator_instance_get_plan_buffer(generator));
    step_generator_instance_resume_after_stop(generator);
    for (uint32_t waited_ms = 0U; step_generator_instance_is_busy(generator) != 0U; ++waited_ms) {
        if (waited_ms == MOTION_TASK_STOP_SETTLE_TIMEOUT_MS) {
            return 0U;
        }
        (void)osDelay(1U);
    }
    return 1U;
}

/* The stop reason of a stream is only meant for the homing that waits for it; it is not left behind for later moves. */
static void motion_task_home_forget_stream_stop(MotionAxis_t axis) { motion_axis_hardware_clear_stop_reason(axis); }

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
    /* A move still in the stream would run on without anyone watching its switches. */
    (void)motion_task_stream_flush(motion_task_generator_for_axis(state->axis));
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

/* An axis moves with one plan block at the constant target rate of the profile, with no ramp. The stream, idle
 * at this point, plays it; a switch ends it. A switch that blocks the move at once ends it just the same. */
static uint8_t motion_task_stream_start_move(
    MotionAxis_t axis,
    AxisBenchDirection_t direction,
    uint32_t steps,
    const AxisBenchProfile_t *profile
) {
    MotionPlanBlock_t block;
    if (home_block_make(
            axis,
            motion_task_hardware_config(axis),
            direction,
            steps,
            profile->target_step_rate_hz,
            &block
        ) == 0U) {
        return 0U;
    }
    /* The stop of the previous move is still recorded; the next poll must wait for the stop of this one. */
    motion_axis_hardware_clear_stop_reason(axis);
    PlanBuffer_t *const queue = step_generator_instance_get_plan_buffer(motion_task_generator_for_axis(axis));
    if (plan_buffer_try_push(queue, &block) != PLAN_BUFFER_STATUS_OK) {
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
    if (motion_task_stream_start_move(state->axis, direction, MOTION_TASK_HOME_MOVE_STEPS, profile) == 0U) {
        motion_task_home_abort(state, context);
        return;
    }
    state->home_phase = phase;
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

/* Callers start homing only while the axis is idle, so no homing or calibration of it is running here. The context is
 * not NULL: motion_task_start_axes refuses a command without one. */
static void motion_task_home_begin(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    /* The axis is no longer where its zero was, whether or not the homing can start now. */
    const uint8_t axis_mask = motion_task_axis_mask(state->axis);
    context->machine_state.homed_axes_mask &= (uint8_t)~axis_mask;
    if (motion_task_hardware_is_active(motion_axis_hardware_get_status(state->axis)) != 0U) {
        return;
    }

    state->home_phase = MOTION_TASK_HOME_IDLE;
    /* Queued blocks were planned from the old position of this axis. */
    (void)plan_buffer_clear(motion_task_plan_buffer);
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
 * the generator stopped on a fault, or the stream did not reset. The axis is also made ready for the next move
 * here, because a stop on a switch leaves its generator halted. A stream that does not reset blocks MotionTask for the
 * settle timeout here and for another one in the abort that follows. */
static uint8_t motion_task_move_has_ended(
    StepGeneratorInstance_t *generator,
    MotionAxis_t axis,
    MotionAxisHardwareStopReason_t *stop_reason
) {
    *stop_reason = motion_axis_hardware_get_last_stop_reason(axis);
    if (*stop_reason != MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        if (motion_task_stream_flush(generator) == 0U) {
            *stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
        }
        return 1U;
    }
    return step_generator_instance_is_halted(generator) != 0U || step_generator_instance_is_busy(generator) == 0U ? 1U
                                                                                                                  : 0U;
}

/* The moves of the calibration procedure for an axis: one plan block each, like the homing moves. */
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
    StepGeneratorInstance_t *const generator = motion_task_generator_for_axis(axis);
    MotionAxisHardwareStopReason_t stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (motion_task_move_has_ended(generator, axis, &stop_reason) == 0U) {
        return AXIS_CALIBRATION_RUN_MOVE_RUNNING;
    }
    if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT) {
        return AXIS_CALIBRATION_RUN_MOVE_END_LIMIT;
    }
    if (stop_reason == MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        if (step_generator_instance_is_halted(generator) == 0U) {
            return AXIS_CALIBRATION_RUN_MOVE_COMPLETED;
        }
        (void)motion_task_stream_flush(generator);
    }
    motion_task_home_forget_stream_stop(axis);
    return AXIS_CALIBRATION_RUN_MOVE_FAILED;
}

/* Like the cancel of a homing, this leaves the reset of the generator to whoever aborts: STOP resets it in
 * motion_task_finish_stop once every axis is at rest. */
static void motion_task_stream_mover_abort(MotionAxis_t axis) {
    step_generator_instance_halt(motion_task_generator_for_axis(axis));
    motion_task_home_forget_stream_stop(axis);
}

static const AxisCalibrationRunMover_t motion_task_stream_mover = {
    .start = motion_task_stream_mover_start,
    .poll = motion_task_stream_mover_poll,
    .abort = motion_task_stream_mover_abort,
};

static void motion_task_home_poll(MotionTaskAxisState_t *state, MotionTaskContext_t *context) {
    if (motion_task_home_is_active(state) == 0U) {
        return;
    }
    /* Every homing move is meant to end on the start switch; running out of steps or any other stop is a
     * failure. */
    MotionAxisHardwareStopReason_t stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (motion_task_move_has_ended(motion_task_generator_for_axis(state->axis), state->axis, &stop_reason) == 0U) {
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
                                                         &motion_task_stream_mover
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

/* The axes whose single-axis generator instance is busy: it plays a homing or calibration move, or has not been reset
 * after one yet. */
static uint8_t motion_task_axis_streams_busy_axes(void) {
    uint8_t busy_mask = 0U;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (step_generator_instance_is_busy(motion_task_generator_for_axis((MotionAxis_t)axis)) != 0U) {
            busy_mask |= motion_task_axis_mask((MotionAxis_t)axis);
        }
    }
    return busy_mask;
}

/* The axes whose hardware moves right now; a command that names one of them is refused as a whole, so it never starts
 * only some of its axes. */
static uint8_t motion_task_active_hardware_axes(void) {
    uint8_t active_mask = 0U;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (motion_task_hardware_is_active(motion_axis_hardware_get_status((MotionAxis_t)axis)) != 0U) {
            active_mask |= motion_task_axis_mask((MotionAxis_t)axis);
        }
    }
    return active_mask;
}

/* The axes whose stream channels a generator instance holds: all of them while the printing stream is busy, plus the
 * axes of the busy single-axis instances. HOME and CALIBRATE of other axes are not held up by the latter. */
static uint8_t motion_task_stream_busy_axes(void) {
    const uint8_t print_busy_mask = step_generator_instance_is_busy(step_generator_task_get_print_instance()) != 0U
                                        ? MOTION_TASK_STREAM_AXES_MASK
                                        : 0U;
    return (uint8_t)(print_busy_mask | motion_task_axis_streams_busy_axes());
}

/* 1 while the printing stream is halted and STOP can lift it. An instance whose feeder was never built stays halted
 * for good and does nothing, so it is not counted: refusing commands for it would only lock out the axes, whose
 * HOME and CALIBRATE run on their own instances. */
static uint8_t motion_task_print_stream_halted(void) {
    const StepGeneratorInstance_t *const print = step_generator_task_get_print_instance();
    return (uint8_t)(step_generator_instance_is_ready(print) != 0U && step_generator_instance_is_halted(print) != 0U);
}

/* HOME and CALIBRATE name axes that can home and are idle; anything else refuses the whole command. Returns 0 when the
 * command may start, otherwise the COMMAND_ERROR_* code of the refusal. */
static uint16_t motion_task_axes_start_refusal(uint8_t axes_mask, uint8_t busy_mask, uint8_t stream_halted) {
    if (axes_mask == 0U || (axes_mask & (uint8_t)~MOTION_TASK_HOME_AXES_MASK) != 0U) {
        return COMMAND_ERROR_INVALID_PARAMETER;
    }
    if (stream_halted != 0U) {
        return COMMAND_ERROR_STREAM_HALTED;
    }
    return (axes_mask & busy_mask) != 0U ? COMMAND_ERROR_AXIS_BUSY : 0U;
}

/* Stops the pulses at the next timer interrupt and starts nothing new. */
static void motion_task_halt_motion(void) {
    step_generator_task_halt_all();
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        motion_axis_hardware_request_stop((MotionAxis_t)axis);
    }
}

/* The network task accepts a STOP: the motion stops and the commands accepted earlier are void. The flush starts a new
 * epoch of the command stream after it has emptied the queue, so the move MotionTask holds back and the results it has
 * given for commands taken before are cancelled with it, in the window between the halt and the flush as well. */
void motion_task_request_stop(void) {
    motion_task_halt_motion();
    command_queue_flush();
}

static uint8_t motion_task_any_hardware_is_active(void) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (motion_task_hardware_is_active(motion_axis_hardware_get_status((MotionAxis_t)axis)) != 0U) {
            return 1U;
        }
    }
    return 0U;
}

/* Second half of STOP: once every axis is at rest the step pipeline is emptied and restarted. Returns 0 when an axis
 * does not come to rest in time, or the printing stream is still or again halted after its reset: the generator stays
 * halted, so nothing moves until the next STOP succeeds. */
static uint8_t motion_task_finish_stop(MotionTaskContext_t *context) {
    for (uint32_t waited_ms = 0U; motion_task_any_hardware_is_active() != 0U; ++waited_ms) {
        if (waited_ms == MOTION_TASK_STOP_SETTLE_TIMEOUT_MS) {
            return 0U;
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
    const StepGeneratorInstance_t *const print = step_generator_task_get_print_instance();
    const uint32_t resets_before = step_generator_instance_reset_count(print);
    step_generator_task_resume_all_after_stop();
    /* The generator task resets on its own pass, so the next command could still find the stream halted. The wait is
     * bounded, and it ends only when the reset has finished: it lifts the halt first and a failing reset halts again.
     * An instance that is not ready is never reset, so there is nothing to wait for. */
    for (
        uint32_t waited_ms = 0U;
        step_generator_instance_is_ready(print) != 0U && step_generator_instance_reset_count(print) == resets_before;
        ++waited_ms
    ) {
        if (waited_ms == MOTION_TASK_STOP_SETTLE_TIMEOUT_MS) {
            return 0U;
        }
        (void)osDelay(1U);
    }
    return motion_task_print_stream_halted() == 0U ? 1U : 0U;
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

/* `context` is the epoch of the command stream that was read before the snapshot of the stream: a STOP that comes
 * after that read voids the results of the snapshot too. */
static void motion_task_emit_move_result(void *context, const MoveResult_t *result) {
    const uint32_t epoch = *(const uint32_t *)context;
    if (result->kind == MOVE_RESULT_COMPLETED) {
        command_result_queue_post_in_epoch(epoch, result->sequence, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_COMPLETED);
        return;
    }
    command_result_queue_post_detailed_in_epoch(
        epoch,
        result->sequence,
        COMMAND_MESSAGE_TYPE_ERROR,
        COMMAND_ERROR_STREAM_INTERRUPTED,
        result->cause,
        result->axis,
        result->detail
    );
}

/* Takes the snapshot of the printing stream and answers the moves it has played or cut off. */
static void motion_task_process_move_results(void) {
    uint32_t epoch = command_queue_epoch();
    StepGeneratorProgress_t progress = {0};
    step_generator_instance_get_progress(step_generator_task_get_print_instance(), &progress);
    const MoveResultProgress_t input = {
        .generation = progress.generation,
        .played_blocks = progress.played_blocks,
        .played_order = progress.played_order,
        .stop_resolved = progress.stop_resolved,
        /* STOP and a disconnect answer for themselves. */
        .report_stop = progress.halt_cause != STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL ? 1U : 0U,
        .has_unplayed = progress.has_unplayed,
        .unplayed_order = progress.unplayed_order,
        .cause = (uint8_t)progress.halt_cause,
        .axis =
            progress.halt_axis == STEP_GENERATOR_TASK_NO_AXIS ? COMMAND_ERROR_AXIS_NONE : (uint8_t)progress.halt_axis,
        .detail = (uint8_t)progress.halt_detail,
    };
    move_result_tracker_update(&motion_task_move_tracker, &input, motion_task_emit_move_result, &epoch);
}

static void motion_task_signal(uint32_t flags) {
    const osThreadId_t thread_id = motion_task_thread_id;
    if (thread_id != NULL) {
        (void)osThreadFlagsSet(thread_id, flags);
    }
}

void motion_task_notify_command(void) { motion_task_signal(MOTION_TASK_FLAG_COMMAND); }

void motion_task_notify_progress(void) { motion_task_signal(MOTION_TASK_FLAG_PROGRESS); }

/* The step generator has freed room in the plan buffer of the printing stream. */
static void motion_task_notify_space(void) { motion_task_signal(MOTION_TASK_FLAG_SPACE); }

/* Returns 0 when the axes did not come to rest in time or the printing stream is still halted after its reset. */
static uint8_t motion_task_stop(MotionTaskAxisState_t *states, MotionTaskContext_t *context) {
    /* The moves that waited are cancelled with the stream; STOP is their answer. */
    StepGeneratorProgress_t progress = {0};
    step_generator_instance_get_progress(step_generator_task_get_print_instance(), &progress);
    move_result_tracker_reset(&motion_task_move_tracker, progress.generation);
    (void)plan_buffer_clear(motion_task_plan_buffer);
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        /* The homing and calibration moves still queued were cut short as well. */
        const StepGeneratorInstance_t *const generator = motion_task_generator_for_axis((MotionAxis_t)axis);
        (void)plan_buffer_clear(step_generator_instance_get_plan_buffer(generator));
        motion_task_calibration_cancel(&states[axis]);
        motion_task_stop_axis(&states[axis], context);
    }
    return motion_task_finish_stop(context);
}

/* A queue failure leaves no way to supervise a running homing or calibration, so all of them are aborted. */
static void motion_task_abort_axes(MotionTaskAxisState_t *states, MotionTaskContext_t *context) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if (motion_task_home_is_active(&states[axis]) != 0U) {
            motion_task_home_abort(&states[axis], context);
        }
        const uint8_t calibrating = motion_task_calibration_is_busy(&states[axis].calibration);
        motion_task_calibration_cancel(&states[axis]);
        if (calibrating != 0U) {
            /* No STOP follows this abort, so the halted generator is reset here, as in the homing abort. */
            (void)motion_task_stream_flush(motion_task_generator_for_axis(states[axis].axis));
        }
    }
}

/* Starts the homing or calibration of every named axis. Returns 0 when all of them started, otherwise the code of the
 * first axis that did not; the others still run. */
static uint16_t motion_task_start_axes(
    MotionTaskAxisState_t *states,
    MotionTaskContext_t *context,
    uint8_t axes_mask,
    uint8_t calibrate
) {
    if (context == NULL) {
        return COMMAND_ERROR_NOT_CONFIGURED;
    }
    uint16_t error_code = 0U;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        if ((axes_mask & motion_task_axis_mask((MotionAxis_t)axis)) == 0U) {
            continue;
        }
        const uint8_t limits_conflict = motion_task_limits_conflict((MotionAxis_t)axis);
        if (calibrate != 0U) {
            motion_task_calibration_begin(&states[axis], context);
        } else {
            motion_task_home_begin(&states[axis], context);
        }
        /* Both procedures begin with a homing move, so an idle homing means the axis did not start. */
        if (motion_task_home_is_active(&states[axis]) == 0U && error_code == 0U) {
            error_code = limits_conflict != 0U ? COMMAND_ERROR_LIMITS_CONFLICT : COMMAND_ERROR_HOME_NOT_STARTED;
        }
    }
    return error_code;
}

/* 1 while the plan buffer of the printing stream or the tracker of the moves has no room for one more move. */
static uint8_t motion_task_move_waits_for_room(void) {
    const uint8_t plan_full = plan_buffer_is_full(motion_task_plan_buffer);
    const uint8_t tracker_full = move_result_tracker_has_room(&motion_task_move_tracker) == 0U ? 1U : 0U;
    return (plan_full | tracker_full) != 0U ? 1U : 0U;
}

/* The decision about one command other than STOP: accepted, refused with the reason, or to wait for room. The move
 * itself is not finished by then. */
static MotionTaskDecision_t motion_task_execute_command(
    MotionTaskAxisState_t *states,
    MotionTaskContext_t *context,
    const CommandDTO_t *command,
    uint8_t busy_mask
) {
    /* A halted stream stays so until STOP, whatever the cause; a command taken now would only queue behind it (a move
     * would even shift the planned position) while the client is not told that STOP is needed. A malformed or
     * unsupported command is refused for that first, so the client sees its own mistake and not the state. */
    const uint8_t stream_halted = motion_task_print_stream_halted();
    /* A stream owns the timers of its axes from the first queued block to its end, so HOME and CALIBRATE are
     * refused for those axes meanwhile like any other command that finds the axis busy. */
    const uint8_t start_busy_mask =
        (uint8_t)(busy_mask | motion_task_stream_busy_axes() | motion_task_active_hardware_axes());
    if (command->code == COMMAND_CODE_HOME || command->code == COMMAND_CODE_CALIBRATE) {
        const uint8_t calibrate = command->code == COMMAND_CODE_CALIBRATE ? 1U : 0U;
        const uint8_t axes_mask = calibrate != 0U ? command->parameters.calibrate.axes : command->parameters.home.axes;
        const uint16_t refusal = motion_task_axes_start_refusal(axes_mask, start_busy_mask, stream_halted);
        return motion_task_decision_from_error(
            refusal != 0U ? refusal : motion_task_start_axes(states, context, axes_mask, calibrate)
        );
    }
    if (motion_task_is_xyz_move(command) != 0) {
        /* A stream whose feeder was never built plays nothing, so a block queued for it would wait for good. */
        if (context == NULL || step_generator_instance_is_ready(step_generator_task_get_print_instance()) == 0U) {
            return motion_task_decision_from_error(COMMAND_ERROR_NOT_CONFIGURED);
        }
        if (stream_halted != 0U) {
            return motion_task_decision_from_error(COMMAND_ERROR_STREAM_HALTED);
        }
        if (busy_mask != 0U || motion_task_axis_streams_busy_axes() != 0U) {
            return motion_task_decision_from_error(COMMAND_ERROR_AXIS_BUSY);
        }
        /* Last, after the refusals that depend on the state of the machine, so a halted stream, which frees no room,
         * never waits. The planner's own refusal comes only after the wait, when the move is decided for good. */
        if (motion_task_move_waits_for_room() != 0U) {
            return motion_task_wait();
        }
        return motion_task_queue_xyz_move(context, command);
    }
    return motion_task_decision_from_error(COMMAND_ERROR_NOT_SUPPORTED);
}

/* Sleeps until a command arrives, the stream makes progress or the plan buffer has room; while an axis homes or
 * calibrates it also wakes to poll it. A flag set between the caller's check and the wait is not lost. */
static void motion_task_wait_for_work(MotionTaskAxisState_t *axes, MotionTaskContext_t *context, uint8_t busy_mask) {
    const uint32_t wait_timeout_ms = busy_mask != 0U ? MOTION_TASK_HOME_POLL_INTERVAL_MS : osWaitForever;
    const uint32_t flags = osThreadFlagsWait(MOTION_TASK_FLAGS_ALL, osFlagsWaitAny, wait_timeout_ms);
    /* Timeouts only occur while an axis homes or calibrates; the poll is a no-op once that has ended. */
    if (flags == (uint32_t)osFlagsErrorTimeout) {
        motion_task_poll_axes(axes, context);
        motion_task_publish_axes(axes, context);
    } else if ((flags & (uint32_t)osFlagsError) != 0U) {
        motion_task_counter_increment(&motion_task_diagnostics.queue_receive_errors);
        (void)osDelay(MOTION_TASK_ERROR_RETRY_DELAY_MS);
    }
}

/* The command MotionTask took from the queue and could not decide yet. While it waits the queue is not read, so the
 * commands behind it keep their place. Only MotionTask touches it. */
typedef struct {
    CommandDTO_t command;
    /* The epoch of the command stream when the command was taken. */
    uint32_t epoch;
    uint8_t valid;
} MotionTaskDeferred_t;

/* The next command to decide: the deferred one while no STOP has come since it was taken, otherwise the next of the
 * queue, which is read without waiting (the task sleeps on its flags instead). A command taken before a STOP is older
 * than it, so a deferred one is dropped without an answer when the epoch has moved on. The epoch is read before the
 * queue: a flush after that read makes it differ, whichever side of the drain the command was taken on. */
static osStatus_t motion_task_take_command(MotionTaskDeferred_t *deferred, CommandDTO_t *command, uint32_t *epoch) {
    if (deferred->valid != 0U) {
        deferred->valid = 0U;
        if (deferred->epoch == command_queue_epoch()) {
            *command = deferred->command;
            *epoch = deferred->epoch;
            return osOK;
        }
    }
    *epoch = command_queue_epoch();
    const osStatus_t status = command_queue_receive(command, 0U);
    if (status == osOK) {
        motion_task_counter_increment(&motion_task_diagnostics.commands_dequeued);
        motion_task_diagnostics.last_sequence = command->sequence;
        motion_task_diagnostics.last_command_code = (uint32_t)command->code;
    }
    return status;
}

void MotionTask(void *argument) {
    MotionTaskContext_t *context = argument;
    MotionTaskAxisState_t axes[MOTION_AXIS_HARDWARE_COUNT] = {0};
    MotionTaskDeferred_t deferred = {0};
    motion_task_plan_buffer = context == NULL ? NULL : context->plan_buffer;
    motion_task_thread_id = osThreadGetId();
    {
        /* A task that starts has no move waiting for a result. */
        StepGeneratorProgress_t progress = {0};
        step_generator_instance_get_progress(step_generator_task_get_print_instance(), &progress);
        move_result_tracker_reset(&motion_task_move_tracker, progress.generation);
    }
    command_queue_set_put_listener(motion_task_notify_command);
    step_generator_task_set_progress_listener(motion_task_notify_progress);
    plan_buffer_set_space_listener(motion_task_plan_buffer, motion_task_notify_space);

    /* Drive STEP/DIR/ENABLE of every axis to defined levels before any command can arrive. */
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        axes[axis].axis = (MotionAxis_t)axis;
        axes[axis].calibration.procedure.axis = (MotionAxis_t)axis;
        (void)motion_axis_hardware_init((MotionAxis_t)axis, motion_task_hardware_config((MotionAxis_t)axis));
        const MotionEventSource_t source = step_executor_event_source(&motion_task_stream_executors[axis]);
        (void)motion_axis_hardware_set_event_source((MotionAxis_t)axis, &source);
    }
    motion_task_load_calibration(context);
    motion_task_publish_axes(axes, context);

    for (;;) {
        motion_task_process_move_results();
        CommandDTO_t command = {0};
        uint32_t epoch = 0U;
        const uint8_t busy_mask = motion_task_busy_axes(axes);
        /* A command, the progress of the stream and room in the plan buffer all wake the task, and a flag set between
         * the read and the wait is not lost. */
        const osStatus_t receive_status = motion_task_take_command(&deferred, &command, &epoch);
        if (receive_status == osErrorResource || receive_status == osErrorTimeout) {
            motion_task_wait_for_work(axes, context, busy_mask);
            continue;
        }
        if (receive_status != osOK) {
            motion_task_abort_axes(axes, context);
            motion_task_publish_axes(axes, context);
            motion_task_counter_increment(&motion_task_diagnostics.queue_receive_errors);
            (void)osDelay(MOTION_TASK_ERROR_RETRY_DELAY_MS);
            continue;
        }

        if (command.code == COMMAND_CODE_STOP) {
            const uint8_t settled = motion_task_stop(axes, context);
            /* Whatever the client has not read of the earlier commands is gone with them. */
            command_result_queue_discard_all();
            if (command.sequence != COMMAND_SEQUENCE_INTERNAL) {
                const MotionTaskDecision_t decision =
                    motion_task_decision_from_error(settled != 0U ? 0U : COMMAND_ERROR_STOP_NOT_SETTLED);
                /* The epoch of now, not of the read before the queue: the flush that came with this STOP is older. */
                command_result_queue_post_in_epoch(
                    command_queue_epoch(),
                    command.sequence,
                    decision.response_type,
                    decision.code
                );
            }
            motion_task_publish_axes(axes, context);
            continue;
        }

        const MotionTaskDecision_t decision = motion_task_execute_command(axes, context, &command, busy_mask);
        if (decision.wait != 0U) {
            /* No room yet and nothing changed: the move stays here and is decided again when the task wakes. */
            deferred = (MotionTaskDeferred_t){.command = command, .epoch = epoch, .valid = 1U};
            motion_task_wait_for_work(axes, context, busy_mask);
            continue;
        }
        if (motion_task_is_planner_refusal(&decision) != 0U) {
            /* A move the planner refuses is a client bug, and what was queued behind it cannot be trusted: everything
             * is stopped and emptied as by STOP (streams, rings, plan queues, command queue, results the client has not
             * read), and only this command is answered, with the planner's reason. No flush: no command is deferred
             * here, and the answer below must not be void. */
            motion_task_halt_motion();
            command_queue_discard_pending();
            (void)motion_task_stop(axes, context);
            command_result_queue_discard_all();
        }
        command_result_queue_post_in_epoch(epoch, command.sequence, decision.response_type, decision.code);

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

void motion_task_test_fill_tracker(void) {
    while (move_result_tracker_add(&motion_task_move_tracker, 0U) != 0U) {
    }
}

MotionPlannerResult_t motion_task_test_get_last_planner_result(void) { return motion_task_test_last_planner_result; }
#endif

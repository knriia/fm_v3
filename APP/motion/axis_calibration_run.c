#include "motion/axis_calibration_run.h"

#include "motion/axis_bench.h"

#include <stddef.h>

static uint32_t axis_calibration_run_distance_um(int32_t from_um, int32_t to_um) {
    const int64_t delta = (int64_t)to_um - (int64_t)from_um;
    return (uint32_t)(delta < 0 ? -delta : delta);
}

static uint8_t axis_calibration_run_hardware_is_moving(MotionAxisHardwareStatus_t status) {
    return status == MOTION_AXIS_HARDWARE_STATUS_STARTING || status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
           status == MOTION_AXIS_HARDWARE_STATUS_RUNNING;
}

static uint8_t axis_calibration_run_jog_start(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    const AxisBenchProfile_t *profile
) {
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    return axis_bench_start_profile(
               axis,
               config,
               direction,
               steps,
               profile->initial_step_rate_hz,
               profile->target_step_rate_hz,
               profile->acceleration_steps_per_s2,
               &hardware_status
           ) == AXIS_BENCH_STATUS_STARTED
               ? 1U
               : 0U;
}

/* A move must run all its steps unless the end switch stops it; any other stop means the axis did not travel as
 * commanded. */
static AxisCalibrationRunMoveState_t axis_calibration_run_jog_poll(MotionAxis_t axis, uint32_t steps) {
    const MotionAxisHardwareStatus_t status = motion_axis_hardware_get_status(axis);
    if (axis_calibration_run_hardware_is_moving(status) != 0U) {
        return AXIS_CALIBRATION_RUN_MOVE_RUNNING;
    }
    if (status == MOTION_AXIS_HARDWARE_STATUS_STOPPED &&
        motion_axis_hardware_get_last_stop_reason(axis) == MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT) {
        return AXIS_CALIBRATION_RUN_MOVE_END_LIMIT;
    }
    if (status == MOTION_AXIS_HARDWARE_STATUS_COMPLETE &&
        motion_axis_hardware_get_last_jog_steps_completed(axis) == steps) {
        return AXIS_CALIBRATION_RUN_MOVE_COMPLETED;
    }
    return AXIS_CALIBRATION_RUN_MOVE_FAILED;
}

static void axis_calibration_run_jog_abort(MotionAxis_t axis) { motion_axis_hardware_request_stop(axis); }

const AxisCalibrationRunMover_t axis_calibration_run_jog_mover = {
    .start = axis_calibration_run_jog_start,
    .poll = axis_calibration_run_jog_poll,
    .abort = axis_calibration_run_jog_abort,
};

static AxisCalibrationRunResult_t
axis_calibration_run_fail(AxisCalibrationRun_t *run, AxisCalibrationRunResult_t result) {
    *run = (AxisCalibrationRun_t){.axis = run->axis};
    return result;
}

/* Take-up and measurement both head from the end switch back toward the start. */
static AxisCalibrationRunResult_t axis_calibration_run_start_return_move(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    uint32_t steps,
    AxisCalibrationRunPhase_t phase
) {
    if (run->mover->start(run->axis, config, AXIS_BENCH_TOWARD_START, steps, &run->settings->measure) == 0U) {
        return axis_calibration_run_fail(run, AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED);
    }
    run->phase = phase;
    return AXIS_CALIBRATION_RUN_RESULT_RUNNING;
}

AxisCalibrationRunResult_t axis_calibration_run_begin(
    AxisCalibrationRun_t *run,
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *config,
    const AxisCalibrationRunSettings_t *settings,
    const AxisCalibrationRunMover_t *mover
) {
    *run = (AxisCalibrationRun_t){.axis = axis, .settings = settings, .mover = mover};
    if (mover->start(axis, config, AXIS_BENCH_TOWARD_END, AXIS_BENCH_MAX_STEPS, &settings->seek) == 0U) {
        *run = (AxisCalibrationRun_t){.axis = axis};
        return AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED;
    }
    run->phase = AXIS_CALIBRATION_RUN_PHASE_SEEK_END;
    return AXIS_CALIBRATION_RUN_RESULT_RUNNING;
}

/* The end switch stops the seek; the scale reading latched when it fired, zeroed at the start switch, is the
 * whole travel. The axis brakes a little past that point. */
static AxisCalibrationRunResult_t
axis_calibration_run_poll_seek_end(AxisCalibrationRun_t *run, const MotionAxisHardwareConfig_t *config) {
    const AxisCalibrationRunMoveState_t state = run->mover->poll(run->axis, AXIS_BENCH_MAX_STEPS);
    if (state == AXIS_CALIBRATION_RUN_MOVE_RUNNING) {
        return AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    }
    if (state != AXIS_CALIBRATION_RUN_MOVE_END_LIMIT) {
        return axis_calibration_run_fail(run, AXIS_CALIBRATION_RUN_RESULT_END_NOT_REACHED);
    }

    run->travel_um = axis_calibration_run_distance_um(0, motion_axis_hardware_get_limit_trigger_position(run->axis));
    if (run->travel_um == 0U) {
        return axis_calibration_run_fail(run, AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE);
    }

    return axis_calibration_run_start_return_move(
        run,
        config,
        run->settings->take_up_steps,
        AXIS_CALIBRATION_RUN_PHASE_TAKE_UP
    );
}

/* With the slack taken up the axis is at rest; the measurement starts from this scale reading. */
static AxisCalibrationRunResult_t
axis_calibration_run_poll_take_up(AxisCalibrationRun_t *run, const MotionAxisHardwareConfig_t *config) {
    const AxisCalibrationRunMoveState_t state = run->mover->poll(run->axis, run->settings->take_up_steps);
    if (state == AXIS_CALIBRATION_RUN_MOVE_RUNNING) {
        return AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    }
    if (state != AXIS_CALIBRATION_RUN_MOVE_COMPLETED) {
        return axis_calibration_run_fail(run, AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED);
    }
    run->measure_start_um = config->position_reader();
    return axis_calibration_run_start_return_move(
        run,
        config,
        run->settings->measure_steps,
        AXIS_CALIBRATION_RUN_PHASE_MEASURE
    );
}

static AxisCalibrationRunResult_t axis_calibration_run_poll_measure(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    AxisCalibration_t *result
) {
    const AxisCalibrationRunMoveState_t state = run->mover->poll(run->axis, run->settings->measure_steps);
    if (state == AXIS_CALIBRATION_RUN_MOVE_RUNNING) {
        return AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    }
    if (state != AXIS_CALIBRATION_RUN_MOVE_COMPLETED) {
        return axis_calibration_run_fail(run, AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED);
    }

    const uint32_t distance_um = axis_calibration_run_distance_um(run->measure_start_um, config->position_reader());
    if (distance_um == 0U) {
        return axis_calibration_run_fail(run, AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE);
    }

    *result = (AxisCalibration_t){
        .steps = run->settings->measure_steps,
        .distance_um = distance_um,
        .travel_um = run->travel_um,
    };
    *run = (AxisCalibrationRun_t){.axis = run->axis};
    return AXIS_CALIBRATION_RUN_RESULT_OK;
}

AxisCalibrationRunResult_t axis_calibration_run_poll(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    AxisCalibration_t *result
) {
    switch (run->phase) {
    case AXIS_CALIBRATION_RUN_PHASE_SEEK_END:
        return axis_calibration_run_poll_seek_end(run, config);
    case AXIS_CALIBRATION_RUN_PHASE_TAKE_UP:
        return axis_calibration_run_poll_take_up(run, config);
    case AXIS_CALIBRATION_RUN_PHASE_MEASURE:
        return axis_calibration_run_poll_measure(run, config, result);
    default:
        return AXIS_CALIBRATION_RUN_RESULT_NONE;
    }
}

void axis_calibration_run_abort(AxisCalibrationRun_t *run) {
    if (axis_calibration_run_is_active(run) != 0U) {
        run->mover->abort(run->axis);
    }
    *run = (AxisCalibrationRun_t){.axis = run->axis};
}

uint8_t axis_calibration_run_is_active(const AxisCalibrationRun_t *run) {
    return run->phase != AXIS_CALIBRATION_RUN_PHASE_IDLE ? 1U : 0U;
}

#include "motion/x_axis_calibration.h"

#include "encoder.h"
#include "motion/x_axis_bench.h"

#include <stddef.h>

static int32_t x_axis_calibration_toward_end(const MotionHardwareXConfig_t *config, uint32_t steps) {
    return config->positive_moves_toward_end != 0U ? (int32_t)steps : -(int32_t)steps;
}

static uint32_t x_axis_calibration_distance_um(int32_t from_um, int32_t to_um) {
    const int64_t delta = (int64_t)to_um - (int64_t)from_um;
    return (uint32_t)(delta < 0 ? -delta : delta);
}

static uint8_t x_axis_calibration_hardware_is_moving(MotionHardwareXStatus_t status) {
    return status == MOTION_HARDWARE_X_STATUS_STARTING || status == MOTION_HARDWARE_X_STATUS_WAITING ||
           status == MOTION_HARDWARE_X_STATUS_RUNNING;
}

static XAxisCalibrationResult_t
x_axis_calibration_fail(XAxisCalibration_t *calibration, XAxisCalibrationResult_t result) {
    *calibration = (XAxisCalibration_t){0};
    return result;
}

XAxisCalibrationResult_t
x_axis_calibration_begin(XAxisCalibration_t *calibration, const MotionHardwareXConfig_t *config) {
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
    *calibration = (XAxisCalibration_t){0};
    if (x_axis_bench_start_profile(
            config,
            x_axis_calibration_toward_end(config, X_AXIS_BENCH_MAX_STEPS),
            X_AXIS_CALIBRATION_SEEK_INITIAL_STEP_RATE_HZ,
            X_AXIS_CALIBRATION_SEEK_STEP_RATE_HZ,
            X_AXIS_CALIBRATION_SEEK_ACCELERATION_STEPS_PER_S2,
            &hardware_status
        ) != X_AXIS_BENCH_STATUS_STARTED) {
        return X_AXIS_CALIBRATION_RESULT_MOVE_FAILED;
    }
    calibration->phase = X_AXIS_CALIBRATION_PHASE_SEEK_END;
    return X_AXIS_CALIBRATION_RESULT_RUNNING;
}

/* The end switch stops the seek; the scale, zeroed at the start switch, then reads the whole travel. */
static XAxisCalibrationResult_t
x_axis_calibration_poll_seek_end(XAxisCalibration_t *calibration, const MotionHardwareXConfig_t *config) {
    const MotionHardwareXStatus_t status = motion_hardware_x_get_status();
    if (x_axis_calibration_hardware_is_moving(status) != 0U) {
        return X_AXIS_CALIBRATION_RESULT_RUNNING;
    }
    if (status != MOTION_HARDWARE_X_STATUS_STOPPED ||
        motion_hardware_x_get_last_stop_reason() != MOTION_HARDWARE_X_STOP_REASON_END_LIMIT) {
        return x_axis_calibration_fail(calibration, X_AXIS_CALIBRATION_RESULT_END_NOT_REACHED);
    }

    const int32_t end_um = encoder_ox_get_coordinate();
    calibration->travel_um = x_axis_calibration_distance_um(0, end_um);
    if (calibration->travel_um == 0U) {
        return x_axis_calibration_fail(calibration, X_AXIS_CALIBRATION_RESULT_NO_DISTANCE);
    }

    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
    calibration->measure_start_um = end_um;
    if (x_axis_bench_start(
            config,
            -x_axis_calibration_toward_end(config, X_AXIS_CALIBRATION_MEASURE_STEPS),
            X_AXIS_CALIBRATION_MEASURE_STEP_RATE_HZ,
            &hardware_status
        ) != X_AXIS_BENCH_STATUS_STARTED) {
        return x_axis_calibration_fail(calibration, X_AXIS_CALIBRATION_RESULT_MOVE_FAILED);
    }
    calibration->phase = X_AXIS_CALIBRATION_PHASE_MEASURE;
    return X_AXIS_CALIBRATION_RESULT_RUNNING;
}

static XAxisCalibrationResult_t
x_axis_calibration_poll_measure(XAxisCalibration_t *calibration, AxisCalibration_t *result) {
    const MotionHardwareXStatus_t status = motion_hardware_x_get_status();
    if (x_axis_calibration_hardware_is_moving(status) != 0U) {
        return X_AXIS_CALIBRATION_RESULT_RUNNING;
    }
    if (status != MOTION_HARDWARE_X_STATUS_COMPLETE ||
        motion_hardware_x_get_last_jog_steps_completed() != X_AXIS_CALIBRATION_MEASURE_STEPS) {
        return x_axis_calibration_fail(calibration, X_AXIS_CALIBRATION_RESULT_MOVE_FAILED);
    }

    const uint32_t distance_um =
        x_axis_calibration_distance_um(calibration->measure_start_um, encoder_ox_get_coordinate());
    if (distance_um == 0U) {
        return x_axis_calibration_fail(calibration, X_AXIS_CALIBRATION_RESULT_NO_DISTANCE);
    }

    *result = (AxisCalibration_t){
        .steps = X_AXIS_CALIBRATION_MEASURE_STEPS,
        .distance_um = distance_um,
        .travel_um = calibration->travel_um,
    };
    *calibration = (XAxisCalibration_t){0};
    return X_AXIS_CALIBRATION_RESULT_OK;
}

XAxisCalibrationResult_t x_axis_calibration_poll(
    XAxisCalibration_t *calibration,
    const MotionHardwareXConfig_t *config,
    AxisCalibration_t *result
) {
    switch (calibration->phase) {
    case X_AXIS_CALIBRATION_PHASE_SEEK_END:
        return x_axis_calibration_poll_seek_end(calibration, config);
    case X_AXIS_CALIBRATION_PHASE_MEASURE:
        return x_axis_calibration_poll_measure(calibration, result);
    default:
        return X_AXIS_CALIBRATION_RESULT_NONE;
    }
}

void x_axis_calibration_abort(XAxisCalibration_t *calibration) {
    if (x_axis_calibration_is_active(calibration) != 0U) {
        motion_hardware_x_request_stop();
    }
    *calibration = (XAxisCalibration_t){0};
}

uint8_t x_axis_calibration_is_active(const XAxisCalibration_t *calibration) {
    return calibration->phase != X_AXIS_CALIBRATION_PHASE_IDLE ? 1U : 0U;
}

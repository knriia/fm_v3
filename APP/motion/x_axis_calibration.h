#ifndef FM_V3_X_AXIS_CALIBRATION_H
#define FM_V3_X_AXIS_CALIBRATION_H

#include "motion/axis_calibration.h"
#include "motion_hardware_x.h"

#include <stdint.h>

/* Bring-up values: tiny and slow on purpose, to be raised once the mechanics are known. */
#define X_AXIS_CALIBRATION_MEASURE_STEPS 10U
#define X_AXIS_CALIBRATION_MEASURE_STEP_RATE_HZ 10U
/* Seek toward the end switch; the switch, not the step count, ends it. */
#define X_AXIS_CALIBRATION_SEEK_INITIAL_STEP_RATE_HZ 1000U
#define X_AXIS_CALIBRATION_SEEK_STEP_RATE_HZ 50000U
#define X_AXIS_CALIBRATION_SEEK_ACCELERATION_STEPS_PER_S2 100000U

typedef enum {
    X_AXIS_CALIBRATION_PHASE_IDLE = 0,
    X_AXIS_CALIBRATION_PHASE_SEEK_END,
    X_AXIS_CALIBRATION_PHASE_MEASURE
} XAxisCalibrationPhase_t;

typedef enum {
    X_AXIS_CALIBRATION_RESULT_NONE = 0,
    X_AXIS_CALIBRATION_RESULT_RUNNING,
    X_AXIS_CALIBRATION_RESULT_OK,
    X_AXIS_CALIBRATION_RESULT_HOME_FAILED,
    X_AXIS_CALIBRATION_RESULT_END_NOT_REACHED,
    X_AXIS_CALIBRATION_RESULT_MOVE_FAILED,
    X_AXIS_CALIBRATION_RESULT_NO_DISTANCE,
    X_AXIS_CALIBRATION_RESULT_ABORTED,
    /* PF10 and PF9 are both active: wiring or sensor fault, so the axis is not moved. */
    X_AXIS_CALIBRATION_RESULT_LIMITS_CONFLICT
} XAxisCalibrationResult_t;

typedef struct {
    XAxisCalibrationPhase_t phase;
    uint32_t travel_um;
    int32_t measure_start_um;
} XAxisCalibration_t;

/* Starts at the homed start switch with the scale zeroed there: drives to the end switch to measure the travel,
 * then makes one slow measuring move back toward the start. */
XAxisCalibrationResult_t
x_axis_calibration_begin(XAxisCalibration_t *calibration, const MotionHardwareXConfig_t *config);
/* Supervises the active phase; on OK `result` holds the measured axis values. */
XAxisCalibrationResult_t x_axis_calibration_poll(
    XAxisCalibration_t *calibration,
    const MotionHardwareXConfig_t *config,
    AxisCalibration_t *result
);
void x_axis_calibration_abort(XAxisCalibration_t *calibration);
uint8_t x_axis_calibration_is_active(const XAxisCalibration_t *calibration);

#endif /* FM_V3_X_AXIS_CALIBRATION_H */

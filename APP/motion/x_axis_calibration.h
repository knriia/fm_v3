#ifndef FM_V3_X_AXIS_CALIBRATION_H
#define FM_V3_X_AXIS_CALIBRATION_H

#include "motion/axis_calibration.h"
#include "motion_hardware_x.h"

#include <stdint.h>

/* After the reversal at the end switch this move takes up the belt slack, so the measurement does not include it.
 * About 1 mm at 1/40000 and 2 mm at 1/20000 microstepping. */
#define X_AXIS_CALIBRATION_TAKE_UP_STEPS 2000U
/* The measured stretch: about 25 mm at 1/40000 and 50 mm at 1/20000, well inside the travel, so the 1 um scale
 * resolution stays far below 0.01 %. */
#define X_AXIS_CALIBRATION_MEASURE_STEPS 50000U
/* Both moves ramp up and down; the scale is read only at rest, so the ramp itself adds no error. */
#define X_AXIS_CALIBRATION_MEASURE_INITIAL_STEP_RATE_HZ 100U
#define X_AXIS_CALIBRATION_MEASURE_STEP_RATE_HZ 2000U
#define X_AXIS_CALIBRATION_MEASURE_ACCELERATION_STEPS_PER_S2 2000U
/* Seek toward the end switch; the switch, not the step count, ends it. */
#define X_AXIS_CALIBRATION_SEEK_INITIAL_STEP_RATE_HZ 1000U
#define X_AXIS_CALIBRATION_SEEK_STEP_RATE_HZ 50000U
#define X_AXIS_CALIBRATION_SEEK_ACCELERATION_STEPS_PER_S2 100000U

typedef enum {
    X_AXIS_CALIBRATION_PHASE_IDLE = 0,
    X_AXIS_CALIBRATION_PHASE_SEEK_END,
    X_AXIS_CALIBRATION_PHASE_MEASURE,
    /* Runs between SEEK_END and MEASURE; appended so the values in the diagnostics frame stay the same. */
    X_AXIS_CALIBRATION_PHASE_TAKE_UP
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
 * takes up the slack back toward the start, then measures one long move further toward the start. */
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

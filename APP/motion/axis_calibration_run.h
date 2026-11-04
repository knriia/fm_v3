#ifndef FM_V3_AXIS_CALIBRATION_RUN_H
#define FM_V3_AXIS_CALIBRATION_RUN_H

#include "motion/axis_calibration.h"
#include "motion_axis_hardware.h"

#include <stdint.h>

/* After the reversal at the end switch this move takes up the belt slack, so the measurement does not include it.
 * About 1 mm at 1/40000 and 2 mm at 1/20000 microstepping. */
#define AXIS_CALIBRATION_RUN_TAKE_UP_STEPS 2000U
/* The measured stretch: about 25 mm at 1/40000 and 50 mm at 1/20000, well inside the travel, so the 1 um scale
 * resolution stays far below 0.01 %. */
#define AXIS_CALIBRATION_RUN_MEASURE_STEPS 50000U
/* Both moves ramp up and down; the scale is read only at rest, so the ramp itself adds no error. */
#define AXIS_CALIBRATION_RUN_MEASURE_INITIAL_STEP_RATE_HZ 100U
#define AXIS_CALIBRATION_RUN_MEASURE_STEP_RATE_HZ 2000U
#define AXIS_CALIBRATION_RUN_MEASURE_ACCELERATION_STEPS_PER_S2 2000U
/* Seek toward the end switch; the switch, not the step count, ends it. */
#define AXIS_CALIBRATION_RUN_SEEK_INITIAL_STEP_RATE_HZ 1000U
#define AXIS_CALIBRATION_RUN_SEEK_STEP_RATE_HZ 50000U
#define AXIS_CALIBRATION_RUN_SEEK_ACCELERATION_STEPS_PER_S2 100000U

/* Values are sent in the diagnostics frame. */
typedef enum {
    AXIS_CALIBRATION_RUN_PHASE_IDLE = 0,
    AXIS_CALIBRATION_RUN_PHASE_SEEK_END,
    AXIS_CALIBRATION_RUN_PHASE_MEASURE,
    /* Runs between SEEK_END and MEASURE; appended so the values in the diagnostics frame stay the same. */
    AXIS_CALIBRATION_RUN_PHASE_TAKE_UP
} AxisCalibrationRunPhase_t;

/* Values are sent in the diagnostics frame. */
typedef enum {
    AXIS_CALIBRATION_RUN_RESULT_NONE = 0,
    AXIS_CALIBRATION_RUN_RESULT_RUNNING,
    AXIS_CALIBRATION_RUN_RESULT_OK,
    AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED,
    AXIS_CALIBRATION_RUN_RESULT_END_NOT_REACHED,
    AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
    AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE,
    AXIS_CALIBRATION_RUN_RESULT_ABORTED,
    /* Both switches of the axis are active: wiring or sensor fault, so the axis is not moved. */
    AXIS_CALIBRATION_RUN_RESULT_LIMITS_CONFLICT
} AxisCalibrationRunResult_t;

typedef struct {
    MotionAxis_t axis;
    AxisCalibrationRunPhase_t phase;
    uint32_t travel_um;
    int32_t measure_start_um;
} AxisCalibrationRun_t;

/* Starts at the homed start switch with the scale zeroed there: drives to the end switch to measure the travel,
 * takes up the slack back toward the start, then measures one long move further toward the start. The scale is
 * read through config->position_reader, which must not be NULL. */
AxisCalibrationRunResult_t
axis_calibration_run_begin(AxisCalibrationRun_t *run, MotionAxis_t axis, const MotionAxisHardwareConfig_t *config);
/* Supervises the active phase; on OK `result` holds the measured axis values. */
AxisCalibrationRunResult_t axis_calibration_run_poll(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    AxisCalibration_t *result
);
void axis_calibration_run_abort(AxisCalibrationRun_t *run);
uint8_t axis_calibration_run_is_active(const AxisCalibrationRun_t *run);

#endif /* FM_V3_AXIS_CALIBRATION_RUN_H */

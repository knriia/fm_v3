#ifndef FM_V3_AXIS_CALIBRATION_RUN_H
#define FM_V3_AXIS_CALIBRATION_RUN_H

#include "motion/axis_bench.h"
#include "motion/axis_calibration.h"
#include "motion_axis_hardware.h"

#include <stdint.h>

/* Speeds and lengths of the procedure; each axis has its own, since a belt and a screw differ by orders of
 * magnitude. */
typedef struct {
    /* Toward the end switch; the switch, not the step count, ends it. */
    AxisBenchProfile_t seek;
    /* Take-up and measurement; the scale is read only at rest, so a ramp adds no error. */
    AxisBenchProfile_t measure;
    /* After the reversal at the end switch this move takes up the slack, so the measurement does not include it. */
    uint32_t take_up_steps;
    /* The measured stretch; it must fit in the travel together with the take-up. */
    uint32_t measure_steps;
} AxisCalibrationRunSettings_t;

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

/* How a move of the procedure ended, as seen by the mover that drives the axis. */
typedef enum {
    AXIS_CALIBRATION_RUN_MOVE_RUNNING = 0,
    /* Every commanded step was played. */
    AXIS_CALIBRATION_RUN_MOVE_COMPLETED,
    /* The end switch stopped the move. */
    AXIS_CALIBRATION_RUN_MOVE_END_LIMIT,
    /* Any other stop: the axis did not travel as commanded. */
    AXIS_CALIBRATION_RUN_MOVE_FAILED
} AxisCalibrationRunMoveState_t;

/* Drives the moves of the procedure. An axis moved by its own jog and an axis moved by the planned step stream
 * differ only here; the procedure itself is the same. */
typedef struct {
    /* Starts a constant-profile move of `steps` steps; returns 0 when it did not start. */
    uint8_t (*start)(
        MotionAxis_t axis,
        const MotionAxisHardwareConfig_t *config,
        AxisBenchDirection_t direction,
        uint32_t steps,
        const AxisBenchProfile_t *profile
    );
    /* `steps` is the length given to start. */
    AxisCalibrationRunMoveState_t (*poll)(MotionAxis_t axis, uint32_t steps);
    /* Stops the move that is running. */
    void (*abort)(MotionAxis_t axis);
} AxisCalibrationRunMover_t;

/* Moves the axis with its own jog profile (axis_bench). */
extern const AxisCalibrationRunMover_t axis_calibration_run_jog_mover;

typedef struct {
    MotionAxis_t axis;
    /* Set by begin and used while the procedure is active. */
    const AxisCalibrationRunSettings_t *settings;
    const AxisCalibrationRunMover_t *mover;
    AxisCalibrationRunPhase_t phase;
    uint32_t travel_um;
    int32_t measure_start_um;
} AxisCalibrationRun_t;

/* Starts at the homed start switch with the scale zeroed there: drives to the end switch to measure the travel,
 * takes up the slack back toward the start, then measures one long move further toward the start. The scale is
 * read through config->position_reader, which must not be NULL; `settings` and `mover` must outlive the procedure. */
AxisCalibrationRunResult_t axis_calibration_run_begin(
    AxisCalibrationRun_t *run,
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *config,
    const AxisCalibrationRunSettings_t *settings,
    const AxisCalibrationRunMover_t *mover
);
/* Supervises the active phase; on OK `result` holds the measured axis values. */
AxisCalibrationRunResult_t axis_calibration_run_poll(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    AxisCalibration_t *result
);
void axis_calibration_run_abort(AxisCalibrationRun_t *run);
uint8_t axis_calibration_run_is_active(const AxisCalibrationRun_t *run);

#endif /* FM_V3_AXIS_CALIBRATION_RUN_H */

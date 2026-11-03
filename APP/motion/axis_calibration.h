#ifndef FM_V3_AXIS_CALIBRATION_H
#define FM_V3_AXIS_CALIBRATION_H

#include "motion/motion_planner.h"

#include <stddef.h>
#include <stdint.h>

/* Two 256-bit flash words; the encoded record is padded to this size. */
#define AXIS_CALIBRATION_RECORD_SIZE 64U

/* Measured on the machine: `steps` motor steps moved the axis by `distance_um` on the linear scale. */
typedef struct {
    uint32_t steps;
    uint32_t distance_um;
    /* Distance between the start and end limit switches measured by the linear scale. */
    uint32_t travel_um;
} AxisCalibration_t;

typedef struct {
    uint8_t calibrated_axes_mask;
    AxisCalibration_t axis[MOTION_PLANNER_AXIS_COUNT];
} MachineCalibration_t;

typedef enum {
    AXIS_CALIBRATION_STATUS_OK = 0,
    AXIS_CALIBRATION_STATUS_EMPTY,
    AXIS_CALIBRATION_STATUS_CORRUPT,
    AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT
} AxisCalibrationStatus_t;

/* An axis is usable only with all three values measured. */
uint8_t axis_calibration_is_valid(const AxisCalibration_t *calibration);

/* Builds the planner limits of one axis: steps per um from the measured ratio, travel inside the limit margins,
 * and the speed the driver step-rate limit allows for this ratio. */
AxisCalibrationStatus_t axis_calibration_to_planner_axis(
    const AxisCalibration_t *calibration,
    uint32_t max_step_rate_hz,
    MotionPlannerAxisConfig_t *axis_config
);

/* Writes a little-endian record with a CRC into a buffer of AXIS_CALIBRATION_RECORD_SIZE bytes. */
AxisCalibrationStatus_t axis_calibration_encode(const MachineCalibration_t *calibration, uint8_t *record);
/* EMPTY means erased flash; CORRUPT means a damaged or foreign record. */
AxisCalibrationStatus_t
axis_calibration_decode(const uint8_t *record, size_t length, MachineCalibration_t *calibration);

#endif /* FM_V3_AXIS_CALIBRATION_H */

#include "motion/axis_calibration.h"

#include "command_protocol.h"

#include <string.h>

/* "CAL1" in little-endian byte order. */
#define AXIS_CALIBRATION_MAGIC 0x314C4143U
#define AXIS_CALIBRATION_VERSION 1U
#define AXIS_CALIBRATION_HEADER_SIZE 12U
#define AXIS_CALIBRATION_AXIS_SIZE 12U
#define AXIS_CALIBRATION_PAYLOAD_SIZE                                                                                  \
    (AXIS_CALIBRATION_HEADER_SIZE + MOTION_PLANNER_AXIS_COUNT * AXIS_CALIBRATION_AXIS_SIZE)
#define AXIS_CALIBRATION_ERASED_BYTE 0xFFU

_Static_assert(AXIS_CALIBRATION_PAYLOAD_SIZE + 4U <= AXIS_CALIBRATION_RECORD_SIZE, "record does not fit");

static void axis_calibration_put_u32(uint8_t *buffer, uint32_t value) {
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8U);
    buffer[2] = (uint8_t)(value >> 16U);
    buffer[3] = (uint8_t)(value >> 24U);
}

static uint32_t axis_calibration_get_u32(const uint8_t *buffer) {
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8U) | ((uint32_t)buffer[2] << 16U) |
           ((uint32_t)buffer[3] << 24U);
}

uint8_t axis_calibration_is_valid(const AxisCalibration_t *calibration) {
    return calibration != NULL && calibration->steps != 0U && calibration->distance_um != 0U &&
                   calibration->travel_um != 0U
               ? 1U
               : 0U;
}

AxisCalibrationStatus_t axis_calibration_to_planner_axis(
    const AxisCalibration_t *calibration,
    uint32_t max_step_rate_hz,
    MotionPlannerAxisConfig_t *axis_config
) {
    if (axis_calibration_is_valid(calibration) == 0U || max_step_rate_hz == 0U || axis_config == NULL ||
        calibration->travel_um > (uint32_t)INT32_MAX) {
        return AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT;
    }

    /* Speed at the step-rate limit: rate * um / steps, rounded down so the limit is never exceeded. */
    const uint64_t max_speed_um_per_s = (uint64_t)max_step_rate_hz * calibration->distance_um / calibration->steps;
    if (max_speed_um_per_s == 0U) {
        return AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT;
    }

    /* Home is 0; the whole measured travel up to the end switch is usable. */
    axis_config->min_position_um = 0;
    axis_config->max_position_um = (int32_t)calibration->travel_um;
    axis_config->steps_per_um_numerator = calibration->steps;
    axis_config->steps_per_um_denominator = calibration->distance_um;
    axis_config->max_speed_um_per_s = max_speed_um_per_s > UINT32_MAX ? UINT32_MAX : (uint32_t)max_speed_um_per_s;
    return AXIS_CALIBRATION_STATUS_OK;
}

/* Layout: magic, version (u16), payload size (u16), axes mask (u8) + 3 reserved, then per axis
 * steps, distance_um and travel_um, then the CRC of everything before it. Unused bytes are zero. */
AxisCalibrationStatus_t axis_calibration_encode(const MachineCalibration_t *calibration, uint8_t *record) {
    if (calibration == NULL || record == NULL ||
        (calibration->calibrated_axes_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U) {
        return AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT;
    }
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const uint8_t marked = (calibration->calibrated_axes_mask & (uint8_t)(1U << axis)) != 0U ? 1U : 0U;
        if (marked != 0U && axis_calibration_is_valid(&calibration->axis[axis]) == 0U) {
            return AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT;
        }
    }

    (void)memset(record, 0, AXIS_CALIBRATION_RECORD_SIZE);
    axis_calibration_put_u32(&record[0], AXIS_CALIBRATION_MAGIC);
    axis_calibration_put_u32(&record[4], AXIS_CALIBRATION_VERSION | ((uint32_t)AXIS_CALIBRATION_PAYLOAD_SIZE << 16U));
    record[8] = calibration->calibrated_axes_mask;
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        uint8_t *axis_record = &record[AXIS_CALIBRATION_HEADER_SIZE + axis * AXIS_CALIBRATION_AXIS_SIZE];
        axis_calibration_put_u32(&axis_record[0], calibration->axis[axis].steps);
        axis_calibration_put_u32(&axis_record[4], calibration->axis[axis].distance_um);
        axis_calibration_put_u32(&axis_record[8], calibration->axis[axis].travel_um);
    }
    axis_calibration_put_u32(
        &record[AXIS_CALIBRATION_PAYLOAD_SIZE],
        command_protocol_crc32(record, AXIS_CALIBRATION_PAYLOAD_SIZE)
    );
    return AXIS_CALIBRATION_STATUS_OK;
}

AxisCalibrationStatus_t
axis_calibration_decode(const uint8_t *record, size_t length, MachineCalibration_t *calibration) {
    if (record == NULL || calibration == NULL || length < AXIS_CALIBRATION_RECORD_SIZE) {
        return AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT;
    }

    uint8_t erased = 1U;
    for (uint32_t index = 0U; index < AXIS_CALIBRATION_RECORD_SIZE; ++index) {
        if (record[index] != AXIS_CALIBRATION_ERASED_BYTE) {
            erased = 0U;
            break;
        }
    }
    if (erased != 0U) {
        return AXIS_CALIBRATION_STATUS_EMPTY;
    }

    const uint32_t expected_version = AXIS_CALIBRATION_VERSION | ((uint32_t)AXIS_CALIBRATION_PAYLOAD_SIZE << 16U);
    if (axis_calibration_get_u32(&record[0]) != AXIS_CALIBRATION_MAGIC ||
        axis_calibration_get_u32(&record[4]) != expected_version ||
        axis_calibration_get_u32(&record[AXIS_CALIBRATION_PAYLOAD_SIZE]) !=
            command_protocol_crc32(record, AXIS_CALIBRATION_PAYLOAD_SIZE)) {
        return AXIS_CALIBRATION_STATUS_CORRUPT;
    }

    MachineCalibration_t decoded = {0};
    decoded.calibrated_axes_mask = record[8];
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const uint8_t *axis_record = &record[AXIS_CALIBRATION_HEADER_SIZE + axis * AXIS_CALIBRATION_AXIS_SIZE];
        decoded.axis[axis].steps = axis_calibration_get_u32(&axis_record[0]);
        decoded.axis[axis].distance_um = axis_calibration_get_u32(&axis_record[4]);
        decoded.axis[axis].travel_um = axis_calibration_get_u32(&axis_record[8]);
    }
    /* A record that passed the CRC but marks an axis without its values was not written by this firmware. */
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const uint8_t marked = (decoded.calibrated_axes_mask & (uint8_t)(1U << axis)) != 0U ? 1U : 0U;
        if (marked != 0U && axis_calibration_is_valid(&decoded.axis[axis]) == 0U) {
            return AXIS_CALIBRATION_STATUS_CORRUPT;
        }
    }
    if ((decoded.calibrated_axes_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U) {
        return AXIS_CALIBRATION_STATUS_CORRUPT;
    }

    *calibration = decoded;
    return AXIS_CALIBRATION_STATUS_OK;
}

#include "motion/calibration_store.h"

#include "flash_storage.h"

#include <stddef.h>

_Static_assert(AXIS_CALIBRATION_RECORD_SIZE % FLASH_STORAGE_WORD_SIZE == 0U, "record must fill whole flash words");

static CalibrationStoreStatus_t calibration_store_status_from_decode(AxisCalibrationStatus_t status) {
    switch (status) {
    case AXIS_CALIBRATION_STATUS_OK:
        return CALIBRATION_STORE_STATUS_OK;
    case AXIS_CALIBRATION_STATUS_EMPTY:
        return CALIBRATION_STORE_STATUS_EMPTY;
    case AXIS_CALIBRATION_STATUS_CORRUPT:
        return CALIBRATION_STORE_STATUS_CORRUPT;
    default:
        return CALIBRATION_STORE_STATUS_INVALID_ARGUMENT;
    }
}

CalibrationStoreStatus_t calibration_store_load(MachineCalibration_t *calibration) {
    if (calibration == NULL) {
        return CALIBRATION_STORE_STATUS_INVALID_ARGUMENT;
    }
    return calibration_store_status_from_decode(
        axis_calibration_decode(flash_storage_settings_data(), flash_storage_settings_size(), calibration)
    );
}

CalibrationStoreStatus_t calibration_store_save(const MachineCalibration_t *calibration) {
    uint8_t record[AXIS_CALIBRATION_RECORD_SIZE];
    if (axis_calibration_encode(calibration, record) != AXIS_CALIBRATION_STATUS_OK) {
        return CALIBRATION_STORE_STATUS_INVALID_ARGUMENT;
    }
    if (flash_storage_erase_settings() != FLASH_STORAGE_STATUS_OK ||
        flash_storage_program_settings(0U, record, sizeof(record)) != FLASH_STORAGE_STATUS_OK) {
        return CALIBRATION_STORE_STATUS_FLASH_ERROR;
    }

    MachineCalibration_t stored = {0};
    if (calibration_store_load(&stored) != CALIBRATION_STORE_STATUS_OK ||
        stored.calibrated_axes_mask != calibration->calibrated_axes_mask) {
        return CALIBRATION_STORE_STATUS_VERIFY_ERROR;
    }
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        if (stored.axis[axis].steps != calibration->axis[axis].steps ||
            stored.axis[axis].distance_um != calibration->axis[axis].distance_um ||
            stored.axis[axis].travel_um != calibration->axis[axis].travel_um) {
            return CALIBRATION_STORE_STATUS_VERIFY_ERROR;
        }
    }
    return CALIBRATION_STORE_STATUS_OK;
}

#ifndef FM_V3_CALIBRATION_STORE_H
#define FM_V3_CALIBRATION_STORE_H

#include "motion/axis_calibration.h"

typedef enum {
    CALIBRATION_STORE_STATUS_OK = 0,
    CALIBRATION_STORE_STATUS_EMPTY,
    CALIBRATION_STORE_STATUS_CORRUPT,
    CALIBRATION_STORE_STATUS_INVALID_ARGUMENT,
    CALIBRATION_STORE_STATUS_FLASH_ERROR,
    CALIBRATION_STORE_STATUS_VERIFY_ERROR
} CalibrationStoreStatus_t;

/* Reads the calibration kept in the flash settings sector. */
CalibrationStoreStatus_t calibration_store_load(MachineCalibration_t *calibration);
/* Erases the settings sector and writes the record; flash access stalls the whole MCU for seconds.
 * Call only while motion is stopped. */
CalibrationStoreStatus_t calibration_store_save(const MachineCalibration_t *calibration);

#endif /* FM_V3_CALIBRATION_STORE_H */

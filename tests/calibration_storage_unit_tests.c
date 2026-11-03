#include "command_protocol.h"
#include "flash_storage.h"
#include "motion/axis_calibration.h"
#include "motion/calibration_store.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_FLASH_SIZE 256U

static uint32_t test_failures;
static uint8_t test_flash[TEST_FLASH_SIZE];
static size_t test_flash_reported_size;
static FlashStorageStatus_t test_erase_result;
static FlashStorageStatus_t test_program_result;
static uint8_t test_program_corrupts;
static uint32_t test_erase_calls;
static uint32_t test_program_calls;
static const uint8_t *test_program_substitute;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

const uint8_t *flash_storage_settings_data(void) { return test_flash; }

size_t flash_storage_settings_size(void) { return test_flash_reported_size; }

FlashStorageStatus_t flash_storage_erase_settings(void) {
    ++test_erase_calls;
    (void)memset(test_flash, 0xFF, sizeof(test_flash));
    return test_erase_result;
}

FlashStorageStatus_t flash_storage_program_settings(size_t offset, const uint8_t *data, size_t length) {
    ++test_program_calls;
    if (test_program_result == FLASH_STORAGE_STATUS_OK) {
        (void)memcpy(&test_flash[offset], test_program_substitute != NULL ? test_program_substitute : data, length);
        if (test_program_corrupts != 0U) {
            test_flash[offset + 12U] ^= 1U;
        }
    }
    return test_program_result;
}

static void reset_flash(void) {
    (void)memset(test_flash, 0xFF, sizeof(test_flash));
    test_flash_reported_size = TEST_FLASH_SIZE;
    test_erase_result = FLASH_STORAGE_STATUS_OK;
    test_program_result = FLASH_STORAGE_STATUS_OK;
    test_program_corrupts = 0U;
    test_erase_calls = 0U;
    test_program_calls = 0U;
    test_program_substitute = NULL;
}

static MachineCalibration_t make_x_calibration(void) {
    MachineCalibration_t calibration = {0};
    calibration.calibrated_axes_mask = MOTION_PLANNER_AXIS_X_MASK;
    calibration.axis[0] = (AxisCalibration_t){.steps = 100000U, .distance_um = 49873U, .travel_um = 301250U};
    return calibration;
}

/* Rewrites the CRC so a deliberately edited record still passes the integrity check. */
static void reseal(uint8_t *record) {
    const size_t payload_size = 12U + MOTION_PLANNER_AXIS_COUNT * 12U;
    const uint32_t crc = command_protocol_crc32(record, payload_size);
    record[payload_size] = (uint8_t)crc;
    record[payload_size + 1U] = (uint8_t)(crc >> 8U);
    record[payload_size + 2U] = (uint8_t)(crc >> 16U);
    record[payload_size + 3U] = (uint8_t)(crc >> 24U);
}

static void test_validity(void) {
    AxisCalibration_t calibration = {.steps = 1U, .distance_um = 1U, .travel_um = 1U};
    expect_u32(axis_calibration_is_valid(&calibration), 1U, "measured axis is valid");
    expect_u32(axis_calibration_is_valid(NULL), 0U, "missing axis is not valid");
    calibration.steps = 0U;
    expect_u32(axis_calibration_is_valid(&calibration), 0U, "axis without steps is not valid");
    calibration = (AxisCalibration_t){.steps = 1U, .distance_um = 0U, .travel_um = 1U};
    expect_u32(axis_calibration_is_valid(&calibration), 0U, "axis without distance is not valid");
    calibration = (AxisCalibration_t){.steps = 1U, .distance_um = 1U, .travel_um = 0U};
    expect_u32(axis_calibration_is_valid(&calibration), 0U, "axis without travel is not valid");
}

static void test_encode_decode(void) {
    uint8_t record[AXIS_CALIBRATION_RECORD_SIZE];
    MachineCalibration_t calibration = make_x_calibration();
    MachineCalibration_t decoded = {0};

    expect_u32(axis_calibration_encode(&calibration, record), AXIS_CALIBRATION_STATUS_OK, "calibration encodes");
    expect_u32(
        axis_calibration_decode(record, sizeof(record), &decoded),
        AXIS_CALIBRATION_STATUS_OK,
        "encoded calibration decodes"
    );
    expect_u32(decoded.calibrated_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "calibrated axes survive the record");
    expect_u32(decoded.axis[0].steps, 100000U, "steps survive the record");
    expect_u32(decoded.axis[0].distance_um, 49873U, "distance survives the record");
    expect_u32(decoded.axis[0].travel_um, 301250U, "travel survives the record");

    expect_u32(axis_calibration_encode(NULL, record), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "missing input");
    expect_u32(axis_calibration_encode(&calibration, NULL), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "missing record");
    calibration.calibrated_axes_mask = 0x08U;
    expect_u32(axis_calibration_encode(&calibration, record), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "unknown axis");
    calibration = make_x_calibration();
    calibration.calibrated_axes_mask |= MOTION_PLANNER_AXIS_Y_MASK;
    expect_u32(
        axis_calibration_encode(&calibration, record),
        AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT,
        "axis marked calibrated without values is rejected"
    );

    expect_u32(axis_calibration_decode(NULL, sizeof(record), &decoded), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "no record");
    expect_u32(axis_calibration_decode(record, sizeof(record), NULL), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "no output");
    expect_u32(axis_calibration_decode(record, 8U, &decoded), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "short record");

    uint8_t erased[AXIS_CALIBRATION_RECORD_SIZE];
    (void)memset(erased, 0xFF, sizeof(erased));
    expect_u32(axis_calibration_decode(erased, sizeof(erased), &decoded), AXIS_CALIBRATION_STATUS_EMPTY, "erased flash");

    calibration = make_x_calibration();
    (void)axis_calibration_encode(&calibration, record);
    record[0] ^= 1U;
    reseal(record);
    expect_u32(axis_calibration_decode(record, sizeof(record), &decoded), AXIS_CALIBRATION_STATUS_CORRUPT, "bad magic");

    (void)axis_calibration_encode(&calibration, record);
    record[4] ^= 1U;
    reseal(record);
    expect_u32(axis_calibration_decode(record, sizeof(record), &decoded), AXIS_CALIBRATION_STATUS_CORRUPT, "bad version");

    (void)axis_calibration_encode(&calibration, record);
    record[12] ^= 1U;
    expect_u32(axis_calibration_decode(record, sizeof(record), &decoded), AXIS_CALIBRATION_STATUS_CORRUPT, "bad CRC");

    (void)axis_calibration_encode(&calibration, record);
    record[8] |= MOTION_PLANNER_AXIS_Y_MASK;
    reseal(record);
    expect_u32(
        axis_calibration_decode(record, sizeof(record), &decoded),
        AXIS_CALIBRATION_STATUS_CORRUPT,
        "axis marked without values is corrupt"
    );

    (void)axis_calibration_encode(&calibration, record);
    record[8] |= 0x80U;
    reseal(record);
    expect_u32(axis_calibration_decode(record, sizeof(record), &decoded), AXIS_CALIBRATION_STATUS_CORRUPT, "unknown axis bit");
}

static void test_store(void) {
    MachineCalibration_t calibration = make_x_calibration();
    MachineCalibration_t loaded = {0};

    reset_flash();
    expect_u32(calibration_store_load(&loaded), CALIBRATION_STORE_STATUS_EMPTY, "erased sector has no calibration");
    expect_u32(calibration_store_load(NULL), CALIBRATION_STORE_STATUS_INVALID_ARGUMENT, "missing output");

    expect_u32(calibration_store_save(&calibration), CALIBRATION_STORE_STATUS_OK, "calibration is saved");
    expect_u32(test_erase_calls, 1U, "sector is erased before writing");
    expect_u32(test_program_calls, 1U, "record is written once");
    expect_u32(calibration_store_load(&loaded), CALIBRATION_STORE_STATUS_OK, "saved calibration loads");
    expect_u32(loaded.axis[0].distance_um, 49873U, "loaded values match the saved ones");

    test_flash[0] ^= 1U;
    expect_u32(calibration_store_load(&loaded), CALIBRATION_STORE_STATUS_CORRUPT, "damaged sector is reported");

    reset_flash();
    test_flash_reported_size = 8U;
    expect_u32(calibration_store_load(&loaded), CALIBRATION_STORE_STATUS_INVALID_ARGUMENT, "too small sector");

    reset_flash();
    MachineCalibration_t invalid = make_x_calibration();
    invalid.axis[0].steps = 0U;
    expect_u32(calibration_store_save(&invalid), CALIBRATION_STORE_STATUS_INVALID_ARGUMENT, "invalid data is not saved");
    expect_u32(test_erase_calls, 0U, "invalid data leaves flash untouched");

    reset_flash();
    test_erase_result = FLASH_STORAGE_STATUS_ERROR;
    expect_u32(calibration_store_save(&calibration), CALIBRATION_STORE_STATUS_FLASH_ERROR, "erase failure");
    expect_u32(test_program_calls, 0U, "nothing is written after a failed erase");

    reset_flash();
    test_program_result = FLASH_STORAGE_STATUS_ERROR;
    expect_u32(calibration_store_save(&calibration), CALIBRATION_STORE_STATUS_FLASH_ERROR, "program failure");

    reset_flash();
    test_program_corrupts = 1U;
    expect_u32(calibration_store_save(&calibration), CALIBRATION_STORE_STATUS_VERIFY_ERROR, "unreadable record fails");
}

/* Flash that accepts the write but returns another valid record must fail the read-back check. */
static void test_store_detects_mismatch(void) {
    const MachineCalibration_t calibration = make_x_calibration();
    uint8_t record[AXIS_CALIBRATION_RECORD_SIZE];

    for (uint32_t variant = 0U; variant < 4U; ++variant) {
        MachineCalibration_t other = make_x_calibration();
        if (variant == 0U) {
            other.calibrated_axes_mask = 0U;
        } else if (variant == 1U) {
            other.axis[0].steps += 1U;
        } else if (variant == 2U) {
            other.axis[0].distance_um += 1U;
        } else {
            other.axis[0].travel_um += 1U;
        }
        reset_flash();
        (void)axis_calibration_encode(&other, record);
        test_program_substitute = record;
        expect_u32(calibration_store_save(&calibration), CALIBRATION_STORE_STATUS_VERIFY_ERROR, "read-back mismatch");
    }
}

static void test_planner_axis_config(void) {
    const AxisCalibration_t calibration = {.steps = 100000U, .distance_um = 50000U, .travel_um = 301000U};
    MotionPlannerAxisConfig_t config = {0};

    expect_u32(
        axis_calibration_to_planner_axis(&calibration, 150000U, &config),
        AXIS_CALIBRATION_STATUS_OK,
        "calibration converts to planner limits"
    );
    expect_u32((uint32_t)config.min_position_um, 0U, "travel starts at home");
    expect_u32((uint32_t)config.max_position_um, 301000U, "travel ends at the measured length");
    expect_u32(config.steps_per_um_numerator, 100000U, "steps come from the measurement");
    expect_u32(config.steps_per_um_denominator, 50000U, "micrometres come from the scale");
    expect_u32(config.max_speed_um_per_s, 75000U, "speed limit follows from the step-rate limit");

    expect_u32(axis_calibration_to_planner_axis(NULL, 150000U, &config), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "no data");
    expect_u32(axis_calibration_to_planner_axis(&calibration, 0U, &config), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "no rate");
    expect_u32(axis_calibration_to_planner_axis(&calibration, 150000U, NULL), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "no output");

    AxisCalibration_t no_travel = calibration;
    no_travel.travel_um = 0U;
    expect_u32(axis_calibration_to_planner_axis(&no_travel, 150000U, &config), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "no travel");
    AxisCalibration_t huge_travel = calibration;
    huge_travel.travel_um = (uint32_t)INT32_MAX + 1U;
    expect_u32(axis_calibration_to_planner_axis(&huge_travel, 150000U, &config), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "travel beyond int32");
    AxisCalibration_t fine = calibration;
    fine.steps = 1000000U;
    fine.distance_um = 1U;
    expect_u32(axis_calibration_to_planner_axis(&fine, 1U, &config), AXIS_CALIBRATION_STATUS_INVALID_ARGUMENT, "speed rounds to zero");
    AxisCalibration_t coarse = calibration;
    coarse.steps = 1U;
    coarse.distance_um = UINT32_MAX;
    expect_u32(axis_calibration_to_planner_axis(&coarse, 150000U, &config), AXIS_CALIBRATION_STATUS_OK, "huge speed is capped");
    expect_u32(config.max_speed_um_per_s, UINT32_MAX, "speed limit saturates");
}

int main(void) {
    test_validity();
    test_planner_axis_config();
    test_encode_decode();
    test_store();
    test_store_detects_mismatch();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Calibration storage unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Calibration storage unit tests passed.\n");
    return 0;
}

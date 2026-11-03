#ifndef FM_V3_FLASH_STORAGE_H
#define FM_V3_FLASH_STORAGE_H

#include <stddef.h>
#include <stdint.h>

/* The flash is programmed in 256-bit flash words; offsets and lengths must be multiples of this. */
#define FLASH_STORAGE_WORD_SIZE 32U

typedef enum {
    FLASH_STORAGE_STATUS_OK = 0,
    FLASH_STORAGE_STATUS_INVALID_ARGUMENT,
    FLASH_STORAGE_STATUS_LOCKED,
    FLASH_STORAGE_STATUS_ERROR
} FlashStorageStatus_t;

/* Read-only view of the settings sector reserved in the linker script. */
const uint8_t *flash_storage_settings_data(void);
size_t flash_storage_settings_size(void);

/* Erase and program stall every flash read, including code fetches and interrupt vectors, until they finish;
 * a sector erase takes seconds. Call only while motion is stopped and from one task at a time. */
FlashStorageStatus_t flash_storage_erase_settings(void);
FlashStorageStatus_t flash_storage_program_settings(size_t offset, const uint8_t *data, size_t length);

#endif /* FM_V3_FLASH_STORAGE_H */

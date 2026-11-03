#include "flash_storage.h"

#include "stm32h7xx.h"

/* Unlock keys and the erase/program sequence follow RM0468, flash bank 1 registers. */
#define FLASH_STORAGE_KEY1 0x45670123U
#define FLASH_STORAGE_KEY2 0xCDEF89ABU
#define FLASH_STORAGE_ERROR_FLAGS                                                                                      \
    (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR |       \
        FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR)

/* Provided by the linker script. */
extern const uint8_t _settings_flash_start[];
extern const uint8_t _settings_flash_size[];
extern const uint8_t _settings_flash_sector[];

const uint8_t *flash_storage_settings_data(void) { return _settings_flash_start; }

size_t flash_storage_settings_size(void) { return (size_t)(uintptr_t)_settings_flash_size; }

static FlashStorageStatus_t flash_storage_unlock(void) {
    if ((FLASH->CR1 & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR1 = FLASH_STORAGE_KEY1;
        FLASH->KEYR1 = FLASH_STORAGE_KEY2;
    }
    return (FLASH->CR1 & FLASH_CR_LOCK) == 0U ? FLASH_STORAGE_STATUS_OK : FLASH_STORAGE_STATUS_LOCKED;
}

static void flash_storage_lock(void) { FLASH->CR1 |= FLASH_CR_LOCK; }

/* QW drops when the operation ends, successful or not; the error flags tell which. */
static FlashStorageStatus_t flash_storage_wait(void) {
    while ((FLASH->SR1 & FLASH_SR_QW) != 0U) {
    }
    const uint32_t errors = FLASH->SR1 & FLASH_STORAGE_ERROR_FLAGS;
    FLASH->CCR1 = errors | FLASH_CCR_CLR_EOP;
    return errors == 0U ? FLASH_STORAGE_STATUS_OK : FLASH_STORAGE_STATUS_ERROR;
}

/* The CPU may hold old sector contents in its data cache. */
static void flash_storage_invalidate_cache(const uint8_t *address, size_t length) {
    SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)address, (int32_t)length);
}

FlashStorageStatus_t flash_storage_erase_settings(void) {
    FlashStorageStatus_t status = flash_storage_unlock();
    if (status != FLASH_STORAGE_STATUS_OK) {
        return status;
    }
    status = flash_storage_wait();
    if (status == FLASH_STORAGE_STATUS_OK) {
        const uint32_t sector = (uint32_t)(uintptr_t)_settings_flash_sector;
        FLASH->CR1 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
        FLASH->CR1 |= FLASH_CR_SER | FLASH_CR_PSIZE_1 | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_START;
        status = flash_storage_wait();
        FLASH->CR1 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    }
    flash_storage_lock();
    flash_storage_invalidate_cache(flash_storage_settings_data(), flash_storage_settings_size());
    return status;
}

FlashStorageStatus_t flash_storage_program_settings(size_t offset, const uint8_t *data, size_t length) {
    const size_t size = flash_storage_settings_size();
    if (data == NULL || length == 0U || (offset % FLASH_STORAGE_WORD_SIZE) != 0U ||
        (length % FLASH_STORAGE_WORD_SIZE) != 0U || offset > size || length > size - offset) {
        return FLASH_STORAGE_STATUS_INVALID_ARGUMENT;
    }

    FlashStorageStatus_t status = flash_storage_unlock();
    if (status != FLASH_STORAGE_STATUS_OK) {
        return status;
    }
    status = flash_storage_wait();
    volatile uint32_t *destination = (volatile uint32_t *)(uintptr_t)(flash_storage_settings_data() + offset);
    for (
        size_t written = 0U; status == FLASH_STORAGE_STATUS_OK && written < length; written += FLASH_STORAGE_WORD_SIZE
    ) {
        FLASH->CR1 |= FLASH_CR_PG;
        __ISB();
        __DSB();
        /* One flash word is eight 32-bit writes in a row; the source may be unaligned, so it is read bytewise. */
        for (uint32_t word = 0U; word < FLASH_STORAGE_WORD_SIZE / 4U; ++word) {
            const uint8_t *source = &data[written + word * 4U];
            *destination = (uint32_t)source[0] | ((uint32_t)source[1] << 8U) | ((uint32_t)source[2] << 16U) |
                           ((uint32_t)source[3] << 24U);
            ++destination;
        }
        __ISB();
        __DSB();
        status = flash_storage_wait();
        FLASH->CR1 &= ~FLASH_CR_PG;
    }
    flash_storage_lock();
    flash_storage_invalidate_cache(flash_storage_settings_data() + offset, length);
    return status;
}

#include "step_stream_brake.h"

#include <stddef.h>

/* 2 * deceleration * k is at most 2 * 2^32 * 2^16 = 2^49, so the squared rate fits 64 bits. */
#define STEP_STREAM_BRAKE_MAX_CAPACITY 65536U

static uint64_t step_stream_brake_isqrt(uint64_t value) {
    uint64_t root = 0U;
    uint64_t bit = 1ULL << 62;
    while (bit > value) {
        bit >>= 2U;
    }
    while (bit != 0U) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1U) + bit;
        } else {
            root >>= 1U;
        }
        bit >>= 2U;
    }
    return root;
}

static uint8_t step_stream_brake_config_valid(const StepStreamBrakeConfig_t *config) {
    return config->timer_clock_hz != 0U && config->pulse_ticks != 0U &&
                   config->min_period_ticks > config->pulse_ticks &&
                   config->min_period_ticks <= STEP_STREAM_BRAKE_MAX_PERIOD_TICKS && config->idle_record_ticks != 0U &&
                   config->idle_record_ticks <= STEP_STREAM_BRAKE_MAX_PERIOD_TICKS &&
                   config->deceleration_steps_per_s2 != 0U && config->max_step_rate_hz != 0U
               ? 1U
               : 0U;
}

StepStreamBrakeStatus_t step_stream_brake_init(
    StepStreamBrake_t *brake,
    const StepStreamBrakeConfig_t *config,
    uint16_t *storage,
    uint32_t capacity
) {
    if (brake == NULL || config == NULL || storage == NULL || capacity == 0U ||
        capacity > STEP_STREAM_BRAKE_MAX_CAPACITY || step_stream_brake_config_valid(config) == 0U) {
        return STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT;
    }

    /* The slowest rate a record can hold: its period is 65536 ticks. The ramp ends there and the axis stops. */
    const uint64_t end_rate_hz = ((uint64_t)config->timer_clock_hz + STEP_STREAM_BRAKE_MAX_PERIOD_TICKS - 1U) /
                                 STEP_STREAM_BRAKE_MAX_PERIOD_TICKS;
    const uint64_t end_rate_squared = end_rate_hz * end_rate_hz;

    /* Built from the slow end, so entry k is k steps before the stop; reversed below into play order. */
    uint32_t count = 0U;
    for (;;) {
        const uint64_t rate_squared = end_rate_squared + 2ULL * config->deceleration_steps_per_s2 * count;
        const uint64_t rate_hz = step_stream_brake_isqrt(rate_squared);
        if (rate_hz > config->max_step_rate_hz) {
            break;
        }
        /* rate_hz is at least end_rate_hz, so this never exceeds 65536 ticks. */
        const uint64_t ticks = config->timer_clock_hz / rate_hz;
        if (ticks < config->min_period_ticks) {
            break;
        }
        if (count >= capacity) {
            return STEP_STREAM_BRAKE_STATUS_CAPACITY;
        }
        storage[count] = (uint16_t)(ticks - 1U);
        ++count;
    }
    if (count == 0U) {
        return STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT;
    }

    for (uint32_t low = 0U, high = count - 1U; low < high; ++low, --high) {
        const uint16_t swap = storage[low];
        storage[low] = storage[high];
        storage[high] = swap;
    }

    brake->config = *config;
    brake->arr = storage;
    brake->count = count;
    return STEP_STREAM_BRAKE_STATUS_OK;
}

uint32_t step_stream_brake_find_start(const StepStreamBrake_t *brake, uint16_t arr) {
    if (brake == NULL || brake->arr == NULL) {
        return 0U;
    }
    uint32_t low = 0U;
    uint32_t high = brake->count;
    while (low < high) {
        const uint32_t middle = low + (high - low) / 2U;
        if (brake->arr[middle] < arr) {
            low = middle + 1U;
        } else {
            high = middle;
        }
    }
    return low;
}

void step_stream_brake_cursor_init(
    StepStreamBrakeCursor_t *cursor,
    const StepStreamBrake_t *brake,
    uint32_t start_index
) {
    if (cursor == NULL) {
        return;
    }
    cursor->brake = brake;
    cursor->next = 0U;
    if (brake != NULL) {
        cursor->next = start_index > brake->count ? brake->count : start_index;
    }
}

uint32_t step_stream_brake_fill(StepStreamBrakeCursor_t *cursor, uint16_t *records, uint32_t record_count) {
    if (cursor == NULL || cursor->brake == NULL || records == NULL) {
        return 0U;
    }
    const StepStreamBrake_t *brake = cursor->brake;
    uint32_t steps = 0U;
    for (uint32_t index = 0U; index < record_count; ++index) {
        uint16_t *record = &records[(size_t)index * STEP_STREAM_BRAKE_RECORD_WORDS];
        record[1] = 0U;
        if (cursor->next < brake->count) {
            record[0] = brake->arr[cursor->next];
            record[2] = (uint16_t)brake->config.pulse_ticks;
            ++cursor->next;
            ++steps;
        } else {
            record[0] = (uint16_t)(brake->config.idle_record_ticks - 1U);
            record[2] = 0U;
        }
    }
    return steps;
}

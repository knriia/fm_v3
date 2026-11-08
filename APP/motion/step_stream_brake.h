#ifndef FM_V3_STEP_STREAM_BRAKE_H
#define FM_V3_STEP_STREAM_BRAKE_H

#include "step_stream_format.h"

#include <stdint.h>

/* Braking ramp of one stream axis, as ring records. Pure logic: it knows the record format but not the ring or the
 * registers. The ramp holds the constant deceleration a: the step rate v at k steps before the end is
 * sqrt(v_end^2 + 2 * a * k), where v_end is the slowest rate one record can still hold (period 65536 ticks). */

#define STEP_STREAM_BRAKE_MAX_PERIOD_TICKS 65536U
#define STEP_STREAM_BRAKE_RECORD_WORDS STEP_STREAM_FORMAT_DMA_BURST_LENGTH

typedef struct {
    uint32_t timer_clock_hz;
    /* CCR of every step record. */
    uint32_t pulse_ticks;
    /* Shortest period of a step record; the caller passes at least pulse + low time. */
    uint32_t min_period_ticks;
    /* Period of the filler record that follows the ramp (no pulse). */
    uint32_t idle_record_ticks;
    uint32_t deceleration_steps_per_s2;
    /* The ramp starts at this rate; a faster axis is brought down to it at once. */
    uint32_t max_step_rate_hz;
} StepStreamBrakeConfig_t;

typedef enum {
    STEP_STREAM_BRAKE_STATUS_OK = 0,
    STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT,
    /* The ramp from max_step_rate_hz needs more entries than the storage holds. */
    STEP_STREAM_BRAKE_STATUS_CAPACITY
} StepStreamBrakeStatus_t;

typedef struct {
    StepStreamBrakeConfig_t config;
    /* ARR values in play order: the fastest step first, the slowest last; never decreasing. */
    uint16_t *arr;
    uint32_t count;
} StepStreamBrake_t;

/* Builds the ramp into storage (capacity entries). The storage must outlive the table. */
StepStreamBrakeStatus_t step_stream_brake_init(
    StepStreamBrake_t *brake,
    const StepStreamBrakeConfig_t *config,
    uint16_t *storage,
    uint32_t capacity
);

/* Index of the first entry whose period is not shorter than arr + 1, so the ramp continues from a speed that is not
 * above the current one. count when the current step is already slower than the whole ramp (nothing to brake). */
uint32_t step_stream_brake_find_start(const StepStreamBrake_t *brake, uint16_t arr);

/* A position in the ramp; the ring halves are filled from it one after another. */
typedef struct {
    const StepStreamBrake_t *brake;
    uint32_t next;
} StepStreamBrakeCursor_t;

void step_stream_brake_cursor_init(
    StepStreamBrakeCursor_t *cursor,
    const StepStreamBrake_t *brake,
    uint32_t start_index
);

/* Writes record_count records: the rest of the ramp first, then filler records. Returns how many step records of the
 * ramp it wrote, so 0 means that the stretch carries no step. */
uint32_t step_stream_brake_fill(StepStreamBrakeCursor_t *cursor, uint16_t *records, uint32_t record_count);

#endif /* FM_V3_STEP_STREAM_BRAKE_H */

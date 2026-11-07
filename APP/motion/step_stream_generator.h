#ifndef FM_V3_STEP_STREAM_GENERATOR_H
#define FM_V3_STEP_STREAM_GENERATOR_H

#include "motion/motion_planner.h"
#include "step_stream_format.h"

#include <stddef.h>
#include <stdint.h>

#define STEP_STREAM_GENERATOR_RECORD_WORDS STEP_STREAM_FORMAT_DMA_BURST_LENGTH
#define STEP_STREAM_GENERATOR_MAX_RECORD_TICKS 65536U
#define STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND 1000000000ULL

typedef struct {
    uint32_t timer_clock_hz;
    uint32_t pulse_ticks;
    uint32_t low_ticks;
    uint32_t min_record_ticks;
} StepStreamGeneratorConfig_t;

typedef struct {
    uint32_t ns_to_ticks_remainder;
} StepStreamGeneratorCarry_t;

typedef enum {
    STEP_STREAM_GENERATOR_STATUS_OK = 0,
    STEP_STREAM_GENERATOR_STATUS_DONE,
    STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT,
    STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW,
    STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT,
    STEP_STREAM_GENERATOR_STATUS_IDLE_INTERVAL_TOO_SHORT
} StepStreamGeneratorStatus_t;

typedef struct {
    StepStreamGeneratorConfig_t config;
    uint64_t step_count;
    uint64_t duration_ns;
    uint64_t step_base_ns;
    uint64_t step_remainder_ns;
    uint64_t grid_remainder_ns;
    uint64_t next_interval;
    uint64_t total_intervals;
    uint64_t current_record_count;
    uint64_t current_record_index;
    uint64_t current_extra_records;
    uint32_t current_base_ticks;
    uint32_t tick_remainder;
    uint8_t initialized;
} StepStreamGenerator_t;

StepStreamGeneratorStatus_t step_stream_generator_init(
    StepStreamGenerator_t *generator,
    const StepStreamGeneratorConfig_t *config,
    uint64_t step_count,
    uint64_t duration_ns,
    const StepStreamGeneratorCarry_t *input_carry
);

StepStreamGeneratorStatus_t step_stream_generator_init_from_block(
    StepStreamGenerator_t *generator,
    const StepStreamGeneratorConfig_t *config,
    const MotionPlanBlock_t *block,
    uint32_t axis_index,
    const StepStreamGeneratorCarry_t *input_carry
);

StepStreamGeneratorStatus_t
step_stream_generator_fill(StepStreamGenerator_t *generator, uint16_t *records, size_t max_records, size_t *written);

StepStreamGeneratorStatus_t
step_stream_generator_get_carry(const StepStreamGenerator_t *generator, StepStreamGeneratorCarry_t *carry);

#endif /* FM_V3_STEP_STREAM_GENERATOR_H */

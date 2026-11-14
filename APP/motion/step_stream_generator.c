#include "motion/step_stream_generator.h"

#include <limits.h>
#include <stddef.h>

static uint64_t step_stream_generator_abs_i64(int64_t value) {
    return value < 0 ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
}

static uint32_t step_stream_generator_max_record_ticks(const StepStreamGeneratorConfig_t *config) {
    return config->max_record_ticks != 0U ? config->max_record_ticks : STEP_STREAM_GENERATOR_MAX_RECORD_TICKS;
}

static StepStreamGeneratorStatus_t step_stream_generator_validate_config(const StepStreamGeneratorConfig_t *config) {
    if (config->max_record_ticks > STEP_STREAM_GENERATOR_MAX_RECORD_TICKS) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    const uint32_t max_ticks = step_stream_generator_max_record_ticks(config);
    if (config->timer_clock_hz == 0U || config->pulse_ticks == 0U || config->pulse_ticks >= max_ticks ||
        config->min_record_ticks == 0U || config->min_record_ticks > max_ticks) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    /* An interval just over the limit is split in two records of about half of it; each must still be a valid record,
     * and the first of a step must be longer than the pulse. */
    if (config->max_record_ticks != 0U &&
        ((uint64_t)config->min_record_ticks * 2U > max_ticks || (uint64_t)config->pulse_ticks * 2U >= max_ticks)) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    const uint64_t pulse_and_low = (uint64_t)config->pulse_ticks + config->low_ticks;
    if (pulse_and_low > max_ticks) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    return STEP_STREAM_GENERATOR_STATUS_OK;
}

static StepStreamGeneratorStatus_t
step_stream_generator_calculate_duration(const MotionPlanBlock_t *block, uint64_t *duration_ns) {
    if (block->master_step_count == 0U || block->step_event_rate_numerator == 0U ||
        block->step_event_rate_denominator == 0U) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    if (block->step_event_rate_denominator > UINT64_MAX / STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND) {
        return STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW;
    }

    const uint64_t interval_numerator =
        block->step_event_rate_denominator * STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND;
    if (block->master_step_count > UINT64_MAX / interval_numerator) {
        return STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW;
    }

    const uint64_t duration_numerator = block->master_step_count * interval_numerator;
    *duration_ns = duration_numerator / block->step_event_rate_numerator;
    if (duration_numerator % block->step_event_rate_numerator != 0U) {
        ++(*duration_ns);
    }
    return STEP_STREAM_GENERATOR_STATUS_OK;
}

static StepStreamGeneratorStatus_t step_stream_generator_convert_interval(
    const StepStreamGenerator_t *generator,
    uint64_t interval_ns,
    uint64_t *interval_ticks,
    uint32_t *next_tick_remainder
) {
    if (interval_ns > (UINT64_MAX - generator->tick_remainder) / generator->config.timer_clock_hz) {
        return STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW;
    }

    const uint64_t numerator = interval_ns * generator->config.timer_clock_hz + generator->tick_remainder;
    *interval_ticks = numerator / STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND;
    *next_tick_remainder = (uint32_t)(numerator % STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND);
    return STEP_STREAM_GENERATOR_STATUS_OK;
}

static StepStreamGeneratorStatus_t step_stream_generator_prepare_interval(StepStreamGenerator_t *generator) {
    uint64_t interval_ns;
    uint64_t next_grid_remainder = generator->grid_remainder_ns;

    if (generator->step_count == 0U) {
        interval_ns = generator->duration_ns;
    } else {
        interval_ns = generator->step_base_ns;
        if (generator->step_remainder_ns != 0U) {
            const uint64_t threshold = generator->step_count - generator->step_remainder_ns;
            if (generator->grid_remainder_ns >= threshold) {
                next_grid_remainder = generator->grid_remainder_ns - threshold;
                ++interval_ns;
            } else {
                next_grid_remainder = generator->grid_remainder_ns + generator->step_remainder_ns;
            }
        }

        if (next_grid_remainder != 0U && generator->grid_remainder_ns == 0U) {
            ++interval_ns;
        } else if (next_grid_remainder == 0U && generator->grid_remainder_ns != 0U) {
            --interval_ns;
        }
    }

    uint64_t interval_ticks;
    uint32_t next_tick_remainder;
    const StepStreamGeneratorStatus_t conversion_status =
        step_stream_generator_convert_interval(generator, interval_ns, &interval_ticks, &next_tick_remainder);
    if (conversion_status != STEP_STREAM_GENERATOR_STATUS_OK) {
        return conversion_status;
    }

    const uint64_t max_ticks = step_stream_generator_max_record_ticks(&generator->config);
    if (interval_ticks == 0U) {
        return generator->step_count == 0U ? STEP_STREAM_GENERATOR_STATUS_IDLE_INTERVAL_TOO_SHORT
                                           : STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT;
    }

    const uint64_t record_count = interval_ticks / max_ticks + (interval_ticks % max_ticks != 0U ? 1U : 0U);
    const uint64_t base_ticks = interval_ticks / record_count;
    const uint64_t extra_records = interval_ticks % record_count;
    const uint32_t minimum_record_ticks = generator->config.min_record_ticks;
    const uint64_t minimum_step_ticks =
        generator->config.min_record_ticks > (uint64_t)generator->config.pulse_ticks + generator->config.low_ticks
            ? generator->config.min_record_ticks
            : (uint64_t)generator->config.pulse_ticks + generator->config.low_ticks;

    if (base_ticks < minimum_record_ticks) {
        return generator->step_count == 0U ? STEP_STREAM_GENERATOR_STATUS_IDLE_INTERVAL_TOO_SHORT
                                           : STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT;
    }
    if (generator->step_count != 0U && interval_ticks < minimum_step_ticks) {
        return STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT;
    }
    if (generator->step_count != 0U && base_ticks + (extra_records != 0U ? 1U : 0U) <= generator->config.pulse_ticks) {
        return STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT;
    }

    generator->grid_remainder_ns = next_grid_remainder;
    generator->tick_remainder = next_tick_remainder;
    generator->current_record_count = record_count;
    generator->current_record_index = 0U;
    generator->current_extra_records = extra_records;
    generator->current_base_ticks = (uint32_t)base_ticks;
    ++generator->next_interval;
    return STEP_STREAM_GENERATOR_STATUS_OK;
}

StepStreamGeneratorStatus_t step_stream_generator_init(
    StepStreamGenerator_t *generator,
    const StepStreamGeneratorConfig_t *config,
    uint64_t step_count,
    uint64_t duration_ns,
    const StepStreamGeneratorCarry_t *input_carry
) {
    if (generator == NULL || config == NULL || duration_ns == 0U) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    const StepStreamGeneratorStatus_t config_status = step_stream_generator_validate_config(config);
    if (config_status != STEP_STREAM_GENERATOR_STATUS_OK) {
        return config_status;
    }
    if (input_carry != NULL && input_carry->ns_to_ticks_remainder >= STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    StepStreamGenerator_t next_generator = {0};
    next_generator.config = *config;
    next_generator.step_count = step_count;
    next_generator.duration_ns = duration_ns;
    next_generator.step_base_ns = step_count == 0U ? 0U : duration_ns / step_count;
    next_generator.step_remainder_ns = step_count == 0U ? 0U : duration_ns % step_count;
    next_generator.total_intervals = step_count == 0U ? 1U : step_count;
    next_generator.tick_remainder = input_carry == NULL ? 0U : input_carry->ns_to_ticks_remainder;
    next_generator.initialized = 1U;
    *generator = next_generator;
    return STEP_STREAM_GENERATOR_STATUS_OK;
}

StepStreamGeneratorStatus_t step_stream_generator_init_from_block(
    StepStreamGenerator_t *generator,
    const StepStreamGeneratorConfig_t *config,
    const MotionPlanBlock_t *block,
    uint32_t axis_index,
    const StepStreamGeneratorCarry_t *input_carry
) {
    if (generator == NULL || config == NULL || block == NULL || axis_index >= MOTION_PLANNER_AXIS_COUNT) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    const uint64_t step_count = step_stream_generator_abs_i64(block->step_delta[axis_index]);
    if (step_count > block->master_step_count) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    uint64_t duration_ns;
    const StepStreamGeneratorStatus_t duration_status = step_stream_generator_calculate_duration(block, &duration_ns);
    if (duration_status != STEP_STREAM_GENERATOR_STATUS_OK) {
        return duration_status;
    }

    return step_stream_generator_init(generator, config, step_count, duration_ns, input_carry);
}

StepStreamGeneratorStatus_t
step_stream_generator_fill(StepStreamGenerator_t *generator, uint16_t *records, size_t max_records, size_t *written) {
    if (generator == NULL || records == NULL || written == NULL || max_records == 0U ||
        max_records > SIZE_MAX / STEP_STREAM_GENERATOR_RECORD_WORDS) {
        if (written != NULL) {
            *written = 0U;
        }
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    *written = 0U;

    if (generator->initialized == 0U) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    if (generator->current_record_index == generator->current_record_count &&
        generator->next_interval == generator->total_intervals) {
        return STEP_STREAM_GENERATOR_STATUS_DONE;
    }

    size_t output_count = 0U;
    while (output_count < max_records) {
        if (generator->current_record_index == generator->current_record_count) {
            if (generator->next_interval == generator->total_intervals) {
                break;
            }
            const StepStreamGeneratorStatus_t prepare_status = step_stream_generator_prepare_interval(generator);
            if (prepare_status != STEP_STREAM_GENERATOR_STATUS_OK) {
                *written = output_count;
                return prepare_status;
            }
        }

        const uint64_t records_left = generator->current_record_count - generator->current_record_index;
        size_t batch_count = max_records - output_count;
        if (records_left < (uint64_t)batch_count) {
            batch_count = (size_t)records_left;
        }

        for (size_t index = 0U; index < batch_count; ++index) {
            const uint64_t record_index = generator->current_record_index;
            const uint32_t record_ticks =
                generator->current_base_ticks + (record_index < generator->current_extra_records ? 1U : 0U);
            const size_t output_index = output_count + index;
            const size_t offset = output_index * STEP_STREAM_GENERATOR_RECORD_WORDS;
            records[offset] = (uint16_t)(record_ticks - 1U);
            records[offset + 1U] = 0U;
            records[offset + 2U] =
                generator->step_count != 0U && record_index == 0U ? (uint16_t)generator->config.pulse_ticks : 0U;
            ++generator->current_record_index;
        }
        output_count += batch_count;
    }

    *written = output_count;
    return generator->current_record_index == generator->current_record_count &&
                   generator->next_interval == generator->total_intervals
               ? STEP_STREAM_GENERATOR_STATUS_DONE
               : STEP_STREAM_GENERATOR_STATUS_OK;
}

StepStreamGeneratorStatus_t
step_stream_generator_get_carry(const StepStreamGenerator_t *generator, StepStreamGeneratorCarry_t *carry) {
    if (generator == NULL || carry == NULL || generator->initialized == 0U) {
        return STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    carry->ns_to_ticks_remainder = generator->tick_remainder;
    return STEP_STREAM_GENERATOR_STATUS_OK;
}

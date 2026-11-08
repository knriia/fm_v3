#include "motion/step_stream_feeder.h"

#include <string.h>

#define STEP_STREAM_FEEDER_HALF_COUNT 2U
#define STEP_STREAM_FEEDER_NANOSECONDS_PER_SECOND 1000000000ULL

static void step_stream_feeder_counter_increment(uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static uint64_t step_stream_feeder_ns_to_ticks_ceil(uint64_t ns, uint32_t timer_clock_hz) {
    const uint64_t numerator = ns * timer_clock_hz;
    return numerator / STEP_STREAM_FEEDER_NANOSECONDS_PER_SECOND +
           (numerator % STEP_STREAM_FEEDER_NANOSECONDS_PER_SECOND != 0U ? 1U : 0U);
}

StepStreamFeederStatus_t step_stream_feeder_make_generator_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    StepStreamGeneratorConfig_t *config
) {
    if (config == NULL || timer_clock_hz == 0U || step_pulse_width_ns == 0U || min_record_ns == 0U) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    const uint64_t pulse_ticks = step_stream_feeder_ns_to_ticks_ceil(step_pulse_width_ns, timer_clock_hz);
    const uint64_t low_ticks = step_stream_feeder_ns_to_ticks_ceil(step_low_time_ns, timer_clock_hz);
    const uint64_t min_record_ticks = step_stream_feeder_ns_to_ticks_ceil(min_record_ns, timer_clock_hz);
    /* The generator checks the ranges again when a block starts; here only the narrowing must be exact. */
    if (pulse_ticks > UINT32_MAX || low_ticks > UINT32_MAX || min_record_ticks > UINT32_MAX) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    config->timer_clock_hz = timer_clock_hz;
    config->pulse_ticks = (uint32_t)pulse_ticks;
    config->low_ticks = (uint32_t)low_ticks;
    config->min_record_ticks = (uint32_t)min_record_ticks;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

StepStreamFeederStatus_t step_stream_feeder_make_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    uint32_t idle_record_ns,
    uint32_t startup_period_ns,
    uint32_t half_record_count,
    uint32_t axis_index,
    StepStreamFeederConfig_t *config
) {
    if (config == NULL || timer_clock_hz == 0U || idle_record_ns == 0U) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    StepStreamFeederConfig_t result = {0};
    const StepStreamFeederStatus_t generator_status = step_stream_feeder_make_generator_config(
        timer_clock_hz,
        step_pulse_width_ns,
        step_low_time_ns,
        min_record_ns,
        &result.generator
    );
    if (generator_status != STEP_STREAM_FEEDER_STATUS_OK) {
        return generator_status;
    }
    const uint64_t idle_ticks = step_stream_feeder_ns_to_ticks_ceil(idle_record_ns, timer_clock_hz);
    if (idle_ticks > UINT32_MAX) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    result.idle_record_ticks = (uint32_t)idle_ticks;
    result.startup_period_ns = startup_period_ns;
    result.half_record_count = half_record_count;
    result.axis_index = axis_index;
    *config = result;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

StepStreamFeederStatus_t step_stream_feeder_init(
    StepStreamFeeder_t *feeder,
    const StepStreamFeederConfig_t *config,
    const StepStreamFeederPorts_t *ports
) {
    if (feeder == NULL || config == NULL || ports == NULL || ports->prepare == NULL || ports->acquire_half == NULL ||
        ports->commit_half == NULL || ports->set_direction == NULL || ports->start == NULL || ports->stop == NULL ||
        ports->pop_block == NULL) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }
    if (config->axis_index >= MOTION_PLANNER_AXIS_COUNT || config->startup_period_ns == 0U ||
        config->half_record_count == 0U || config->idle_record_ticks == 0U ||
        config->idle_record_ticks > STEP_STREAM_GENERATOR_MAX_RECORD_TICKS ||
        config->idle_record_ticks < config->generator.min_record_ticks) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    memset(feeder, 0, sizeof(*feeder));
    feeder->config = *config;
    feeder->ports = *ports;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

void step_stream_feeder_reset(StepStreamFeeder_t *feeder) {
    if (feeder == NULL) {
        return;
    }
    feeder->phase = STEP_STREAM_FEEDER_PHASE_IDLE;
    feeder->fault_status = STEP_STREAM_FEEDER_STATUS_OK;
    feeder->carry = (StepStreamGeneratorCarry_t){0};
    feeder->generator_active = 0U;
    feeder->held_valid = 0U;
    feeder->direction_known = 0U;
    feeder->direction_positive = 0U;
    feeder->half_has_block[0] = 0U;
    feeder->half_has_block[1] = 0U;
}

static StepStreamFeederStatus_t step_stream_feeder_fault(StepStreamFeeder_t *feeder, StepStreamFeederStatus_t status) {
    feeder->ports.stop();
    feeder->phase = STEP_STREAM_FEEDER_PHASE_FAULT;
    feeder->fault_status = status;
    return status;
}

static StepStreamFeederStatus_t step_stream_feeder_axis_fault(StepStreamFeeder_t *feeder) {
    step_stream_feeder_counter_increment(&feeder->diagnostics.axis_errors);
    return step_stream_feeder_fault(feeder, STEP_STREAM_FEEDER_STATUS_AXIS_ERROR);
}

static StepStreamFeederStatus_t step_stream_feeder_generator_fault(StepStreamFeeder_t *feeder) {
    step_stream_feeder_counter_increment(&feeder->diagnostics.generator_errors);
    return step_stream_feeder_fault(feeder, STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR);
}

static uint8_t step_stream_feeder_axis_bit(const StepStreamFeeder_t *feeder, uint8_t mask) {
    return (uint8_t)((mask >> feeder->config.axis_index) & 1U);
}

/* The block waiting to join the stream, taken from the plan buffer on first need. */
static uint8_t step_stream_feeder_fetch_block(StepStreamFeeder_t *feeder) {
    if (feeder->held_valid != 0U) {
        return 1U;
    }
    if (feeder->ports.pop_block(&feeder->held_block) == 0U) {
        return 0U;
    }
    feeder->held_valid = 1U;
    return 1U;
}

/* A block that does not move the axis fits any stream. A moving block fits only when the stream already runs in its
 * direction: DIR changes only while the axis is stopped. */
static uint8_t step_stream_feeder_block_fits(const StepStreamFeeder_t *feeder) {
    const MotionPlanBlock_t *block = &feeder->held_block;
    if (step_stream_feeder_axis_bit(feeder, block->moving_axes_mask) == 0U) {
        return 1U;
    }
    return feeder->direction_known != 0U &&
                   feeder->direction_positive == step_stream_feeder_axis_bit(feeder, block->positive_direction_mask)
               ? 1U
               : 0U;
}

static uint8_t step_stream_feeder_joinable_block_waiting(StepStreamFeeder_t *feeder) {
    return step_stream_feeder_fetch_block(feeder) != 0U && step_stream_feeder_block_fits(feeder) != 0U ? 1U : 0U;
}

/* Starts the generator on the held block; 1 when it runs, 0 when no fitting block waits, -1 on a generator error. */
static int8_t step_stream_feeder_begin_block(StepStreamFeeder_t *feeder) {
    if (step_stream_feeder_joinable_block_waiting(feeder) == 0U) {
        return 0;
    }
    feeder->held_valid = 0U;
    if (step_stream_generator_init_from_block(
            &feeder->generator,
            &feeder->config.generator,
            &feeder->held_block,
            feeder->config.axis_index,
            &feeder->carry
        ) != STEP_STREAM_GENERATOR_STATUS_OK) {
        return -1;
    }
    feeder->generator_active = 1U;
    step_stream_feeder_counter_increment(&feeder->diagnostics.blocks_started);
    return 1;
}

static void step_stream_feeder_write_idle(const StepStreamFeeder_t *feeder, uint16_t *records, uint32_t count) {
    for (uint32_t index = 0U; index < count; ++index) {
        uint16_t *record = &records[(size_t)index * STEP_STREAM_GENERATOR_RECORD_WORDS];
        record[0] = (uint16_t)(feeder->config.idle_record_ticks - 1U);
        record[1] = 0U;
        record[2] = 0U;
    }
}

/* Writes one whole half: block records first, then idle records to its end. */
static StepStreamFeederStatus_t
step_stream_feeder_fill_half(StepStreamFeeder_t *feeder, uint16_t *records, uint8_t *has_block) {
    const uint32_t capacity = feeder->config.half_record_count;
    uint32_t used = 0U;
    *has_block = 0U;

    while (used < capacity) {
        if (feeder->generator_active == 0U) {
            const int8_t begin = step_stream_feeder_begin_block(feeder);
            if (begin < 0) {
                return step_stream_feeder_generator_fault(feeder);
            }
            if (begin == 0) {
                break;
            }
        }

        size_t written = 0U;
        const StepStreamGeneratorStatus_t status = step_stream_generator_fill(
            &feeder->generator,
            &records[(size_t)used * STEP_STREAM_GENERATOR_RECORD_WORDS],
            capacity - used,
            &written
        );
        if (status != STEP_STREAM_GENERATOR_STATUS_OK && status != STEP_STREAM_GENERATOR_STATUS_DONE) {
            return step_stream_feeder_generator_fault(feeder);
        }
        used += (uint32_t)written;
        /* Block time counts even when this axis does not move: the stream may not stop in the middle of a block. */
        *has_block = 1U;
        if (status == STEP_STREAM_GENERATOR_STATUS_DONE) {
            /* The carry was just read from an initialized generator, so it cannot fail. */
            (void)step_stream_generator_get_carry(&feeder->generator, &feeder->carry);
            feeder->generator_active = 0U;
            step_stream_feeder_counter_increment(&feeder->diagnostics.blocks_completed);
        }
    }

    step_stream_feeder_write_idle(feeder, &records[(size_t)used * STEP_STREAM_GENERATOR_RECORD_WORDS], capacity - used);
    return STEP_STREAM_FEEDER_STATUS_OK;
}

/* Opens a stream on the held block: resets the ring and puts DIR where the block needs it. */
static StepStreamFeederStatus_t step_stream_feeder_begin_stream(StepStreamFeeder_t *feeder) {
    const uint8_t moves = step_stream_feeder_axis_bit(feeder, feeder->held_block.moving_axes_mask);
    const uint8_t positive = step_stream_feeder_axis_bit(feeder, feeder->held_block.positive_direction_mask);

    if (feeder->ports.prepare() != STEP_STREAM_FEEDER_AXIS_OK) {
        return step_stream_feeder_axis_fault(feeder);
    }
    if (moves != 0U && feeder->ports.set_direction(positive) != STEP_STREAM_FEEDER_AXIS_OK) {
        return step_stream_feeder_axis_fault(feeder);
    }

    feeder->direction_known = moves;
    feeder->direction_positive = moves != 0U ? positive : 0U;
    feeder->half_has_block[0] = 0U;
    feeder->half_has_block[1] = 0U;
    feeder->carry = (StepStreamGeneratorCarry_t){0};
    feeder->generator_active = 0U;
    feeder->phase = STEP_STREAM_FEEDER_PHASE_PRIMING;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

/* The released half and the one now playing carry no block time and no block can follow in this stream. */
static uint8_t step_stream_feeder_stream_finished(StepStreamFeeder_t *feeder, uint8_t released_half) {
    return feeder->half_has_block[released_half] == 0U &&
                   feeder->half_has_block[STEP_STREAM_FEEDER_HALF_COUNT - 1U - released_half] == 0U &&
                   step_stream_feeder_joinable_block_waiting(feeder) == 0U
               ? 1U
               : 0U;
}

/* Refills every released half. *stopped is set when the idle stream was stopped instead. */
static StepStreamFeederStatus_t step_stream_feeder_fill_released_halves(StepStreamFeeder_t *feeder, uint8_t *stopped) {
    *stopped = 0U;
    for (;;) {
        uint8_t half = 0U;
        uint16_t *records = NULL;
        const StepStreamFeederAxisResult_t acquired = feeder->ports.acquire_half(&half, &records);
        if (acquired == STEP_STREAM_FEEDER_AXIS_NO_REQUEST) {
            return STEP_STREAM_FEEDER_STATUS_OK;
        }
        if (acquired != STEP_STREAM_FEEDER_AXIS_OK || half >= STEP_STREAM_FEEDER_HALF_COUNT || records == NULL) {
            return step_stream_feeder_axis_fault(feeder);
        }

        if (feeder->phase == STEP_STREAM_FEEDER_PHASE_STREAMING && step_stream_feeder_stream_finished(feeder, half)) {
            feeder->ports.stop();
            feeder->phase = STEP_STREAM_FEEDER_PHASE_IDLE;
            step_stream_feeder_counter_increment(&feeder->diagnostics.streams_stopped);
            *stopped = 1U;
            return STEP_STREAM_FEEDER_STATUS_OK;
        }

        uint8_t has_block = 0U;
        const StepStreamFeederStatus_t filled = step_stream_feeder_fill_half(feeder, records, &has_block);
        if (filled != STEP_STREAM_FEEDER_STATUS_OK) {
            return filled;
        }
        if (feeder->ports.commit_half(half) != STEP_STREAM_FEEDER_AXIS_OK) {
            return step_stream_feeder_axis_fault(feeder);
        }
        feeder->half_has_block[half] = has_block;
        step_stream_feeder_counter_increment(&feeder->diagnostics.halves_filled);
    }
}

StepStreamFeederStatus_t step_stream_feeder_service(StepStreamFeeder_t *feeder) {
    if (feeder == NULL) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }
    if (feeder->phase == STEP_STREAM_FEEDER_PHASE_FAULT) {
        return feeder->fault_status;
    }

    for (;;) {
        if (feeder->phase == STEP_STREAM_FEEDER_PHASE_IDLE) {
            if (step_stream_feeder_fetch_block(feeder) == 0U) {
                return STEP_STREAM_FEEDER_STATUS_OK;
            }
            const StepStreamFeederStatus_t begun = step_stream_feeder_begin_stream(feeder);
            if (begun != STEP_STREAM_FEEDER_STATUS_OK) {
                return begun;
            }
        }

        uint8_t stopped = 0U;
        const StepStreamFeederStatus_t filled = step_stream_feeder_fill_released_halves(feeder, &stopped);
        if (filled != STEP_STREAM_FEEDER_STATUS_OK) {
            return filled;
        }
        if (stopped != 0U) {
            /* A block that waited for the direction change opens the next stream right away. */
            continue;
        }

        if (feeder->phase == STEP_STREAM_FEEDER_PHASE_PRIMING) {
            /* Both halves were filled above; an axis that still lacks one refuses the start and faults. */
            if (feeder->ports.start(feeder->config.startup_period_ns) != STEP_STREAM_FEEDER_AXIS_OK) {
                return step_stream_feeder_axis_fault(feeder);
            }
            feeder->phase = STEP_STREAM_FEEDER_PHASE_STREAMING;
            step_stream_feeder_counter_increment(&feeder->diagnostics.streams_started);
        }
        return STEP_STREAM_FEEDER_STATUS_OK;
    }
}

StepStreamFeederPhase_t step_stream_feeder_get_phase(const StepStreamFeeder_t *feeder) {
    return feeder == NULL ? STEP_STREAM_FEEDER_PHASE_IDLE : feeder->phase;
}

void step_stream_feeder_get_diagnostics(const StepStreamFeeder_t *feeder, StepStreamFeederDiagnostics_t *diagnostics) {
    if (feeder == NULL || diagnostics == NULL) {
        return;
    }
    *diagnostics = feeder->diagnostics;
}

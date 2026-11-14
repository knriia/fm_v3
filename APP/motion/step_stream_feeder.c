#include "motion/step_stream_feeder.h"

#include <string.h>

#define STEP_STREAM_FEEDER_HALF_COUNT 2U
#define STEP_STREAM_FEEDER_NANOSECONDS_PER_SECOND 1000000000ULL
/* end_block while the stream still takes blocks. */
#define STEP_STREAM_FEEDER_STREAM_OPEN UINT32_MAX
#define STEP_STREAM_FEEDER_NO_LANE UINT8_MAX

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
    uint32_t max_record_ns,
    StepStreamGeneratorConfig_t *config
) {
    if (config == NULL || timer_clock_hz == 0U || step_pulse_width_ns == 0U || min_record_ns == 0U) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    const uint64_t pulse_ticks = step_stream_feeder_ns_to_ticks_ceil(step_pulse_width_ns, timer_clock_hz);
    const uint64_t low_ticks = step_stream_feeder_ns_to_ticks_ceil(step_low_time_ns, timer_clock_hz);
    const uint64_t min_record_ticks = step_stream_feeder_ns_to_ticks_ceil(min_record_ns, timer_clock_hz);
    const uint64_t max_record_ticks = step_stream_feeder_ns_to_ticks_ceil(max_record_ns, timer_clock_hz);
    /* The generator checks the ranges again when a block starts; here only the narrowing must be exact. */
    if (pulse_ticks > UINT32_MAX || low_ticks > UINT32_MAX || min_record_ticks > UINT32_MAX ||
        max_record_ticks > UINT32_MAX) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    config->timer_clock_hz = timer_clock_hz;
    config->pulse_ticks = (uint32_t)pulse_ticks;
    config->low_ticks = (uint32_t)low_ticks;
    config->min_record_ticks = (uint32_t)min_record_ticks;
    config->max_record_ticks = (uint32_t)max_record_ticks;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

StepStreamFeederStatus_t step_stream_feeder_make_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    uint32_t max_record_ns,
    uint32_t idle_record_ns,
    uint32_t startup_period_ns,
    uint32_t half_record_count,
    const uint32_t *axis_index,
    uint32_t lane_count,
    StepStreamFeederConfig_t *config
) {
    if (config == NULL || axis_index == NULL || timer_clock_hz == 0U || idle_record_ns == 0U || lane_count == 0U ||
        lane_count > STEP_STREAM_FEEDER_MAX_LANES) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    StepStreamFeederConfig_t result = {0};
    const StepStreamFeederStatus_t generator_status = step_stream_feeder_make_generator_config(
        timer_clock_hz,
        step_pulse_width_ns,
        step_low_time_ns,
        min_record_ns,
        max_record_ns,
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
    result.lane_count = lane_count;
    for (uint32_t lane = 0U; lane < lane_count; ++lane) {
        result.axis_index[lane] = axis_index[lane];
    }
    *config = result;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

static uint8_t step_stream_feeder_ports_are_complete(const StepStreamFeederPorts_t *ports) {
    return (uint8_t)(ports->prepare != NULL && ports->acquire_half != NULL && ports->commit_half != NULL &&
                     ports->set_direction != NULL && ports->start != NULL && ports->stop != NULL &&
                     ports->plan_epoch != NULL && ports->peek_block != NULL && ports->discard_blocks != NULL);
}

/* Every lane drives its own planner axis. */
static uint8_t step_stream_feeder_lanes_are_valid(const StepStreamFeederConfig_t *config) {
    if (config->lane_count == 0U || config->lane_count > STEP_STREAM_FEEDER_MAX_LANES) {
        return 0U;
    }
    uint8_t seen = 0U;
    for (uint32_t lane = 0U; lane < config->lane_count; ++lane) {
        if (config->axis_index[lane] >= MOTION_PLANNER_AXIS_COUNT || ((seen >> config->axis_index[lane]) & 1U) != 0U) {
            return 0U;
        }
        seen = (uint8_t)(seen | (1U << config->axis_index[lane]));
    }
    return 1U;
}

StepStreamFeederStatus_t step_stream_feeder_init(
    StepStreamFeeder_t *feeder,
    const StepStreamFeederConfig_t *config,
    const StepStreamFeederPorts_t *ports
) {
    if (feeder == NULL || config == NULL || ports == NULL || step_stream_feeder_ports_are_complete(ports) == 0U) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }
    if (step_stream_feeder_lanes_are_valid(config) == 0U || config->startup_period_ns == 0U ||
        config->half_record_count == 0U || config->idle_record_ticks == 0U ||
        config->idle_record_ticks > STEP_STREAM_GENERATOR_MAX_RECORD_TICKS ||
        config->idle_record_ticks < config->generator.min_record_ticks) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    /* A configuration that no block could ever run is refused here, not at the first block. */
    StepStreamGenerator_t probe;
    if (step_stream_generator_init(&probe, &config->generator, 1U, 1U, NULL) != STEP_STREAM_GENERATOR_STATUS_OK) {
        return STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT;
    }

    memset(feeder, 0, sizeof(*feeder));
    feeder->config = *config;
    feeder->ports = *ports;
    for (uint32_t lane = 0U; lane < config->lane_count; ++lane) {
        feeder->lane_axes_mask = (uint8_t)(feeder->lane_axes_mask | (1U << config->axis_index[lane]));
    }
    feeder->end_block = STEP_STREAM_FEEDER_STREAM_OPEN;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

void step_stream_feeder_reset(StepStreamFeeder_t *feeder) {
    if (feeder == NULL) {
        return;
    }
    feeder->phase = STEP_STREAM_FEEDER_PHASE_IDLE;
    feeder->fault_status = STEP_STREAM_FEEDER_STATUS_OK;
    memset(feeder->lane, 0, sizeof(feeder->lane));
    feeder->plan_epoch = 0U;
    feeder->front_block = 0U;
    feeder->end_block = STEP_STREAM_FEEDER_STREAM_OPEN;
    feeder->direction_known_mask = 0U;
    feeder->direction_positive_mask = 0U;
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

static uint8_t step_stream_feeder_bit(uint8_t mask, uint32_t axis) { return (uint8_t)((mask >> axis) & 1U); }

/* Does the block fit the stream? Every axis that it moves must have its direction decided and the same as the block
 * wants: DIR changes only while the stream is stopped. */
static uint8_t step_stream_feeder_block_fits(const StepStreamFeeder_t *feeder, const MotionPlanBlock_t *block) {
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        if (step_stream_feeder_bit(block->moving_axes_mask, axis) == 0U) {
            continue;
        }
        if (step_stream_feeder_bit(feeder->direction_known_mask, axis) == 0U ||
            step_stream_feeder_bit(feeder->direction_positive_mask, axis) !=
                step_stream_feeder_bit(block->positive_direction_mask, axis)) {
            return 0U;
        }
    }
    return 1U;
}

/* A block that moves an axis without a lane would lose that motion, so it is an error and no stream runs it. */
static uint8_t step_stream_feeder_block_is_drivable(const StepStreamFeeder_t *feeder, const MotionPlanBlock_t *block) {
    return (block->moving_axes_mask & (uint8_t)~feeder->lane_axes_mask) == 0U ? 1U : 0U;
}

typedef enum {
    /* The lane can start its next block now; the block is copied out. */
    STEP_STREAM_FEEDER_NEXT_JOINABLE = 0,
    /* No block, or the stream does not take it: the lane fills with idle records. */
    STEP_STREAM_FEEDER_NEXT_NONE,
    STEP_STREAM_FEEDER_NEXT_ERROR
} StepStreamFeederNext_t;

/* The plan buffer was cleared behind the stream's back: a planner error cancels the pending blocks. A single lane has
 * lost only blocks that it had not started, so it goes on with what is queued now, as it always did. With several lanes
 * some may already have started blocks that the others can no longer start, so the lanes cannot go on together: 0. */
static uint8_t step_stream_feeder_rebase_after_clear(StepStreamFeeder_t *feeder) {
    if (feeder->config.lane_count != 1U) {
        return 0U;
    }
    feeder->plan_epoch = feeder->ports.plan_epoch();
    feeder->front_block = feeder->lane[0].next_block;
    return 1U;
}

/* The next block of the lane. With several lanes, where the queue ends the stream closes for every lane: a block that
 * arrives later could not be started by all lanes at the same moment, because a lane that has already filled idle
 * records for the time after the end cannot take them back. The block waits for the next stream. */
static StepStreamFeederNext_t
step_stream_feeder_next_block(StepStreamFeeder_t *feeder, uint32_t lane, MotionPlanBlock_t *block) {
    const uint32_t index = feeder->lane[lane].next_block;
    if (index >= feeder->end_block) {
        return STEP_STREAM_FEEDER_NEXT_NONE;
    }
    const StepStreamFeederPlanResult_t peeked =
        feeder->ports.peek_block(feeder->plan_epoch, index - feeder->front_block, block);
    if (peeked == STEP_STREAM_FEEDER_PLAN_NONE) {
        /* One lane alone cannot get out of step with itself, so its stream stays open for a late block. */
        if (feeder->config.lane_count > 1U) {
            feeder->end_block = index;
        }
        return STEP_STREAM_FEEDER_NEXT_NONE;
    }
    if (peeked == STEP_STREAM_FEEDER_PLAN_STALE && step_stream_feeder_rebase_after_clear(feeder) != 0U) {
        return STEP_STREAM_FEEDER_NEXT_NONE;
    }
    if (peeked != STEP_STREAM_FEEDER_PLAN_OK || step_stream_feeder_block_is_drivable(feeder, block) == 0U) {
        return STEP_STREAM_FEEDER_NEXT_ERROR;
    }
    return step_stream_feeder_block_fits(feeder, block) != 0U ? STEP_STREAM_FEEDER_NEXT_JOINABLE
                                                              : STEP_STREAM_FEEDER_NEXT_NONE;
}

/* Gives back the blocks that every lane has started. */
static uint8_t step_stream_feeder_retire_blocks(StepStreamFeeder_t *feeder) {
    uint32_t started_by_all = feeder->lane[0].next_block;
    for (uint32_t lane = 1U; lane < feeder->config.lane_count; ++lane) {
        if (feeder->lane[lane].next_block < started_by_all) {
            started_by_all = feeder->lane[lane].next_block;
        }
    }
    if (started_by_all == feeder->front_block) {
        return 1U;
    }
    const StepStreamFeederPlanResult_t discarded =
        feeder->ports.discard_blocks(feeder->plan_epoch, started_by_all - feeder->front_block);
    if (discarded == STEP_STREAM_FEEDER_PLAN_STALE) {
        /* The clear has taken the blocks away already. */
        return step_stream_feeder_rebase_after_clear(feeder);
    }
    if (discarded != STEP_STREAM_FEEDER_PLAN_OK) {
        return 0U;
    }
    feeder->front_block = started_by_all;
    return 1U;
}

/* Starts the generator of the lane on its next block; 1 when it runs, 0 when no block can start, -1 on an error. */
static int8_t step_stream_feeder_begin_block(StepStreamFeeder_t *feeder, uint32_t lane) {
    MotionPlanBlock_t block;
    const StepStreamFeederNext_t next = step_stream_feeder_next_block(feeder, lane, &block);
    if (next == STEP_STREAM_FEEDER_NEXT_NONE) {
        return 0;
    }
    StepStreamFeederLane_t *state = &feeder->lane[lane];
    if (next == STEP_STREAM_FEEDER_NEXT_ERROR || step_stream_generator_init_from_block(
                                                     &state->generator,
                                                     &feeder->config.generator,
                                                     &block,
                                                     feeder->config.axis_index[lane],
                                                     &state->carry
                                                 ) != STEP_STREAM_GENERATOR_STATUS_OK) {
        return -1;
    }
    state->generator_active = 1U;
    ++state->next_block;
    if (lane == 0U) {
        step_stream_feeder_counter_increment(&feeder->diagnostics.blocks_started);
    }
    return step_stream_feeder_retire_blocks(feeder) != 0U ? 1 : -1;
}

static void step_stream_feeder_write_idle(const StepStreamFeeder_t *feeder, uint16_t *records, uint32_t count) {
    for (uint32_t index = 0U; index < count; ++index) {
        uint16_t *record = &records[(size_t)index * STEP_STREAM_GENERATOR_RECORD_WORDS];
        record[0] = (uint16_t)(feeder->config.idle_record_ticks - 1U);
        record[1] = 0U;
        record[2] = 0U;
    }
}

/* Writes one whole half of a lane: block records first, then idle records to its end. */
static StepStreamFeederStatus_t
step_stream_feeder_fill_half(StepStreamFeeder_t *feeder, uint32_t lane, uint16_t *records, uint8_t *has_block) {
    StepStreamFeederLane_t *state = &feeder->lane[lane];
    const uint32_t capacity = feeder->config.half_record_count;
    uint32_t used = 0U;
    *has_block = 0U;

    while (used < capacity) {
        if (state->generator_active == 0U) {
            const int8_t begin = step_stream_feeder_begin_block(feeder, lane);
            if (begin < 0) {
                return step_stream_feeder_generator_fault(feeder);
            }
            if (begin == 0) {
                break;
            }
        }

        size_t written = 0U;
        const StepStreamGeneratorStatus_t status = step_stream_generator_fill(
            &state->generator,
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
            (void)step_stream_generator_get_carry(&state->generator, &state->carry);
            state->generator_active = 0U;
            if (lane == 0U) {
                step_stream_feeder_counter_increment(&feeder->diagnostics.blocks_completed);
            }
        }
    }

    step_stream_feeder_write_idle(feeder, &records[(size_t)used * STEP_STREAM_GENERATOR_RECORD_WORDS], capacity - used);
    return STEP_STREAM_FEEDER_STATUS_OK;
}

/* Decides the direction of every axis the first blocks of the plan move, as far as they agree. The stream cannot
 * change DIR later, so looking ahead lets a following block that moves another axis join instead of waiting for a
 * stop. Returns 0 when the front of the plan has no block (nothing to start) or a stale one (a clear raced with this
 * call: the next call starts afresh), -1 when the plan cannot be run. */
static int8_t step_stream_feeder_decide_directions(StepStreamFeeder_t *feeder, uint32_t epoch) {
    uint8_t known = 0U;
    uint8_t positive = 0U;
    for (uint32_t offset = 0U;; ++offset) {
        MotionPlanBlock_t block;
        const StepStreamFeederPlanResult_t peeked = feeder->ports.peek_block(epoch, offset, &block);
        /* Nothing has started yet, so a clear that comes in the middle of the scan only means: look again next time. */
        if (peeked == STEP_STREAM_FEEDER_PLAN_STALE) {
            return 0;
        }
        if (peeked == STEP_STREAM_FEEDER_PLAN_NONE) {
            if (offset == 0U) {
                return 0;
            }
            break;
        }
        if (peeked != STEP_STREAM_FEEDER_PLAN_OK) {
            return -1;
        }
        if (step_stream_feeder_block_is_drivable(feeder, &block) == 0U) {
            if (offset == 0U) {
                return -1;
            }
            break;
        }

        uint8_t agrees = 1U;
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            if (step_stream_feeder_bit(block.moving_axes_mask, axis) != 0U &&
                step_stream_feeder_bit(known, axis) != 0U &&
                step_stream_feeder_bit(positive, axis) != step_stream_feeder_bit(block.positive_direction_mask, axis)) {
                agrees = 0U;
            }
        }
        if (agrees == 0U) {
            break;
        }
        /* The first block that moves an axis decides its direction. */
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            if (step_stream_feeder_bit(block.moving_axes_mask, axis) != 0U &&
                step_stream_feeder_bit(known, axis) == 0U) {
                known = (uint8_t)(known | (1U << axis));
                positive = (uint8_t)(positive | (step_stream_feeder_bit(block.positive_direction_mask, axis) << axis));
            }
        }
    }
    feeder->direction_known_mask = known;
    feeder->direction_positive_mask = positive;
    return 1;
}

/* Opens a stream on the front of the plan: decides the directions, resets the rings and puts DIR where the blocks need
 * it. *begun is 0 when the plan has nothing to start. */
static StepStreamFeederStatus_t step_stream_feeder_begin_stream(StepStreamFeeder_t *feeder, uint8_t *begun) {
    *begun = 0U;
    const uint32_t epoch = feeder->ports.plan_epoch();
    const int8_t decided = step_stream_feeder_decide_directions(feeder, epoch);
    if (decided < 0) {
        return step_stream_feeder_generator_fault(feeder);
    }
    if (decided == 0) {
        return STEP_STREAM_FEEDER_STATUS_OK;
    }

    if (feeder->ports.prepare() != STEP_STREAM_FEEDER_AXIS_OK) {
        return step_stream_feeder_axis_fault(feeder);
    }
    for (uint32_t lane = 0U; lane < feeder->config.lane_count; ++lane) {
        const uint32_t axis = feeder->config.axis_index[lane];
        if (step_stream_feeder_bit(feeder->direction_known_mask, axis) != 0U &&
            feeder->ports.set_direction(lane, step_stream_feeder_bit(feeder->direction_positive_mask, axis)) !=
                STEP_STREAM_FEEDER_AXIS_OK) {
            return step_stream_feeder_axis_fault(feeder);
        }
    }

    memset(feeder->lane, 0, sizeof(feeder->lane));
    feeder->plan_epoch = epoch;
    feeder->front_block = 0U;
    feeder->end_block = STEP_STREAM_FEEDER_STREAM_OPEN;
    feeder->phase = STEP_STREAM_FEEDER_PHASE_PRIMING;
    *begun = 1U;
    return STEP_STREAM_FEEDER_STATUS_OK;
}

/* Every lane has played its last block and no lane can start another: the halves that it has filled carry no block time
 * and the plan holds nothing that the stream takes. */
static uint8_t step_stream_feeder_stream_finished(StepStreamFeeder_t *feeder) {
    for (uint32_t lane = 0U; lane < feeder->config.lane_count; ++lane) {
        MotionPlanBlock_t block;
        if (feeder->lane[lane].half_has_block[0] != 0U || feeder->lane[lane].half_has_block[1] != 0U ||
            step_stream_feeder_next_block(feeder, lane, &block) != STEP_STREAM_FEEDER_NEXT_NONE) {
            return 0U;
        }
    }
    return 1U;
}

/* Refills every released half of every lane. *stopped is set when the idle stream was stopped instead. */
static StepStreamFeederStatus_t step_stream_feeder_fill_released_halves(StepStreamFeeder_t *feeder, uint8_t *stopped) {
    *stopped = 0U;
    for (uint32_t lane = 0U; lane < feeder->config.lane_count; ++lane) {
        for (;;) {
            uint8_t half = 0U;
            uint16_t *records = NULL;
            const StepStreamFeederAxisResult_t acquired = feeder->ports.acquire_half(lane, &half, &records);
            if (acquired == STEP_STREAM_FEEDER_AXIS_NO_REQUEST) {
                break;
            }
            if (acquired != STEP_STREAM_FEEDER_AXIS_OK || half >= STEP_STREAM_FEEDER_HALF_COUNT || records == NULL) {
                return step_stream_feeder_axis_fault(feeder);
            }

            if (feeder->phase == STEP_STREAM_FEEDER_PHASE_STREAMING && step_stream_feeder_stream_finished(feeder)) {
                feeder->ports.stop();
                feeder->phase = STEP_STREAM_FEEDER_PHASE_IDLE;
                step_stream_feeder_counter_increment(&feeder->diagnostics.streams_stopped);
                *stopped = 1U;
                return STEP_STREAM_FEEDER_STATUS_OK;
            }

            uint8_t has_block = 0U;
            const StepStreamFeederStatus_t filled = step_stream_feeder_fill_half(feeder, lane, records, &has_block);
            if (filled != STEP_STREAM_FEEDER_STATUS_OK) {
                return filled;
            }
            if (feeder->ports.commit_half(lane, half) != STEP_STREAM_FEEDER_AXIS_OK) {
                return step_stream_feeder_axis_fault(feeder);
            }
            feeder->lane[lane].half_has_block[half] = has_block;
            if (lane == 0U) {
                step_stream_feeder_counter_increment(&feeder->diagnostics.halves_filled);
            }
        }
    }
    return STEP_STREAM_FEEDER_STATUS_OK;
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
            uint8_t begun = 0U;
            const StepStreamFeederStatus_t status = step_stream_feeder_begin_stream(feeder, &begun);
            if (status != STEP_STREAM_FEEDER_STATUS_OK || begun == 0U) {
                return status;
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
            /* Both halves of every lane were filled above; an axis that still lacks one refuses the start and faults.
             */
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

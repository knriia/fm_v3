#include "motion/motion_event_generator.h"

#include <stddef.h>
#include <string.h>

static uint64_t motion_event_generator_abs_i64(int64_t value) {
    return value < 0 ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
}

static int motion_event_generator_validate_block(
    const MotionPlanBlock_t *block,
    uint64_t absolute_step_delta[MOTION_PLANNER_AXIS_COUNT],
    uint64_t *interval_numerator_ns
) {
    if (block->master_step_count == 0U || block->path_length_um == 0U || block->feedrate_um_per_s == 0U ||
        block->step_event_rate_numerator == 0U || block->step_event_rate_denominator == 0U ||
        block->step_event_rate_denominator != block->path_length_um) {
        return 0;
    }

    if (block->moving_axes_mask == 0U || (block->moving_axes_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U ||
        (block->positive_direction_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U ||
        (block->positive_direction_mask & (uint8_t)~block->moving_axes_mask) != 0U) {
        return 0;
    }

    uint64_t maximum_step_delta = 0U;
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const int64_t signed_delta = block->step_delta[axis];
        const uint8_t axis_mask = (uint8_t)(1U << axis);
        const int axis_is_moving = (block->moving_axes_mask & axis_mask) != 0U;
        const int direction_is_positive = (block->positive_direction_mask & axis_mask) != 0U;

        absolute_step_delta[axis] = motion_event_generator_abs_i64(signed_delta);
        if (axis_is_moving != (absolute_step_delta[axis] != 0U)) {
            return 0;
        }
        if (axis_is_moving && (signed_delta > 0) != direction_is_positive) {
            return 0;
        }
        if (absolute_step_delta[axis] > block->master_step_count) {
            return 0;
        }
        if (absolute_step_delta[axis] > maximum_step_delta) {
            maximum_step_delta = absolute_step_delta[axis];
        }
    }

    if (maximum_step_delta != block->master_step_count) {
        return 0;
    }
    if ((uint64_t)block->feedrate_um_per_s > UINT64_MAX / block->master_step_count ||
        (uint64_t)block->feedrate_um_per_s * block->master_step_count != block->step_event_rate_numerator) {
        return 0;
    }
    if (block->step_event_rate_denominator > UINT64_MAX / MOTION_EVENT_GENERATOR_NANOSECONDS_PER_SECOND) {
        return 0;
    }

    *interval_numerator_ns = block->step_event_rate_denominator * MOTION_EVENT_GENERATOR_NANOSECONDS_PER_SECOND;
    if (block->step_event_rate_numerator > *interval_numerator_ns) {
        return 0;
    }

    return 1;
}

MotionEventGeneratorStatus_t
motion_event_generator_init(MotionEventGenerator_t *generator, const MotionPlanBlock_t *block) {
    if (generator == NULL) {
        return MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    (void)memset(generator, 0, sizeof(*generator));
    if (block == NULL) {
        return MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    uint64_t interval_numerator_ns = 0U;
    if (!motion_event_generator_validate_block(block, generator->absolute_step_delta, &interval_numerator_ns)) {
        return MOTION_EVENT_GENERATOR_STATUS_INVALID_BLOCK;
    }

    generator->master_step_count = block->master_step_count;
    generator->events_remaining = block->master_step_count;
    generator->interval_floor_ns = interval_numerator_ns / block->step_event_rate_numerator;
    generator->interval_remainder = interval_numerator_ns % block->step_event_rate_numerator;
    generator->interval_fraction_denominator = block->step_event_rate_numerator;
    generator->sequence = block->sequence;
    generator->positive_direction_mask = block->positive_direction_mask;
    generator->initialized = 1U;
    return MOTION_EVENT_GENERATOR_STATUS_OK;
}

MotionEventGeneratorStatus_t motion_event_generator_next(MotionEventGenerator_t *generator, MotionEvent_t *event) {
    if (generator == NULL || event == NULL) {
        return MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    if (generator->initialized == 0U) {
        return MOTION_EVENT_GENERATOR_STATUS_NOT_INITIALIZED;
    }
    if (generator->events_remaining == 0U) {
        return MOTION_EVENT_GENERATOR_STATUS_COMPLETE;
    }

    uint8_t step_mask = 0U;
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const uint64_t step_delta = generator->absolute_step_delta[axis];
        if (step_delta == 0U) {
            continue;
        }

        const uint64_t threshold = generator->master_step_count - step_delta;
        if (generator->step_accumulator[axis] >= threshold) {
            generator->step_accumulator[axis] -= threshold;
            step_mask |= (uint8_t)(1U << axis);
        } else {
            generator->step_accumulator[axis] += step_delta;
        }
    }

    uint64_t interval_ns = generator->interval_floor_ns;
    if (generator->interval_remainder != 0U) {
        const uint64_t remainder_to_carry = generator->interval_fraction_denominator - generator->interval_remainder;
        if (generator->interval_fraction_accumulator >= remainder_to_carry) {
            generator->interval_fraction_accumulator -= remainder_to_carry;
            ++interval_ns;
        } else {
            generator->interval_fraction_accumulator += generator->interval_remainder;
        }
    }

    *event = (MotionEvent_t){
        .sequence = generator->sequence,
        .interval_ns = interval_ns,
        .step_mask = step_mask,
        .positive_direction_mask = generator->positive_direction_mask,
    };
    --generator->events_remaining;
    return MOTION_EVENT_GENERATOR_STATUS_OK;
}

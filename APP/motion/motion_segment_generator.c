#include "motion/motion_segment_generator.h"

#include <stddef.h>
#define MOTION_NANOSECONDS_PER_SECOND 1000000000ULL

static uint64_t motion_segment_abs_i64(int64_t value) {
    return value < 0 ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
}

static MotionSegmentGeneratorStatus_t
motion_segment_validate_block(const MotionPlanBlock_t *block, uint64_t axis_steps[MOTION_PLANNER_AXIS_COUNT]) {
    if (block->master_step_count == 0U || block->step_event_rate_numerator == 0U ||
        block->step_event_rate_denominator == 0U || block->moving_axes_mask == 0U ||
        (block->moving_axes_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U ||
        (block->positive_direction_mask & (uint8_t)~block->moving_axes_mask) != 0U) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
    }

    uint8_t computed_mask = 0U;
    uint8_t computed_positive_mask = 0U;
    uint64_t max_axis_steps = 0U;
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        axis_steps[axis] = motion_segment_abs_i64(block->step_delta[axis]);
        if (axis_steps[axis] > block->master_step_count) {
            return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
        }
        if (axis_steps[axis] != 0U) {
            const uint8_t axis_mask = (uint8_t)(1U << axis);
            computed_mask |= axis_mask;
            if (block->step_delta[axis] > 0) {
                computed_positive_mask |= axis_mask;
            }
            if (axis_steps[axis] > max_axis_steps) {
                max_axis_steps = axis_steps[axis];
            }
        }
    }

    if (computed_mask != block->moving_axes_mask || computed_positive_mask != block->positive_direction_mask ||
        max_axis_steps != block->master_step_count) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
    }

    if (block->step_event_rate_denominator > UINT64_MAX / MOTION_NANOSECONDS_PER_SECOND) {
        return MOTION_SEGMENT_GENERATOR_STATUS_ARITHMETIC_OVERFLOW;
    }
    const uint64_t interval_numerator = block->step_event_rate_denominator * MOTION_NANOSECONDS_PER_SECOND;
    if (interval_numerator < block->step_event_rate_numerator) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
    }
    return MOTION_SEGMENT_GENERATOR_STATUS_OK;
}

MotionSegmentGeneratorStatus_t
motion_segment_generator_init(MotionSegmentGenerator_t *generator, const MotionPlanBlock_t *block) {
    if (generator == NULL || block == NULL) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT;
    }

    const MotionPlanBlock_t block_copy = *block;
    uint64_t axis_steps[MOTION_PLANNER_AXIS_COUNT] = {0U};
    const MotionSegmentGeneratorStatus_t validation_status = motion_segment_validate_block(&block_copy, axis_steps);
    if (validation_status != MOTION_SEGMENT_GENERATOR_STATUS_OK) {
        return validation_status;
    }

    const uint64_t interval_numerator = block_copy.step_event_rate_denominator * MOTION_NANOSECONDS_PER_SECOND;
    MotionSegmentGenerator_t next_generator = {0};
    next_generator.block = block_copy;
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        next_generator.axis_steps[axis] = axis_steps[axis];
    }
    next_generator.interval_base_ns = interval_numerator / block_copy.step_event_rate_numerator;
    next_generator.interval_remainder_ns = interval_numerator % block_copy.step_event_rate_numerator;
    next_generator.initialized = 1U;
    *generator = next_generator;
    return MOTION_SEGMENT_GENERATOR_STATUS_OK;
}

MotionSegmentGeneratorStatus_t
motion_segment_generator_next(MotionSegmentGenerator_t *generator, MotionExecutionEvent_t *event) {
    if (generator == NULL || event == NULL) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    if (generator->initialized == 0U) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT;
    }
    if (generator->events_emitted >= generator->block.master_step_count) {
        return MOTION_SEGMENT_GENERATOR_STATUS_DONE;
    }

    MotionExecutionEvent_t next_event = {
        .sequence = generator->block.sequence,
        .interval_ns = generator->interval_base_ns,
        .step_mask = 0U,
        .positive_direction_mask = generator->block.positive_direction_mask,
        .last_in_block = 0U,
    };

    if (generator->interval_remainder_ns != 0U) {
        const uint64_t denominator = generator->block.step_event_rate_numerator;
        const uint64_t remainder = generator->interval_remainder_ns;
        if (generator->interval_remainder_accumulator >= denominator - remainder) {
            generator->interval_remainder_accumulator -= denominator - remainder;
            ++next_event.interval_ns;
        } else {
            generator->interval_remainder_accumulator += remainder;
        }
    }
    if (next_event.interval_ns == 0U) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
    }

    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const uint64_t axis_steps = generator->axis_steps[axis];
        if (axis_steps == 0U) {
            continue;
        }

        const uint64_t threshold = generator->block.master_step_count - axis_steps;
        if (generator->axis_error[axis] >= threshold) {
            generator->axis_error[axis] -= threshold;
            next_event.step_mask |= (uint8_t)(1U << axis);
        } else {
            generator->axis_error[axis] += axis_steps;
        }
    }

    ++generator->events_emitted;
    if (generator->events_emitted == generator->block.master_step_count) {
        next_event.last_in_block = 1U;
    }
    if (next_event.step_mask == 0U || (next_event.step_mask & (uint8_t)~generator->block.moving_axes_mask) != 0U) {
        return MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK;
    }

    *event = next_event;
    return MOTION_SEGMENT_GENERATOR_STATUS_OK;
}

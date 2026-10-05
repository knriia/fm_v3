#include "motion/motion_planner.h"

#include <stddef.h>
#include <stdint.h>

static uint64_t motion_planner_abs_i64(int64_t value) {
    return value < 0 ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
}

static uint64_t motion_planner_integer_sqrt(uint64_t value) {
    uint64_t result = 0U;
    uint64_t bit = 1ULL << 62U;

    while (bit > value) {
        bit >>= 2U;
    }

    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1U) + bit;
        } else {
            result >>= 1U;
        }
        bit >>= 2U;
    }

    return result;
}

static int motion_planner_checked_square_sum(const uint64_t values[MOTION_PLANNER_AXIS_COUNT], uint64_t *sum) {
    uint64_t total = 0U;

    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const uint64_t square = values[axis] * values[axis];
        if (UINT64_MAX - total < square) {
            return 0;
        }
        total += square;
    }

    *sum = total;
    return 1;
}

static int motion_planner_validate_config(const MotionPlannerConfig_t *config) {
    if (config->max_step_event_rate_hz == 0U) {
        return 0;
    }

    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const MotionPlannerAxisConfig_t *axis_config = &config->axis[axis];
        if (axis_config->min_position_um > axis_config->max_position_um || axis_config->steps_per_mm_numerator == 0U ||
            axis_config->steps_per_mm_denominator == 0U || axis_config->max_speed_um_per_s == 0U) {
            return 0;
        }
    }

    return 1;
}

static int64_t motion_planner_position_to_steps(int32_t position_um, const MotionPlannerAxisConfig_t *config) {
    const uint64_t magnitude_um = position_um < 0 ? (uint64_t)(-(int64_t)position_um) : (uint64_t)position_um;
    const uint64_t denominator = 1000U * (uint64_t)config->steps_per_mm_denominator;
    const uint64_t numerator_scale = config->steps_per_mm_numerator;
    const uint64_t rounding_offset = denominator / 2U;
    const uint64_t rounded_steps = (magnitude_um * numerator_scale + rounding_offset) / denominator;
    const int64_t signed_steps = (int64_t)rounded_steps;
    return position_um < 0 ? -signed_steps : signed_steps;
}

static int motion_planner_ratio_exceeds_limit(uint64_t numerator, uint64_t denominator, uint32_t limit) {
    const uint64_t quotient = numerator / denominator;
    const uint64_t remainder = numerator % denominator;
    return quotient > limit || (quotient == limit && remainder != 0U);
}

MotionPlannerResult_t motion_planner_plan_linear(
    const MotionPlannerRequest_t *request,
    const MotionPlannerMachineState_t *machine_state,
    const MotionPlannerConfig_t *config
) {
    MotionPlannerResult_t result = {0};
    result.status = MOTION_PLANNER_STATUS_INVALID_ARGUMENT;

    if (request == NULL || machine_state == NULL || config == NULL) {
        return result;
    }
    if ((machine_state->homed_axes_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U) {
        return result;
    }
    if (!motion_planner_validate_config(config)) {
        result.status = MOTION_PLANNER_STATUS_INVALID_CONFIGURATION;
        return result;
    }
    if ((machine_state->homed_axes_mask & MOTION_PLANNER_ALL_AXES_MASK) != MOTION_PLANNER_ALL_AXES_MASK) {
        result.status = MOTION_PLANNER_STATUS_NOT_HOMED;
        return result;
    }
    if (request->feedrate_um_per_s == 0U) {
        result.status = MOTION_PLANNER_STATUS_INVALID_FEEDRATE;
        return result;
    }

    int64_t start_steps[MOTION_PLANNER_AXIS_COUNT] = {0};
    int64_t target_steps[MOTION_PLANNER_AXIS_COUNT] = {0};
    int64_t delta_um[MOTION_PLANNER_AXIS_COUNT] = {0};
    uint64_t absolute_delta_um[MOTION_PLANNER_AXIS_COUNT] = {0};
    uint64_t distance_squared = 0U;
    MotionPlanBlock_t block = {0};

    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        const MotionPlannerAxisConfig_t *axis_config = &config->axis[axis];
        const int32_t current_um = machine_state->position_um[axis];
        const int32_t target_um = request->target_um[axis];

        if (current_um < axis_config->min_position_um || current_um > axis_config->max_position_um ||
            target_um < axis_config->min_position_um || target_um > axis_config->max_position_um) {
            result.status = MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE;
            return result;
        }

        start_steps[axis] = motion_planner_position_to_steps(current_um, axis_config);
        target_steps[axis] = motion_planner_position_to_steps(target_um, axis_config);

        delta_um[axis] = (int64_t)target_um - (int64_t)current_um;
        absolute_delta_um[axis] = motion_planner_abs_i64(delta_um[axis]);
        block.step_delta[axis] = target_steps[axis] - start_steps[axis];
        if (block.step_delta[axis] != 0) {
            const uint8_t axis_mask = (uint8_t)(1U << axis);
            block.moving_axes_mask |= axis_mask;
            if (block.step_delta[axis] > 0) {
                block.positive_direction_mask |= axis_mask;
            }
            const uint64_t absolute_steps = motion_planner_abs_i64(block.step_delta[axis]);
            if (absolute_steps > block.master_step_count) {
                block.master_step_count = absolute_steps;
            }
        }
    }

    if (block.moving_axes_mask == 0U) {
        result.status = MOTION_PLANNER_STATUS_NO_MOVEMENT;
        return result;
    }

    if (!motion_planner_checked_square_sum(absolute_delta_um, &distance_squared)) {
        result.status = MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW;
        return result;
    }

    block.path_length_um = motion_planner_integer_sqrt(distance_squared);

    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        if (absolute_delta_um[axis] == 0U) {
            continue;
        }

        const uint64_t component_speed_numerator = (uint64_t)request->feedrate_um_per_s * absolute_delta_um[axis];
        if (motion_planner_ratio_exceeds_limit(
                component_speed_numerator,
                block.path_length_um,
                config->axis[axis].max_speed_um_per_s
            )) {
            result.status = MOTION_PLANNER_STATUS_AXIS_SPEED_LIMIT_EXCEEDED;
            return result;
        }
    }

    if (block.master_step_count > UINT64_MAX / request->feedrate_um_per_s) {
        result.status = MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW;
        return result;
    }

    block.step_event_rate_numerator = (uint64_t)request->feedrate_um_per_s * block.master_step_count;
    block.step_event_rate_denominator = block.path_length_um;
    if (motion_planner_ratio_exceeds_limit(
            block.step_event_rate_numerator,
            block.step_event_rate_denominator,
            config->max_step_event_rate_hz
        )) {
        result.status = MOTION_PLANNER_STATUS_STEP_RATE_LIMIT_EXCEEDED;
        return result;
    }

    block.sequence = request->sequence;
    block.feedrate_um_per_s = request->feedrate_um_per_s;
    result.block = block;
    result.status = MOTION_PLANNER_STATUS_OK;
    return result;
}

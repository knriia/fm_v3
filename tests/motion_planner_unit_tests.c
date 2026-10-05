#include "motion/motion_planner.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %lu, got %lu)\n", message, (unsigned long)expected, (unsigned long)actual);
    }
}

static void expect_u64(uint64_t actual, uint64_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %llu, got %llu)\n", message, (unsigned long long)expected, (unsigned long long)actual);
    }
}

static void expect_i64(int64_t actual, int64_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %lld, got %lld)\n", message, (long long)expected, (long long)actual);
    }
}

static MotionPlannerConfig_t valid_config(void) {
    MotionPlannerConfig_t config = {0};
    config.max_step_event_rate_hz = 50000U;
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        config.axis[axis] = (MotionPlannerAxisConfig_t){
            .min_position_um = -100000,
            .max_position_um = 100000,
            .steps_per_mm_numerator = 80U,
            .steps_per_mm_denominator = 1U,
            .max_speed_um_per_s = 100000U,
        };
    }
    return config;
}

static MotionPlannerMachineState_t homed_origin(void) {
    return (MotionPlannerMachineState_t){
        .position_um = {0, 0, 0},
        .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
    };
}

static MotionPlannerRequest_t request_to(int32_t x_um, int32_t y_um, int32_t z_um, uint32_t feedrate_um_per_s) {
    return (MotionPlannerRequest_t){
        .sequence = 73U,
        .target_um = {x_um, y_um, z_um},
        .feedrate_um_per_s = feedrate_um_per_s,
    };
}

static void expect_status(
    MotionPlannerStatus_t actual,
    MotionPlannerStatus_t expected,
    const char *message
) {
    expect_u32((uint32_t)actual, (uint32_t)expected, message);
}

static void test_linear_plan_uses_physical_units_and_constant_feedrate(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[1].max_speed_um_per_s = 8000U;
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(30000, 40000, 0, 10000U);

    const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "valid diagonal move is planned");
    expect_u32(result.block.sequence, 73U, "plan preserves command sequence");
    expect_i64(result.block.step_delta[0], 2400, "X coordinate converts to steps");
    expect_i64(result.block.step_delta[1], 3200, "Y coordinate converts to steps");
    expect_i64(result.block.step_delta[2], 0, "stationary Z has no step delta");
    expect_u64(result.block.path_length_um, 50000U, "3-4-5 path length is calculated");
    expect_u64(result.block.master_step_count, 3200U, "master axis is the axis with most steps");
    expect_u64(result.block.step_event_rate_numerator, 32000000U, "event rate keeps exact numerator");
    expect_u64(result.block.step_event_rate_denominator, 50000U, "event rate keeps exact denominator");
    expect_u32(result.block.feedrate_um_per_s, 10000U, "requested constant feedrate is retained");
    expect_u32(result.block.moving_axes_mask, 3U, "moving axes are represented as a mask");
    expect_u32(result.block.positive_direction_mask, 3U, "positive directions are represented as a mask");
}

static void test_negative_and_positive_axis_directions(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = {
        .position_um = {1000, -1000, 0},
        .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
    };
    const MotionPlannerRequest_t request = request_to(-1000, 1000, 0, 10000U);

    const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "mixed direction move is planned");
    expect_i64(result.block.step_delta[0], -160, "negative X move has negative step delta");
    expect_i64(result.block.step_delta[1], 160, "positive Y move has positive step delta");
    expect_u32(result.block.moving_axes_mask, 3U, "both changing axes are marked");
    expect_u32(result.block.positive_direction_mask, MOTION_PLANNER_AXIS_Y_MASK, "only positive Y direction is marked");
}

static void test_no_motion_when_target_is_same_or_below_step_resolution(void) {
    MotionPlannerConfig_t config = valid_config();
    MotionPlannerMachineState_t state = homed_origin();
    MotionPlannerRequest_t request = request_to(0, 0, 0, 10000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_NO_MOVEMENT,
        "unchanged target is a no-op"
    );

    config.axis[0].steps_per_mm_numerator = 1U;
    request = request_to(1, 0, 0, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_NO_MOVEMENT,
        "target below one step of resolution is a no-op"
    );
}

static void test_rational_calibration_rounds_to_nearest_step(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].steps_per_mm_numerator = 1000U;
    config.axis[0].steps_per_mm_denominator = 1U;
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(1, 0, 0, 1000U);

    const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "sub-millimeter rational calibration is accepted");
    expect_i64(result.block.step_delta[0], 1, "one micrometer at 1000 steps per millimeter rounds to one step");
    expect_u64(result.block.master_step_count, 1U, "single-axis one-step move has one master event");
}

static void test_null_arguments_are_rejected(void) {
    MotionPlannerConfig_t config = valid_config();
    MotionPlannerMachineState_t state = homed_origin();
    MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);

    expect_status(
        motion_planner_plan_linear(NULL, &state, &config).status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "null request is rejected"
    );
    expect_status(
        motion_planner_plan_linear(&request, NULL, &config).status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "null machine state is rejected"
    );
    expect_status(
        motion_planner_plan_linear(&request, &state, NULL).status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "null configuration is rejected"
    );
}

static void test_homing_state_is_required_for_absolute_xyz(void) {
    MotionPlannerConfig_t config = valid_config();
    MotionPlannerMachineState_t state = homed_origin();
    MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);

    state.homed_axes_mask = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK;
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_NOT_HOMED,
        "absolute XYZ request requires all axes to be homed"
    );

    state.homed_axes_mask = (uint8_t)(MOTION_PLANNER_ALL_AXES_MASK | 0x80U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "reserved homed mask bits are rejected"
    );
}

static void test_invalid_configuration_fields_are_rejected(void) {
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);
    MotionPlannerConfig_t config = valid_config();

    config.max_step_event_rate_hz = 0U;
    expect_status(motion_planner_plan_linear(&request, &state, &config).status,
                  MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "zero global event rate limit is rejected");

    config = valid_config();
    config.axis[0].min_position_um = config.axis[0].max_position_um + 1;
    expect_status(motion_planner_plan_linear(&request, &state, &config).status,
                  MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "inverted axis range is rejected");

    config = valid_config();
    config.axis[0].steps_per_mm_numerator = 0U;
    expect_status(motion_planner_plan_linear(&request, &state, &config).status,
                  MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "zero calibration numerator is rejected");

    config = valid_config();
    config.axis[0].steps_per_mm_denominator = 0U;
    expect_status(motion_planner_plan_linear(&request, &state, &config).status,
                  MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "zero calibration denominator is rejected");

    config = valid_config();
    config.axis[0].max_speed_um_per_s = 0U;
    expect_status(motion_planner_plan_linear(&request, &state, &config).status,
                  MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "zero axis speed limit is rejected");
}

static void test_machine_position_and_target_must_be_within_limits(void) {
    MotionPlannerConfig_t config = valid_config();
    MotionPlannerMachineState_t state = homed_origin();
    MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);

    state.position_um[0] = 100001;
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
        "current position outside configured range is rejected"
    );

    state = homed_origin();
    request.target_um[0] = 100001;
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
        "target outside configured range is rejected"
    );

    state = homed_origin();
    state.position_um[0] = -100001;
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
        "current position below configured range is rejected"
    );

    state = homed_origin();
    request = request_to(-100001, 0, 0, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
        "target below configured range is rejected"
    );
}

static void test_zero_feedrate_is_rejected(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(1000, 0, 0, 0U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_INVALID_FEEDRATE,
        "zero feedrate is rejected"
    );
}

static void test_axis_speed_limit_includes_fractional_excess(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].steps_per_mm_numerator = 1000U;
    config.axis[0].max_speed_um_per_s = 6U;
    config.axis[1].steps_per_mm_numerator = 1000U;
    config.axis[1].max_speed_um_per_s = 9000U;
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(3, 4, 0, 10001U);

    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_AXIS_SPEED_LIMIT_EXCEEDED,
        "fractional component speed above axis limit is rejected"
    );
}

static void test_step_event_rate_limit_is_enforced(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(10000, 0, 0, 10000U);

    config.max_step_event_rate_hz = 800U;
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_OK,
        "event rate exactly at limit is accepted"
    );

    config.max_step_event_rate_hz = 799U;
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_STEP_RATE_LIMIT_EXCEEDED,
        "event rate above limit is rejected"
    );

    config.max_step_event_rate_hz = 800U;
    const MotionPlannerRequest_t fractional_request = request_to(3000, 0, 0, 10001U);
    expect_status(
        motion_planner_plan_linear(&fractional_request, &state, &config).status,
        MOTION_PLANNER_STATUS_STEP_RATE_LIMIT_EXCEEDED,
        "fractional event rate above limit is rejected"
    );
}

static void test_distance_square_sum_overflow_is_rejected(void) {
    MotionPlannerConfig_t config = valid_config();
    MotionPlannerMachineState_t state = {
        .position_um = {INT32_MIN, INT32_MIN, INT32_MIN},
        .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
    };
    MotionPlannerRequest_t request = {
        .sequence = 5U,
        .target_um = {INT32_MAX, INT32_MAX, INT32_MAX},
        .feedrate_um_per_s = 1U,
    };
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        config.axis[axis].min_position_um = INT32_MIN;
        config.axis[axis].max_position_um = INT32_MAX;
        config.axis[axis].steps_per_mm_numerator = 1U;
        config.axis[axis].max_speed_um_per_s = UINT32_MAX;
    }
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW,
        "overflow while summing squared axis distances is rejected"
    );
}

static void test_step_rate_numerator_overflow_is_rejected(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].min_position_um = 0;
    config.axis[0].max_position_um = INT32_MAX;
    config.axis[0].steps_per_mm_numerator = UINT32_MAX;
    config.axis[0].max_speed_um_per_s = UINT32_MAX;
    config.max_step_event_rate_hz = UINT32_MAX;
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(INT32_MAX, 0, 0, UINT32_MAX);

    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW,
        "event-rate numerator overflow is rejected"
    );
}

int main(void) {
    test_linear_plan_uses_physical_units_and_constant_feedrate();
    test_negative_and_positive_axis_directions();
    test_no_motion_when_target_is_same_or_below_step_resolution();
    test_rational_calibration_rounds_to_nearest_step();
    test_null_arguments_are_rejected();
    test_homing_state_is_required_for_absolute_xyz();
    test_invalid_configuration_fields_are_rejected();
    test_machine_position_and_target_must_be_within_limits();
    test_zero_feedrate_is_rejected();
    test_axis_speed_limit_includes_fractional_excess();
    test_step_event_rate_limit_is_enforced();
    test_distance_square_sum_overflow_is_rejected();
    test_step_rate_numerator_overflow_is_rejected();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Motion planner unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Motion planner unit tests passed.\n");
    return 0;
}

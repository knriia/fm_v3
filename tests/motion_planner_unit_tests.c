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
            .steps_per_um_numerator = 80U,
            .steps_per_um_denominator = 1000U,
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

    config.axis[0].steps_per_um_numerator = 1U;
    request = request_to(1, 0, 0, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_NO_MOVEMENT,
        "target below one step of resolution is a no-op"
    );
}

static void test_rational_calibration_rounds_to_nearest_step(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].steps_per_um_numerator = 1000U;
    config.axis[0].steps_per_um_denominator = 1000U;
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
        MOTION_PLANNER_STATUS_OK,
        "only the moving axis has to be homed"
    );

    request = request_to(1000, 0, 500, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_NOT_HOMED,
        "moving an axis that is not homed is rejected"
    );

    state.homed_axes_mask = MOTION_PLANNER_AXIS_X_MASK;
    config.axis[1] = (MotionPlannerAxisConfig_t){0};
    config.axis[2] = (MotionPlannerAxisConfig_t){0};
    request = request_to(1000, 0, 0, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_OK,
        "axes that do not move need no configuration"
    );
    request = request_to(1000, 0, 500, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "moving an axis without configuration is rejected"
    );
    config = valid_config();
    request = request_to(1000, 0, 0, 1000U);

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
    config.axis[0].steps_per_um_numerator = 0U;
    expect_status(motion_planner_plan_linear(&request, &state, &config).status,
                  MOTION_PLANNER_STATUS_INVALID_CONFIGURATION, "zero calibration numerator is rejected");

    config = valid_config();
    config.axis[0].steps_per_um_denominator = 0U;
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
        MOTION_PLANNER_STATUS_OK,
        "axis outside the usable travel may move back into it"
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
    request = request_to(1000, 0, 0, 1000U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_OK,
        "axis below the usable travel may move back into it"
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
    config.axis[0].steps_per_um_numerator = 1000U;
    config.axis[0].max_speed_um_per_s = 6U;
    config.axis[1].steps_per_um_numerator = 1000U;
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
        config.axis[axis].steps_per_um_numerator = 1U;
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
    config.axis[0].steps_per_um_numerator = UINT32_MAX;
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

static void set_syringe(MotionPlannerConfig_t *config, uint32_t numerator, uint32_t denominator) {
    config->syringe_steps_per_um_numerator = numerator;
    config->syringe_steps_per_um_denominator = denominator;
}

static void test_syringe_off_leaves_block_and_remainder_untouched(void) {
    MotionPlannerConfig_t config = valid_config();
    MotionPlannerMachineState_t state = homed_origin();
    state.syringe_step_remainder = 7U;
    const MotionPlannerRequest_t request = request_to(10000, 0, 0, 1000U);

    /* Numerator 0 is the off switch, whatever the denominator. */
    set_syringe(&config, 0U, 5U);
    MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "move without syringe is planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 0, "no syringe steps");
    expect_u32(result.block.moving_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "syringe bit not in the moving mask");
    expect_u32(
        result.block.positive_direction_mask,
        MOTION_PLANNER_AXIS_X_MASK,
        "syringe bit not in the direction mask"
    );
    expect_u32(result.syringe_step_remainder, 7U, "the remainder is carried over unchanged");
    expect_u64(result.block.master_step_count, 800U, "the X axis stays the master channel");

    set_syringe(&config, 0U, 0U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "zero numerator with zero denominator is no syringe");
    expect_u32(result.syringe_step_remainder, 7U, "remainder unchanged without a denominator");
}

static void test_syringe_steps_follow_the_path(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = homed_origin();
    set_syringe(&config, 1U, 2U);

    /* Even path: 1000 um * 1/2 = 500 steps, nothing left. */
    MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);
    MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "even path planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 500, "half a step per micrometre");
    expect_u32(result.block.moving_axes_mask, 0x09U, "X and the syringe move");
    expect_u32(result.block.positive_direction_mask, 0x09U, "the syringe direction is positive");
    expect_u32(result.syringe_step_remainder, 0U, "even path leaves no remainder");
    expect_i64(result.block.step_delta[0], 80, "the axis steps are unchanged");

    /* Odd path: 1001 um * 1/2 = 500 steps and one half-step (1 in units of 1/2). */
    request = request_to(1001, 0, 0, 1000U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "odd path planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 500, "the half step is not taken");
    expect_u32(result.syringe_step_remainder, 1U, "one half step is left");

    /* The path is the diagonal: 3-4-5 gives 5000 um, so 2500 steps. */
    request = request_to(3000, 4000, 0, 1000U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "diagonal planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 2500, "syringe follows the path, not one axis");
}

static void test_syringe_remainder_carries_over_blocks(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].steps_per_um_numerator = 1000U;
    set_syringe(&config, 1U, 3U);
    MotionPlannerMachineState_t state = homed_origin();

    /* Three blocks of 10 um at 1/3 step per um: 10/3 -> 3 r1, then 11/3 -> 3 r2, then 12/3 -> 4 r0. */
    MotionPlannerRequest_t request = request_to(10, 0, 0, 1000U);
    MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "first block planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 3, "first block steps");
    expect_u32(result.syringe_step_remainder, 1U, "first block remainder");

    state.position_um[0] = 10;
    state.syringe_step_remainder = result.syringe_step_remainder;
    request = request_to(20, 0, 0, 1000U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "second block planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 3, "second block steps");
    expect_u32(result.syringe_step_remainder, 2U, "second block remainder");

    state.position_um[0] = 20;
    state.syringe_step_remainder = result.syringe_step_remainder;
    request = request_to(30, 0, 0, 1000U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "third block planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 4, "third block takes the carried step");
    expect_u32(result.syringe_step_remainder, 0U, "third block closes the fraction");
}

static void test_syringe_path_shorter_than_a_step(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].steps_per_um_numerator = 1000U;
    set_syringe(&config, 1U, 100U);
    MotionPlannerMachineState_t state = homed_origin();
    state.syringe_step_remainder = 5U;
    const MotionPlannerRequest_t request = request_to(10, 0, 0, 1000U);

    const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "short path planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 0, "no whole syringe step in 10 um");
    expect_u32(result.block.moving_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "no syringe bit without steps");
    expect_u32(
        result.block.positive_direction_mask,
        MOTION_PLANNER_AXIS_X_MASK,
        "no syringe direction bit without steps"
    );
    expect_u32(result.syringe_step_remainder, 15U, "remainder grows by path * numerator: 5 + 10");
    expect_u64(result.block.master_step_count, 10U, "the axis is the master channel");
}

static void test_syringe_as_master_channel(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);

    /* X: 1000 um * 80/1000 = 80 steps. Syringe 2 steps per um: 2000 steps. */
    set_syringe(&config, 2U, 1U);
    MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "syringe-led block planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 2000, "syringe steps");
    expect_u64(result.block.master_step_count, 2000U, "the syringe is the master");
    expect_u64(result.block.step_event_rate_numerator, 2000000U, "rate numerator: 1000 um/s * 2000 steps");
    expect_u64(result.block.step_event_rate_denominator, 1000U, "rate denominator is the path");

    /* Syringe 1/1000 step per um: 1 step; X has 80 and stays the master. */
    set_syringe(&config, 1U, 1000U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "axis-led block planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 1, "syringe steps");
    expect_u64(result.block.master_step_count, 80U, "the axis is the master");
    expect_u32(result.block.moving_axes_mask, 0x09U, "both move");

    /* Equal step counts: 80 each, the master stays 80. */
    set_syringe(&config, 80U, 1000U);
    result = motion_planner_plan_linear(&request, &state, &config);
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 80, "equal syringe steps");
    expect_u64(result.block.master_step_count, 80U, "equal counts keep the master");
}

static void test_syringe_configuration_is_validated(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);

    set_syringe(&config, 1U, 0U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "syringe numerator without denominator is rejected"
    );
}

static void test_syringe_step_count_overflow_is_rejected(void) {
    MotionPlannerConfig_t config = valid_config();
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        config.axis[axis].min_position_um = INT32_MIN;
        config.axis[axis].max_position_um = INT32_MAX;
        config.axis[axis].steps_per_um_numerator = 1U;
        config.axis[axis].steps_per_um_denominator = 1U;
        config.axis[axis].max_speed_um_per_s = UINT32_MAX;
    }
    config.max_step_event_rate_hz = UINT32_MAX;
    const MotionPlannerMachineState_t state = {
        .position_um = {INT32_MIN, 0, 0},
        .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
    };
    /* The path is 2^32 - 1 um, the largest that fits the sum of squares. */
    const MotionPlannerRequest_t request = request_to(INT32_MAX, 0, 0, 1U);

    /* (2^32 - 1) * (2^32 - 1) = 18446744065119617025 steps: more than INT64_MAX. */
    set_syringe(&config, UINT32_MAX, 1U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW,
        "syringe steps above INT64_MAX are rejected"
    );

    /* (2^32 - 1) * 2^31 = 2^63 - 2^31 = 9223372034707292160 steps: still below INT64_MAX, accepted. */
    set_syringe(&config, 0x80000000U, 1U);
    const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "syringe steps below INT64_MAX are accepted");
    expect_i64(
        result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE],
        9223372034707292160LL,
        "syringe steps near the limit"
    );
    expect_u64(result.block.master_step_count, 9223372034707292160ULL, "the syringe is the master");

    /* (2^32 - 1) * (2^31 + 1) = 2^63 + 2^31 - 1 steps: above INT64_MAX, rejected. */
    set_syringe(&config, 0x80000001U, 1U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW,
        "syringe steps just above INT64_MAX are rejected"
    );
}

static void test_syringe_arithmetic_holds_at_the_maximum_inputs(void) {
    MotionPlannerConfig_t config = valid_config();
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        config.axis[axis].min_position_um = INT32_MIN;
        config.axis[axis].max_position_um = INT32_MAX;
        config.axis[axis].steps_per_um_numerator = 1U;
        config.axis[axis].steps_per_um_denominator = 1U;
        config.axis[axis].max_speed_um_per_s = UINT32_MAX;
    }
    config.max_step_event_rate_hz = UINT32_MAX;
    set_syringe(&config, UINT32_MAX, UINT32_MAX);
    MotionPlannerMachineState_t state = {
        .position_um = {INT32_MIN, 0, 0},
        .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
        .syringe_step_remainder = UINT32_MAX - 1U,
    };
    /* The path is 2^32 - 1 um. With N = D = 2^32 - 1 and the remainder r = D - 1 the sum path * N + r is
     * 2^64 - 2^32 - 1, below 2^64. It divides as path * D + r = path whole steps and the remainder r, as r < D. */
    const MotionPlannerRequest_t request = request_to(INT32_MAX, 0, 0, 1U);

    const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "the largest path with the largest factor is planned");
    expect_i64(result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE], 4294967295LL, "the syringe has one step per um");
    expect_u32(result.syringe_step_remainder, UINT32_MAX - 1U, "the remainder stays as it was");
    expect_u64(result.block.master_step_count, 4294967295U, "the master has 2^32 - 1 steps");
}

static void test_syringe_remainder_without_steps_builds_up_over_a_chain(void) {
    MotionPlannerConfig_t config = valid_config();
    config.axis[0].steps_per_um_numerator = 1000U;
    set_syringe(&config, 1U, 100U);
    MotionPlannerMachineState_t state = homed_origin();

    /* A step needs 100 um at 1/100 step per um; each block of 10 um adds 10 to the remainder. */
    for (uint32_t call = 1U; call <= 10U; ++call) {
        const MotionPlannerRequest_t request = request_to((int32_t)(10U * call), 0, 0, 1000U);
        const MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
        expect_status(result.status, MOTION_PLANNER_STATUS_OK, "the block of the chain is planned");
        expect_i64(
            result.block.step_delta[MOTION_PLAN_CHANNEL_SYRINGE],
            call == 10U ? 1 : 0,
            "only the tenth block has a syringe step"
        );
        expect_u32(result.syringe_step_remainder, call == 10U ? 0U : 10U * call, "the remainder after the block");
        state.position_um[0] = (int32_t)(10U * call);
        state.syringe_step_remainder = result.syringe_step_remainder;
    }
}

static void test_syringe_rate_limit_applies_to_the_syringe_channel(void) {
    MotionPlannerConfig_t config = valid_config();
    const MotionPlannerMachineState_t state = homed_origin();
    const MotionPlannerRequest_t request = request_to(1000, 0, 0, 1000U);

    /* X alone is 80 steps in 1 s = 80 Hz; the syringe at 50 steps per um is 50000 steps = 50000 Hz. */
    set_syringe(&config, 50U, 1U);
    MotionPlannerResult_t result = motion_planner_plan_linear(&request, &state, &config);
    expect_status(result.status, MOTION_PLANNER_STATUS_OK, "syringe rate exactly at the limit is accepted");
    expect_u64(result.block.master_step_count, 50000U, "the syringe is the master");

    set_syringe(&config, 51U, 1U);
    expect_status(
        motion_planner_plan_linear(&request, &state, &config).status,
        MOTION_PLANNER_STATUS_STEP_RATE_LIMIT_EXCEEDED,
        "syringe rate above the limit is rejected"
    );
}

int main(void) {
    test_syringe_off_leaves_block_and_remainder_untouched();
    test_syringe_steps_follow_the_path();
    test_syringe_remainder_carries_over_blocks();
    test_syringe_path_shorter_than_a_step();
    test_syringe_as_master_channel();
    test_syringe_configuration_is_validated();
    test_syringe_step_count_overflow_is_rejected();
    test_syringe_arithmetic_holds_at_the_maximum_inputs();
    test_syringe_remainder_without_steps_builds_up_over_a_chain();
    test_syringe_rate_limit_applies_to_the_syringe_channel();
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

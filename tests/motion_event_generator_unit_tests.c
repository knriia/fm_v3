#include "motion/motion_event_generator.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>

static uint32_t test_failures;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

static void expect_u64(uint64_t actual, uint64_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %llu, got %llu)\n",
            message,
            (unsigned long long)expected,
            (unsigned long long)actual
        );
    }
}

static MotionPlanBlock_t make_three_step_block(void) {
    return (MotionPlanBlock_t){
        .sequence = 73U,
        .step_delta = {3, -2, 1},
        .path_length_um = 1U,
        .master_step_count = 3U,
        .step_event_rate_numerator = 3U,
        .step_event_rate_denominator = 1U,
        .feedrate_um_per_s = 1U,
        .moving_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
        .positive_direction_mask = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
    };
}

static void expect_invalid_block(const MotionPlanBlock_t *block, const char *message) {
    MotionEventGenerator_t generator = {0};
    expect_u32(motion_event_generator_init(&generator, block), MOTION_EVENT_GENERATOR_STATUS_INVALID_BLOCK, message);
}

static void test_generates_exact_axis_steps_and_distributes_fractional_time(void) {
    const MotionPlanBlock_t block = make_three_step_block();
    MotionEventGenerator_t generator = {0};
    expect_u32(
        motion_event_generator_init(&generator, &block),
        MOTION_EVENT_GENERATOR_STATUS_OK,
        "valid constant-rate block initializes"
    );

    const uint8_t expected_masks[] = {
        MOTION_PLANNER_AXIS_X_MASK,
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
        MOTION_PLANNER_ALL_AXES_MASK,
    };
    const uint64_t expected_intervals_ns[] = {333333333U, 333333333U, 333333334U};
    uint32_t axis_step_totals[MOTION_PLANNER_AXIS_COUNT] = {0U};
    uint64_t total_interval_ns = 0U;

    for (uint32_t index = 0U; index < 3U; ++index) {
        MotionEvent_t event = {0};
        expect_u32(
            motion_event_generator_next(&generator, &event),
            MOTION_EVENT_GENERATOR_STATUS_OK,
            "generator yields each master event"
        );
        expect_u32(event.sequence, block.sequence, "event preserves source sequence");
        expect_u32(event.step_mask, expected_masks[index], "Bresenham event has expected axis mask");
        expect_u32(event.positive_direction_mask, block.positive_direction_mask, "event preserves movement direction");
        expect_u64(
            event.interval_ns,
            expected_intervals_ns[index],
            "fractional interval is distributed by accumulator"
        );
        total_interval_ns += event.interval_ns;
        for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
            if ((event.step_mask & (uint8_t)(1U << axis)) != 0U) {
                ++axis_step_totals[axis];
            }
        }
    }

    expect_u32(axis_step_totals[0], 3U, "X receives its exact step count");
    expect_u32(axis_step_totals[1], 2U, "Y receives its exact step count");
    expect_u32(axis_step_totals[2], 1U, "Z receives its exact step count");
    expect_u64(total_interval_ns, 1000000000U, "three 3-Hz intervals total exactly one second");
    MotionEvent_t completed_event = {0};
    expect_u32(
        motion_event_generator_next(&generator, &completed_event),
        MOTION_EVENT_GENERATOR_STATUS_COMPLETE,
        "generator reports completion after the final master event"
    );
}

static void test_single_negative_axis_move(void) {
    MotionPlanBlock_t block = make_three_step_block();
    block.sequence = 99U;
    block.step_delta[0] = -1;
    block.step_delta[1] = 0;
    block.step_delta[2] = 0;
    block.path_length_um = 2U;
    block.master_step_count = 1U;
    block.step_event_rate_numerator = 2U;
    block.step_event_rate_denominator = 2U;
    block.feedrate_um_per_s = 2U;
    block.moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK;
    block.positive_direction_mask = 0U;

    MotionEventGenerator_t generator = {0};
    expect_u32(
        motion_event_generator_init(&generator, &block),
        MOTION_EVENT_GENERATOR_STATUS_OK,
        "single-axis negative block initializes"
    );
    MotionEvent_t event = {0};
    expect_u32(
        motion_event_generator_next(&generator, &event),
        MOTION_EVENT_GENERATOR_STATUS_OK,
        "single-axis block yields its event"
    );
    expect_u32(event.step_mask, MOTION_PLANNER_AXIS_X_MASK, "negative single-axis event steps X");
    expect_u32(event.positive_direction_mask, 0U, "negative event has no positive direction bits");
    expect_u64(event.interval_ns, 1000000000U, "one-Hz event interval is one second");
}

static void test_invalid_arguments_and_uninitialized_state(void) {
    MotionPlanBlock_t block = make_three_step_block();
    MotionEventGenerator_t generator = {0};
    MotionEvent_t event = {0};

    expect_u32(
        motion_event_generator_init(NULL, &block),
        MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT,
        "null generator is rejected"
    );
    expect_u32(
        motion_event_generator_init(&generator, NULL),
        MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT,
        "null block is rejected"
    );
    expect_u32(
        motion_event_generator_next(NULL, &event),
        MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT,
        "null generator is rejected when requesting an event"
    );
    expect_u32(
        motion_event_generator_next(&generator, NULL),
        MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT,
        "null event output is rejected"
    );
    expect_u32(
        motion_event_generator_next(&generator, &event),
        MOTION_EVENT_GENERATOR_STATUS_NOT_INITIALIZED,
        "event cannot be requested before initialization"
    );
}

static void test_invalid_plan_blocks_are_rejected(void) {
    MotionPlanBlock_t block = make_three_step_block();
    block.master_step_count = 0U;
    expect_invalid_block(&block, "zero master step count is rejected");

    block = make_three_step_block();
    block.path_length_um = 0U;
    block.step_event_rate_denominator = 0U;
    expect_invalid_block(&block, "zero path length is rejected");

    block = make_three_step_block();
    block.feedrate_um_per_s = 0U;
    expect_invalid_block(&block, "zero feedrate is rejected");

    block = make_three_step_block();
    block.step_event_rate_numerator = 0U;
    expect_invalid_block(&block, "zero event rate numerator is rejected");

    block = make_three_step_block();
    block.step_event_rate_denominator = 0U;
    expect_invalid_block(&block, "zero event rate denominator is rejected");

    block = make_three_step_block();
    block.step_event_rate_denominator = 2U;
    expect_invalid_block(&block, "event rate denominator must match path length");

    block = make_three_step_block();
    block.moving_axes_mask = 0U;
    expect_invalid_block(&block, "empty moving-axis mask is rejected");

    block = make_three_step_block();
    block.moving_axes_mask |= 0x80U;
    expect_invalid_block(&block, "reserved moving-axis bits are rejected");

    block = make_three_step_block();
    block.positive_direction_mask |= 0x80U;
    expect_invalid_block(&block, "reserved direction bits are rejected");

    block = make_three_step_block();
    block.moving_axes_mask &= (uint8_t)~MOTION_PLANNER_AXIS_Z_MASK;
    expect_invalid_block(&block, "direction bits cannot refer to a nonmoving axis");

    block = make_three_step_block();
    block.positive_direction_mask &= (uint8_t)~MOTION_PLANNER_AXIS_X_MASK;
    expect_invalid_block(&block, "positive step delta requires positive direction bit");

    block = make_three_step_block();
    block.step_delta[0] = 0;
    expect_invalid_block(&block, "moving axis requires nonzero step delta");

    block = make_three_step_block();
    block.moving_axes_mask &= (uint8_t)~MOTION_PLANNER_AXIS_Z_MASK;
    block.positive_direction_mask &= (uint8_t)~MOTION_PLANNER_AXIS_Z_MASK;
    expect_invalid_block(&block, "nonmoving axis requires zero step delta");

    block = make_three_step_block();
    block.positive_direction_mask |= MOTION_PLANNER_AXIS_Y_MASK;
    expect_invalid_block(&block, "negative step delta cannot have positive direction bit");

    block = make_three_step_block();
    block.step_delta[0] = 4;
    expect_invalid_block(&block, "axis step delta cannot exceed master count");

    block = make_three_step_block();
    block.master_step_count = 4U;
    expect_invalid_block(&block, "master count must equal the largest axis delta");

    block = make_three_step_block();
    block.step_delta[0] = INT64_MAX;
    block.step_delta[1] = 0;
    block.step_delta[2] = 0;
    block.master_step_count = INT64_MAX;
    block.moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK;
    block.positive_direction_mask = MOTION_PLANNER_AXIS_X_MASK;
    block.feedrate_um_per_s = 3U;
    block.step_event_rate_numerator = 3U;
    expect_invalid_block(&block, "rate numerator multiplication overflow is rejected");

    block = make_three_step_block();
    block.step_event_rate_numerator = 4U;
    expect_invalid_block(&block, "rate numerator must match feedrate and master count");

    block = make_three_step_block();
    block.path_length_um = UINT64_MAX / MOTION_EVENT_GENERATOR_NANOSECONDS_PER_SECOND + 1U;
    block.step_event_rate_denominator = block.path_length_um;
    expect_invalid_block(&block, "nanosecond interval scaling overflow is rejected");

    block = make_three_step_block();
    block.master_step_count = 1U;
    block.step_delta[0] = 1;
    block.step_delta[1] = 0;
    block.step_delta[2] = 0;
    block.moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK;
    block.positive_direction_mask = MOTION_PLANNER_AXIS_X_MASK;
    block.feedrate_um_per_s = 1000000001U;
    block.path_length_um = 1U;
    block.step_event_rate_numerator = block.feedrate_um_per_s;
    block.step_event_rate_denominator = 1U;
    expect_invalid_block(&block, "sub-nanosecond event period is rejected");
}

int main(void) {
    test_generates_exact_axis_steps_and_distributes_fractional_time();
    test_single_negative_axis_move();
    test_invalid_arguments_and_uninitialized_state();
    test_invalid_plan_blocks_are_rejected();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Motion event generator tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Motion event generator tests passed.\n");
    return 0;
}

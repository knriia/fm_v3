#include "motion/home_block.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static MotionAxisHardwareConfig_t test_config(uint8_t positive_moves_toward_end) {
    MotionAxisHardwareConfig_t config;
    (void)memset(&config, 0, sizeof(config));
    config.positive_moves_toward_end = positive_moves_toward_end;
    return config;
}

static void test_rejects_bad_arguments(void) {
    const MotionAxisHardwareConfig_t config = test_config(1U);
    MotionPlanBlock_t block;
    (void)memset(&block, 0x5A, sizeof(block));
    MotionPlanBlock_t untouched = block;

    assert(home_block_make(MOTION_AXIS_X, NULL, AXIS_BENCH_TOWARD_START, 100U, 1000U, &block) == 0U);
    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 100U, 1000U, NULL) == 0U);
    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 0U, 1000U, &block) == 0U);
    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 100U, 0U, &block) == 0U);
    assert(home_block_make(MOTION_AXIS_HARDWARE_COUNT, &config, AXIS_BENCH_TOWARD_START, 100U, 1000U, &block) == 0U);
    assert(memcmp(&block, &untouched, sizeof(block)) == 0);
}

static void test_block_describes_a_constant_rate_move_of_x(void) {
    const MotionAxisHardwareConfig_t config = test_config(1U);
    MotionPlanBlock_t block;

    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1000000U, 2000U, &block) == 1U);
    assert(block.step_delta[0] == 1000000);
    assert(block.step_delta[1] == 0 && block.step_delta[2] == 0);
    assert(block.master_step_count == 1000000U);
    assert(block.step_event_rate_numerator == 2000U);
    assert(block.step_event_rate_denominator == 1U);
    assert(block.moving_axes_mask == MOTION_PLANNER_AXIS_X_MASK);
    assert(block.positive_direction_mask == MOTION_PLANNER_AXIS_X_MASK);
}

static void test_block_describes_a_constant_rate_move_of_y(void) {
    const MotionAxisHardwareConfig_t config = test_config(1U);
    MotionPlanBlock_t block;

    assert(home_block_make(MOTION_AXIS_Y, &config, AXIS_BENCH_TOWARD_START, 700U, 3000U, &block) == 1U);
    assert(block.step_delta[0] == 0 && block.step_delta[2] == 0);
    assert(block.step_delta[1] == -700);
    assert(block.master_step_count == 700U);
    assert(block.step_event_rate_numerator == 3000U);
    assert(block.moving_axes_mask == MOTION_PLANNER_AXIS_Y_MASK);
    assert(block.positive_direction_mask == 0U);

    assert(home_block_make(MOTION_AXIS_Y, &config, AXIS_BENCH_TOWARD_END, 700U, 3000U, &block) == 1U);
    assert(block.step_delta[1] == 700);
    assert(block.positive_direction_mask == MOTION_PLANNER_AXIS_Y_MASK);
}

static void test_direction_follows_where_the_end_switch_is(void) {
    MotionPlanBlock_t block;

    /* Positive motion toward the end switch: the start switch is reached by moving negative. */
    MotionAxisHardwareConfig_t config = test_config(1U);
    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 500U, 100U, &block) == 1U);
    assert(block.step_delta[0] == -500);
    assert(block.positive_direction_mask == 0U);

    /* Negative motion toward the end switch: the other way round. */
    config = test_config(0U);
    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 500U, 100U, &block) == 1U);
    assert(block.step_delta[0] == 500);
    assert(block.positive_direction_mask == MOTION_PLANNER_AXIS_X_MASK);
    assert(home_block_make(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 500U, 100U, &block) == 1U);
    assert(block.step_delta[0] == -500);
    assert(block.positive_direction_mask == 0U);
}

int main(void) {
    test_rejects_bad_arguments();
    test_block_describes_a_constant_rate_move_of_x();
    test_block_describes_a_constant_rate_move_of_y();
    test_direction_follows_where_the_end_switch_is();
    return 0;
}

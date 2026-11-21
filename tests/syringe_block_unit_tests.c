#include "motion/syringe_block.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static void expect_only_syringe_channel(const MotionPlanBlock_t *block) {
    assert(block->step_delta[0] == 0);
    assert(block->step_delta[1] == 0);
    assert(block->step_delta[2] == 0);
    assert(block->order == 0U);
    assert(block->path_length_um == 0U);
    assert(block->feedrate_um_per_s == 0U);
    assert(block->moving_axes_mask == 0x08U);
    assert(block->step_event_rate_denominator == 1U);
}

static void test_feed_block_describes_a_constant_rate_move(void) {
    MotionPlanBlock_t block;
    (void)memset(&block, 0x5A, sizeof(block));

    assert(syringe_block_make(42U, 1500, 800U, 2000U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.sequence == 42U);
    assert(block.step_delta[3] == 1500);
    assert(block.master_step_count == 1500U);
    assert(block.step_event_rate_numerator == 800U);
    assert(block.positive_direction_mask == 0x08U);
    expect_only_syringe_channel(&block);
}

static void test_retract_block_keeps_the_sign_and_clears_direction(void) {
    MotionPlanBlock_t block;
    (void)memset(&block, 0x5A, sizeof(block));

    assert(syringe_block_make(7U, -250, 100U, 2000U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.sequence == 7U);
    assert(block.step_delta[3] == -250);
    assert(block.master_step_count == 250U);
    assert(block.step_event_rate_numerator == 100U);
    assert(block.positive_direction_mask == 0U);
    expect_only_syringe_channel(&block);
}

static void test_extreme_step_counts(void) {
    MotionPlanBlock_t block;

    assert(syringe_block_make(1U, INT32_MAX, 10U, 10U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.step_delta[3] == 2147483647);
    assert(block.master_step_count == 2147483647U);
    assert(block.positive_direction_mask == 0x08U);

    /* The magnitude of INT32_MIN does not fit int32_t: it must be 2^31, not wrapped. */
    assert(syringe_block_make(2U, INT32_MIN, 10U, 10U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.step_delta[3] == -2147483648LL);
    assert(block.master_step_count == 2147483648ULL);
    assert(block.positive_direction_mask == 0U);

    assert(syringe_block_make(3U, 1, 10U, 10U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.step_delta[3] == 1);
    assert(block.master_step_count == 1U);

    assert(syringe_block_make(0xFFFFFFFFU, -1, 10U, 10U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.sequence == 0xFFFFFFFFU);
    assert(block.step_delta[3] == -1);
    assert(block.master_step_count == 1U);
}

static void test_rate_limit_boundary(void) {
    MotionPlanBlock_t block;
    MotionPlanBlock_t untouched;
    (void)memset(&block, 0x5A, sizeof(block));
    untouched = block;

    assert(syringe_block_make(1U, 10, 5000U, 5000U, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.step_event_rate_numerator == 5000U);

    (void)memset(&block, 0x5A, sizeof(block));
    assert(syringe_block_make(1U, 10, 5001U, 5000U, &block) == SYRINGE_BLOCK_STATUS_RATE_LIMIT_EXCEEDED);
    assert(memcmp(&block, &untouched, sizeof(block)) == 0);
}

static void test_zero_limit_refuses_every_rate(void) {
    MotionPlanBlock_t block;
    MotionPlanBlock_t untouched;
    (void)memset(&block, 0x5A, sizeof(block));
    untouched = block;

    assert(syringe_block_make(1U, 10, 1U, 0U, &block) == SYRINGE_BLOCK_STATUS_RATE_LIMIT_EXCEEDED);
    assert(syringe_block_make(1U, -10, UINT32_MAX, 0U, &block) == SYRINGE_BLOCK_STATUS_RATE_LIMIT_EXCEEDED);
    assert(memcmp(&block, &untouched, sizeof(block)) == 0);
}

static void test_maximum_rate_with_maximum_limit(void) {
    MotionPlanBlock_t block;

    assert(syringe_block_make(1U, 5, UINT32_MAX, UINT32_MAX, &block) == SYRINGE_BLOCK_STATUS_OK);
    assert(block.step_event_rate_numerator == 4294967295ULL);
}

static void test_zero_step_delta_is_no_movement(void) {
    MotionPlanBlock_t block;
    MotionPlanBlock_t untouched;
    (void)memset(&block, 0x5A, sizeof(block));
    untouched = block;

    assert(syringe_block_make(1U, 0, 100U, 1000U, &block) == SYRINGE_BLOCK_STATUS_NO_MOVEMENT);
    assert(memcmp(&block, &untouched, sizeof(block)) == 0);
}

static void test_invalid_arguments_do_not_write_the_block(void) {
    MotionPlanBlock_t block;
    MotionPlanBlock_t untouched;
    (void)memset(&block, 0x5A, sizeof(block));
    untouched = block;

    assert(syringe_block_make(1U, 10, 0U, 1000U, &block) == SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT);
    assert(memcmp(&block, &untouched, sizeof(block)) == 0);
    assert(syringe_block_make(1U, 10, 100U, 1000U, NULL) == SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT);
}

static void test_error_priority(void) {
    MotionPlanBlock_t block;

    /* A zero rate wins over a zero step count. */
    assert(syringe_block_make(1U, 0, 0U, 1000U, &block) == SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT);
    /* A NULL block wins over a zero step count. */
    assert(syringe_block_make(1U, 0, 100U, 1000U, NULL) == SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT);
    /* A zero step count wins over a rate above the limit. */
    assert(syringe_block_make(1U, 0, 2000U, 1000U, &block) == SYRINGE_BLOCK_STATUS_NO_MOVEMENT);
    /* A zero rate wins over a zero limit. */
    assert(syringe_block_make(1U, 10, 0U, 0U, &block) == SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT);
}

int main(void) {
    test_feed_block_describes_a_constant_rate_move();
    test_retract_block_keeps_the_sign_and_clears_direction();
    test_extreme_step_counts();
    test_rate_limit_boundary();
    test_zero_limit_refuses_every_rate();
    test_maximum_rate_with_maximum_limit();
    test_zero_step_delta_is_no_movement();
    test_invalid_arguments_do_not_write_the_block();
    test_error_priority();
    return 0;
}

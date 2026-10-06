#include "motion/execution_buffer.h"
#include "motion/motion_segment_generator.h"
#include "motion/step_executor.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static MotionPlanBlock_t make_block(
    int64_t x_steps,
    int64_t y_steps,
    uint64_t master_steps,
    uint64_t rate_numerator,
    uint64_t rate_denominator
) {
    MotionPlanBlock_t block = {
        .sequence = 42U,
        .step_delta = {x_steps, y_steps, 0},
        .path_length_um = 1U,
        .master_step_count = master_steps,
        .step_event_rate_numerator = rate_numerator,
        .step_event_rate_denominator = rate_denominator,
        .feedrate_um_per_s = 1U,
        .moving_axes_mask = 0U,
        .positive_direction_mask = 0U,
    };
    if (x_steps != 0) {
        block.moving_axes_mask |= MOTION_PLANNER_AXIS_X_MASK;
        if (x_steps > 0) {
            block.positive_direction_mask |= MOTION_PLANNER_AXIS_X_MASK;
        }
    }
    if (y_steps != 0) {
        block.moving_axes_mask |= MOTION_PLANNER_AXIS_Y_MASK;
        if (y_steps > 0) {
            block.positive_direction_mask |= MOTION_PLANNER_AXIS_Y_MASK;
        }
    }
    return block;
}

static void test_generator_bresenham_and_directions(void) {
    MotionPlanBlock_t block = make_block(7, -4, 7U, 3U, 1U);
    MotionSegmentGenerator_t generator = {0};
    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_OK);

    uint32_t x_count = 0U;
    uint32_t y_count = 0U;
    uint64_t interval_sum = 0U;
    for (uint32_t index = 0U; index < 7U; ++index) {
        MotionExecutionEvent_t event = {0};
        assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
        assert(event.sequence == 42U);
        assert(event.step_mask == (MOTION_PLANNER_AXIS_X_MASK | (event.step_mask & MOTION_PLANNER_AXIS_Y_MASK)));
        assert((event.positive_direction_mask & MOTION_PLANNER_AXIS_X_MASK) != 0U);
        assert((event.positive_direction_mask & MOTION_PLANNER_AXIS_Y_MASK) == 0U);
        assert(event.last_in_block == (index == 6U ? 1U : 0U));
        x_count += (event.step_mask & MOTION_PLANNER_AXIS_X_MASK) != 0U;
        y_count += (event.step_mask & MOTION_PLANNER_AXIS_Y_MASK) != 0U;
        interval_sum += event.interval_ns;
    }

    assert(x_count == 7U);
    assert(y_count == 4U);
    assert(interval_sum == 2333333333ULL);
    MotionExecutionEvent_t unused = {0};
    assert(motion_segment_generator_next(&generator, &unused) == MOTION_SEGMENT_GENERATOR_STATUS_DONE);
}

static void test_generator_validation_and_fractional_timing(void) {
    MotionPlanBlock_t block = make_block(-3, 0, 3U, 3U, 1U);
    MotionSegmentGenerator_t generator = {0};
    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_OK);

    uint64_t interval_sum = 0U;
    for (uint32_t index = 0U; index < 3U; ++index) {
        MotionExecutionEvent_t event = {0};
        assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
        assert(event.step_mask == MOTION_PLANNER_AXIS_X_MASK);
        assert(event.positive_direction_mask == 0U);
        interval_sum += event.interval_ns;
    }
    assert(interval_sum == 1000000000ULL);

    block.master_step_count = 0U;
    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK);
    block = make_block(1, 0, 1U, 1000000001ULL, 1U);
    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK);
    block = make_block(1, 0, 1U, 1U, UINT64_MAX);
    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_ARITHMETIC_OVERFLOW);
    assert(motion_segment_generator_init(NULL, &block) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT);
}

static void expect_invalid_block(const MotionPlanBlock_t *block) {
    MotionSegmentGenerator_t generator = {0};
    assert(motion_segment_generator_init(&generator, block) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK);
    assert(generator.initialized == 0U);
}

static void test_generator_rejects_inconsistent_blocks(void) {
    MotionPlanBlock_t block = make_block(2, 0, 2U, 1U, 1U);
    block.step_event_rate_numerator = 0U;
    expect_invalid_block(&block);

    block = make_block(2, 0, 2U, 1U, 1U);
    block.step_event_rate_denominator = 0U;
    expect_invalid_block(&block);

    block = make_block(2, 0, 2U, 1U, 1U);
    block.moving_axes_mask = 0U;
    block.positive_direction_mask = 0U;
    expect_invalid_block(&block);

    block = make_block(2, 0, 2U, 1U, 1U);
    block.moving_axes_mask |= 0x80U;
    expect_invalid_block(&block);

    block = make_block(2, 0, 2U, 1U, 1U);
    block.positive_direction_mask |= MOTION_PLANNER_AXIS_Y_MASK;
    expect_invalid_block(&block);

    block = make_block(3, 0, 2U, 1U, 1U);
    expect_invalid_block(&block);

    block = make_block(2, -1, 2U, 1U, 1U);
    block.moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK;
    expect_invalid_block(&block);

    block = make_block(2, 0, 2U, 1U, 1U);
    block.positive_direction_mask = 0U;
    expect_invalid_block(&block);

    block = make_block(2, 0, 3U, 1U, 1U);
    expect_invalid_block(&block);

    MotionSegmentGenerator_t generator = {0};
    assert(motion_segment_generator_init(&generator, NULL) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT);
}

static void test_generator_next_guards_and_exact_interval(void) {
    MotionSegmentGenerator_t generator = {0};
    MotionExecutionEvent_t event = {0};
    assert(motion_segment_generator_next(NULL, &event) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT);
    assert(motion_segment_generator_next(&generator, NULL) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT);
    assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT);

    /* 1 step/s divides one second exactly, so no remainder correction is applied. */
    MotionPlanBlock_t block = make_block(1, 0, 1U, 1U, 1U);
    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
    assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
    assert(event.interval_ns == 1000000000ULL);
    assert(event.last_in_block == 1U);
}

static void test_generator_detects_corrupted_state(void) {
    const MotionPlanBlock_t block = make_block(2, 0, 2U, 1U, 1U);
    MotionSegmentGenerator_t generator = {0};
    MotionExecutionEvent_t event = {0};

    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
    generator.interval_base_ns = 0U;
    assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK);

    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
    generator.axis_steps[0] = 0U;
    assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK);

    assert(motion_segment_generator_init(&generator, &block) == MOTION_SEGMENT_GENERATOR_STATUS_OK);
    generator.block.moving_axes_mask = MOTION_PLANNER_AXIS_Y_MASK;
    assert(motion_segment_generator_next(&generator, &event) == MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK);
}

static void test_execution_buffer_capacity_wrap_and_clear(void) {
    execution_buffer_init();
    MotionExecutionEvent_t event = {.interval_ns = 1000U, .sequence = 1U, .step_mask = 1U};
    MotionExecutionEvent_t output = {0};
    assert(execution_buffer_try_pop(&output) == EXECUTION_BUFFER_STATUS_EMPTY);
    assert(execution_buffer_try_push(NULL) == EXECUTION_BUFFER_STATUS_INVALID_ARGUMENT);
    assert(execution_buffer_try_pop(NULL) == EXECUTION_BUFFER_STATUS_INVALID_ARGUMENT);

    for (uint32_t index = 0U; index < EXECUTION_BUFFER_CAPACITY; ++index) {
        event.sequence = index;
        assert(execution_buffer_try_push(&event) == EXECUTION_BUFFER_STATUS_OK);
    }
    assert(execution_buffer_count() == EXECUTION_BUFFER_CAPACITY);
    assert(execution_buffer_try_push(&event) == EXECUTION_BUFFER_STATUS_FULL);

    for (uint32_t index = 0U; index < EXECUTION_BUFFER_CAPACITY / 2U; ++index) {
        assert(execution_buffer_try_pop(&output) == EXECUTION_BUFFER_STATUS_OK);
        assert(output.sequence == index);
    }
    for (uint32_t index = 0U; index < EXECUTION_BUFFER_CAPACITY / 2U; ++index) {
        event.sequence = EXECUTION_BUFFER_CAPACITY + index;
        assert(execution_buffer_try_push(&event) == EXECUTION_BUFFER_STATUS_OK);
    }
    for (uint32_t index = EXECUTION_BUFFER_CAPACITY / 2U; index < EXECUTION_BUFFER_CAPACITY; ++index) {
        assert(execution_buffer_try_pop(&output) == EXECUTION_BUFFER_STATUS_OK);
        assert(output.sequence == index);
    }
    for (uint32_t index = 0U; index < EXECUTION_BUFFER_CAPACITY / 2U; ++index) {
        assert(execution_buffer_try_pop(&output) == EXECUTION_BUFFER_STATUS_OK);
        assert(output.sequence == EXECUTION_BUFFER_CAPACITY + index);
    }
    assert(execution_buffer_count() == 0U);

    assert(execution_buffer_try_push(&event) == EXECUTION_BUFFER_STATUS_OK);
    execution_buffer_clear();
    assert(execution_buffer_try_pop(&output) == EXECUTION_BUFFER_STATUS_EMPTY);
}

static MotionExecutionEvent_t make_event(uint32_t sequence, uint8_t last) {
    return (MotionExecutionEvent_t){
        .interval_ns = 1000000U,
        .sequence = sequence,
        .step_mask = MOTION_PLANNER_AXIS_X_MASK,
        .positive_direction_mask = MOTION_PLANNER_AXIS_X_MASK,
        .last_in_block = last,
    };
}

static void test_step_executor_completion_and_underrun(void) {
    execution_buffer_clear();
    step_executor_reset();
    MotionExecutionEvent_t first = make_event(1U, 0U);
    MotionExecutionEvent_t second = make_event(1U, 1U);
    MotionExecutionEvent_t output = {0};
    assert(execution_buffer_try_push(&first) == EXECUTION_BUFFER_STATUS_OK);
    assert(execution_buffer_try_push(&second) == EXECUTION_BUFFER_STATUS_OK);
    assert(step_executor_begin(&output) == STEP_EXECUTOR_STATUS_OK);
    assert(output.sequence == 1U && output.last_in_block == 0U);
    assert(step_executor_get_status() == STEP_EXECUTOR_STATUS_OK);
    assert(step_executor_pulse_complete(&output) == STEP_EXECUTOR_STATUS_OK);
    assert(output.last_in_block == 1U);
    assert(step_executor_pulse_complete(&output) == STEP_EXECUTOR_STATUS_COMPLETE);
    assert(step_executor_get_status() == STEP_EXECUTOR_STATUS_COMPLETE);

    step_executor_reset();
    first = make_event(2U, 0U);
    assert(execution_buffer_try_push(&first) == EXECUTION_BUFFER_STATUS_OK);
    assert(step_executor_begin(&output) == STEP_EXECUTOR_STATUS_OK);
    assert(step_executor_pulse_complete(&output) == STEP_EXECUTOR_STATUS_UNDERRUN);
    assert(step_executor_get_status() == STEP_EXECUTOR_STATUS_UNDERRUN);
}

static void test_step_executor_state_guards(void) {
    execution_buffer_clear();
    step_executor_reset();
    MotionExecutionEvent_t output = {0};

    assert(step_executor_get_status() == STEP_EXECUTOR_STATUS_NOT_RUNNING);
    assert(step_executor_begin(NULL) == STEP_EXECUTOR_STATUS_INVALID_ARGUMENT);
    assert(step_executor_pulse_complete(NULL) == STEP_EXECUTOR_STATUS_INVALID_ARGUMENT);
    assert(step_executor_pulse_complete(&output) == STEP_EXECUTOR_STATUS_NOT_RUNNING);

    assert(step_executor_begin(&output) == STEP_EXECUTOR_STATUS_UNDERRUN);

    step_executor_reset();
    MotionExecutionEvent_t event = make_event(3U, 1U);
    assert(execution_buffer_try_push(&event) == EXECUTION_BUFFER_STATUS_OK);
    assert(step_executor_begin(&output) == STEP_EXECUTOR_STATUS_OK);
    assert(step_executor_begin(&output) == STEP_EXECUTOR_STATUS_NOT_RUNNING);
    assert(step_executor_pulse_complete(&output) == STEP_EXECUTOR_STATUS_COMPLETE);
    assert(step_executor_pulse_complete(&output) == STEP_EXECUTOR_STATUS_COMPLETE);
}

static void test_step_executor_rejects_invalid_events(void) {
    MotionExecutionEvent_t invalid_events[5];
    for (uint32_t index = 0U; index < 5U; ++index) {
        invalid_events[index] = make_event(4U, 0U);
    }
    invalid_events[0].interval_ns = 0U;
    invalid_events[1].step_mask = 0U;
    invalid_events[2].last_in_block = 2U;
    invalid_events[3].step_mask |= 0x80U;
    invalid_events[4].positive_direction_mask |= 0x80U;

    for (uint32_t index = 0U; index < 5U; ++index) {
        execution_buffer_clear();
        step_executor_reset();
        MotionExecutionEvent_t output = {0};
        assert(execution_buffer_try_push(&invalid_events[index]) == EXECUTION_BUFFER_STATUS_OK);
        assert(step_executor_begin(&output) == STEP_EXECUTOR_STATUS_INVALID_ARGUMENT);
        assert(step_executor_get_status() == STEP_EXECUTOR_STATUS_NOT_RUNNING);
    }
}

int main(void) {
    test_generator_bresenham_and_directions();
    test_generator_validation_and_fractional_timing();
    test_generator_rejects_inconsistent_blocks();
    test_generator_next_guards_and_exact_interval();
    test_generator_detects_corrupted_state();
    test_execution_buffer_capacity_wrap_and_clear();
    test_step_executor_completion_and_underrun();
    test_step_executor_state_guards();
    test_step_executor_rejects_invalid_events();
    puts("motion execution unit tests passed");
    return 0;
}

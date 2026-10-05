#include "motion/plan_buffer.h"

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

static void expect_i64(int64_t actual, int64_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %lld, got %lld)\n", message, (long long)expected, (long long)actual);
    }
}

static MotionPlanBlock_t make_block(uint32_t sequence) {
    const int64_t signed_sequence = (int64_t)sequence;
    return (MotionPlanBlock_t){
        .sequence = sequence,
        .step_delta = {signed_sequence, -2 * signed_sequence, 3 * signed_sequence},
        .path_length_um = (uint64_t)sequence * 100U,
        .master_step_count = (uint64_t)sequence * 3U,
        .step_event_rate_numerator = (uint64_t)sequence * 1000U,
        .step_event_rate_denominator = sequence,
        .feedrate_um_per_s = sequence * 10U,
        .moving_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
        .positive_direction_mask = (uint8_t)(sequence & MOTION_PLANNER_ALL_AXES_MASK),
    };
}

static void expect_block_equal(const MotionPlanBlock_t *actual, uint32_t sequence) {
    const MotionPlanBlock_t expected = make_block(sequence);
    expect_u32(actual->sequence, expected.sequence, "buffer preserves plan sequence");
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        expect_i64(actual->step_delta[axis], expected.step_delta[axis], "buffer preserves signed step delta");
    }
    expect_u64(actual->path_length_um, expected.path_length_um, "buffer preserves path length");
    expect_u64(actual->master_step_count, expected.master_step_count, "buffer preserves master step count");
    expect_u64(
        actual->step_event_rate_numerator,
        expected.step_event_rate_numerator,
        "buffer preserves event rate numerator"
    );
    expect_u64(
        actual->step_event_rate_denominator,
        expected.step_event_rate_denominator,
        "buffer preserves event rate denominator"
    );
    expect_u32(actual->feedrate_um_per_s, expected.feedrate_um_per_s, "buffer preserves feedrate");
    expect_u32(actual->moving_axes_mask, expected.moving_axes_mask, "buffer preserves moving axes mask");
    expect_u32(actual->positive_direction_mask, expected.positive_direction_mask, "buffer preserves direction mask");
}

static void reset_buffer(void) {
    plan_buffer_test_reset();
}

static void test_initialization_and_preinit_results(void) {
    reset_buffer();
    const MotionPlanBlock_t block = make_block(1U);
    MotionPlanBlock_t output = {0};
    PlanBufferDiagnostics_t diagnostics = {0};

    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "push before init is rejected");
    expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "pop before init is rejected");
    expect_u32(plan_buffer_clear(), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "clear before init is rejected");
    plan_buffer_get_diagnostics(NULL);
    expect_u32(plan_buffer_get_diagnostics(&diagnostics), PLAN_BUFFER_STATUS_OK, "preinit diagnostics are available");
    expect_u32(diagnostics.initialized, 0U, "diagnostics report uninitialized buffer");
    expect_u32(diagnostics.capacity, PLAN_BUFFER_CAPACITY, "diagnostics report fixed capacity");
    expect_u32(diagnostics.queued, 0U, "uninitialized buffer reports no queued blocks");
    expect_u32(diagnostics.available, 0U, "uninitialized buffer reports no available slots");

    expect_u32(plan_buffer_init(), PLAN_BUFFER_STATUS_OK, "buffer initializes");
    expect_u32(plan_buffer_init(), PLAN_BUFFER_STATUS_ALREADY_INITIALIZED, "buffer cannot be initialized twice");
    expect_u32(plan_buffer_get_diagnostics(&diagnostics), PLAN_BUFFER_STATUS_OK, "initialized diagnostics are available");
    expect_u32(diagnostics.initialized, 1U, "diagnostics report initialized buffer");
    expect_u32(diagnostics.available, PLAN_BUFFER_CAPACITY, "initialized buffer reports all slots available");
    expect_u32(diagnostics.queued, 0U, "new buffer starts empty");
}

static void test_fifo_full_wraparound_and_copy(void) {
    reset_buffer();
    expect_u32(plan_buffer_init(), PLAN_BUFFER_STATUS_OK, "buffer initializes for FIFO test");

    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "buffer accepts every block while space remains");
        if (sequence == 1U) {
            block = make_block(999U);
        }
    }
    MotionPlanBlock_t overflow = make_block(PLAN_BUFFER_CAPACITY + 1U);
    expect_u32(plan_buffer_try_push(&overflow), PLAN_BUFFER_STATUS_FULL, "full buffer rejects without waiting");

    MotionPlanBlock_t output = {0};
    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY / 2U; ++sequence) {
        expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_OK, "buffer returns head while populated");
        expect_block_equal(&output, sequence);
    }
    for (uint32_t sequence = PLAN_BUFFER_CAPACITY + 1U; sequence <= PLAN_BUFFER_CAPACITY + PLAN_BUFFER_CAPACITY / 2U; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "buffer reuses wrapped slots");
    }

    PlanBufferDiagnostics_t diagnostics = {0};
    expect_u32(plan_buffer_get_diagnostics(&diagnostics), PLAN_BUFFER_STATUS_OK, "full-buffer diagnostics are available");
    expect_u32(diagnostics.queued, PLAN_BUFFER_CAPACITY, "buffer reports full occupancy after wraparound");
    expect_u32(diagnostics.available, 0U, "full buffer reports no available slots");
    expect_u32(diagnostics.max_queued, PLAN_BUFFER_CAPACITY, "buffer records maximum occupancy");
    expect_u32(diagnostics.enqueued_total, PLAN_BUFFER_CAPACITY + PLAN_BUFFER_CAPACITY / 2U, "buffer counts successful pushes");
    expect_u32(diagnostics.dequeued_total, PLAN_BUFFER_CAPACITY / 2U, "buffer counts successful pops");
    expect_u32(diagnostics.full_rejections, 1U, "buffer counts full rejections");

    for (uint32_t sequence = PLAN_BUFFER_CAPACITY / 2U + 1U;
         sequence <= PLAN_BUFFER_CAPACITY + PLAN_BUFFER_CAPACITY / 2U;
         ++sequence) {
        expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_OK, "wrapped FIFO retains all remaining blocks");
        expect_block_equal(&output, sequence);
    }
    expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_EMPTY, "empty buffer reports empty result");
    expect_u32(plan_buffer_get_diagnostics(&diagnostics), PLAN_BUFFER_STATUS_OK, "drained diagnostics are available");
    expect_u32(diagnostics.queued, 0U, "drained buffer reports zero occupancy");
    expect_u32(diagnostics.available, PLAN_BUFFER_CAPACITY, "drained buffer reports full availability");
    expect_u32(diagnostics.empty_reads, 1U, "buffer counts empty reads");
}

static void test_clear_cancels_only_pending_blocks(void) {
    reset_buffer();
    (void)plan_buffer_init();
    MotionPlanBlock_t first = make_block(10U);
    MotionPlanBlock_t second = make_block(11U);
    expect_u32(plan_buffer_try_push(&first), PLAN_BUFFER_STATUS_OK, "first block queues before clear");
    expect_u32(plan_buffer_try_push(&second), PLAN_BUFFER_STATUS_OK, "second block queues before clear");
    expect_u32(plan_buffer_clear(), PLAN_BUFFER_STATUS_OK, "clear cancels queued blocks");

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.queued, 0U, "clear empties the buffer");
    expect_u32(diagnostics.clear_calls, 1U, "clear request is counted");
    expect_u32(diagnostics.cancelled_blocks, 2U, "clear counts cancelled blocks");

    const MotionPlanBlock_t after_clear = make_block(12U);
    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_push(&after_clear), PLAN_BUFFER_STATUS_OK, "buffer accepts plans after clear");
    expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_OK, "new plan can be read after clear");
    expect_block_equal(&output, 12U);
    expect_u32(plan_buffer_clear(), PLAN_BUFFER_STATUS_OK, "clearing an empty buffer succeeds");
    (void)plan_buffer_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.clear_calls, 2U, "empty clear is counted");
    expect_u32(diagnostics.cancelled_blocks, 2U, "empty clear cancels no additional blocks");
}

static void test_invalid_arguments_are_reported(void) {
    reset_buffer();
    (void)plan_buffer_init();
    MotionPlanBlock_t block = make_block(1U);
    expect_u32(plan_buffer_try_push(NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null block is rejected");
    expect_u32(plan_buffer_try_pop(NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null output is rejected");
    expect_u32(plan_buffer_get_diagnostics(NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null diagnostics output is rejected");
    plan_buffer_test_seed_counters(NULL);

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.rejected_operations, 2U, "invalid mutating operations are counted");
    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "valid operation succeeds after invalid arguments");
}

static void test_diagnostic_counters_saturate(void) {
    reset_buffer();
    (void)plan_buffer_init();
    MotionPlanBlock_t block = make_block(1U);
    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "buffer accepts first block before saturation test");
    block = make_block(2U);
    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_OK, "buffer accepts second block before saturation test");

    const PlanBufferDiagnostics_t saturated = {
        .max_queued = 2U,
        .enqueued_total = UINT32_MAX - 1U,
        .dequeued_total = UINT32_MAX - 1U,
        .full_rejections = UINT32_MAX,
        .empty_reads = UINT32_MAX,
        .rejected_operations = UINT32_MAX,
        .clear_calls = UINT32_MAX,
        .cancelled_blocks = UINT32_MAX - 1U,
    };
    plan_buffer_test_seed_counters(&saturated);

    for (uint32_t sequence = 3U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        block = make_block(sequence);
        (void)plan_buffer_try_push(&block);
    }
    block = make_block(PLAN_BUFFER_CAPACITY + 1U);
    expect_u32(plan_buffer_try_push(&block), PLAN_BUFFER_STATUS_FULL, "saturated full counter does not change result");

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_OK, "saturated dequeue counter does not change result");
    expect_u32(plan_buffer_clear(), PLAN_BUFFER_STATUS_OK, "saturated cancellation counters do not change clear result");
    expect_u32(plan_buffer_try_pop(&output), PLAN_BUFFER_STATUS_EMPTY, "saturated empty counter does not change result");
    expect_u32(plan_buffer_try_push(NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "saturated error counter does not change result");
    expect_u32(plan_buffer_clear(), PLAN_BUFFER_STATUS_OK, "clear succeeds when no blocks remain");

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.enqueued_total, UINT32_MAX, "enqueued counter saturates");
    expect_u32(diagnostics.dequeued_total, UINT32_MAX, "dequeued counter saturates");
    expect_u32(diagnostics.full_rejections, UINT32_MAX, "full counter saturates");
    expect_u32(diagnostics.empty_reads, UINT32_MAX, "empty counter saturates");
    expect_u32(diagnostics.rejected_operations, UINT32_MAX, "rejected operation counter saturates");
    expect_u32(diagnostics.clear_calls, UINT32_MAX, "clear counter saturates");
    expect_u32(diagnostics.cancelled_blocks, UINT32_MAX, "cancelled block counter saturates");
}

int main(void) {
    test_initialization_and_preinit_results();
    test_fifo_full_wraparound_and_copy();
    test_clear_cancels_only_pending_blocks();
    test_invalid_arguments_are_reported();
    test_diagnostic_counters_saturate();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Plan buffer unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Plan buffer unit tests passed.\n");
    return 0;
}

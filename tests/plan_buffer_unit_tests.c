#include "motion/plan_buffer.h"

#include "cmsis_os.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static PlanBuffer_t test_buffer;
static MotionPlanBlock_t test_buffer_entries[PLAN_BUFFER_CAPACITY];

static void reset_test_buffer(void) {
    (void)memset(&test_buffer, 0, sizeof(test_buffer));
    (void)memset(test_buffer_entries, 0, sizeof(test_buffer_entries));
}

static uint32_t test_failures;
static uint8_t test_mutex_object;
static uint8_t test_mutex_new_fails;
static uint32_t test_mutex_new_calls;
static uint32_t test_mutex_attr_bits;
static uint32_t test_mutex_acquire_calls;
static uint32_t test_mutex_release_calls;
static uint32_t test_mutex_depth;
static uint32_t test_mutex_max_depth;
static uint32_t test_mutex_last_timeout;
static uint8_t test_mutex_wrong_handle;
static osStatus_t test_mutex_acquire_result;
static osStatus_t test_mutex_release_result;
static uint32_t test_error_handler_calls;
/* What the space listeners saw: how often they were called, how many locks were held at that moment, and how many
 * blocks the buffer held (read from the object, which the listener sees as private state of the reader's buffer). */
static uint32_t test_listener_calls;
static uint32_t test_listener_mutex_depth;
static uint32_t test_listener_queued;
static uint32_t test_second_listener_calls;
static uint32_t test_listener_refills;
static uint32_t test_listener_refill_status;

osMutexId_t osMutexNew(const osMutexAttr_t *attr) {
    ++test_mutex_new_calls;
    test_mutex_attr_bits = attr != NULL ? attr->attr_bits : 0U;
    return test_mutex_new_fails != 0U ? NULL : &test_mutex_object;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout) {
    ++test_mutex_acquire_calls;
    test_mutex_last_timeout = timeout;
    if (mutex_id != &test_mutex_object) {
        test_mutex_wrong_handle = 1U;
    }
    ++test_mutex_depth;
    if (test_mutex_depth > test_mutex_max_depth) {
        test_mutex_max_depth = test_mutex_depth;
    }
    return test_mutex_acquire_result;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id) {
    ++test_mutex_release_calls;
    if (mutex_id != &test_mutex_object) {
        test_mutex_wrong_handle = 1U;
    }
    if (test_mutex_depth != 0U) {
        --test_mutex_depth;
    }
    return test_mutex_release_result;
}

void Error_Handler(void) { ++test_error_handler_calls; }

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
    reset_test_buffer();
    test_mutex_new_fails = 0U;
    test_mutex_new_calls = 0U;
    test_mutex_attr_bits = 0U;
    test_mutex_acquire_calls = 0U;
    test_mutex_release_calls = 0U;
    test_mutex_depth = 0U;
    test_mutex_max_depth = 0U;
    test_mutex_last_timeout = 0U;
    test_mutex_wrong_handle = 0U;
    test_mutex_acquire_result = osOK;
    test_mutex_release_result = osOK;
    test_error_handler_calls = 0U;
    test_listener_calls = 0U;
    test_listener_mutex_depth = UINT32_MAX;
    test_listener_queued = UINT32_MAX;
    test_second_listener_calls = 0U;
    test_listener_refills = 0U;
    test_listener_refill_status = UINT32_MAX;
}

static void test_mutex_guards_every_operation(void) {
    reset_buffer();
    test_mutex_new_fails = 1U;
    expect_u32(plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY), PLAN_BUFFER_STATUS_NO_MEMORY, "missing mutex fails initialization");
    PlanBufferDiagnostics_t diagnostics = {0};
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "buffer stays uninitialized without a mutex");
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(test_mutex_acquire_calls, 0U, "uninitialized buffer does not lock");

    test_mutex_new_fails = 0U;
    expect_u32(plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY), PLAN_BUFFER_STATUS_OK, "buffer initializes once the mutex is created");
    expect_u32(test_mutex_new_calls, 2U, "mutex creation is retried on the next init");
    expect_u32(test_mutex_attr_bits, osMutexPrioInherit, "mutex uses priority inheritance");

    MotionPlanBlock_t block = make_block(1U);
    MotionPlanBlock_t output = {0};
    (void)plan_buffer_try_push(&test_buffer, &block);
    (void)plan_buffer_try_push(&test_buffer, NULL);
    (void)plan_buffer_try_pop(&test_buffer, &output);
    (void)plan_buffer_try_pop(&test_buffer, &output);
    (void)plan_buffer_try_pop(&test_buffer, NULL);
    (void)plan_buffer_try_peek(&test_buffer, plan_buffer_get_epoch(&test_buffer), 0U, &output);
    (void)plan_buffer_discard(&test_buffer, plan_buffer_get_epoch(&test_buffer), 0U);
    (void)plan_buffer_clear(&test_buffer);
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(test_mutex_acquire_calls, 9U, "every operation on an initialized buffer locks");
    expect_u32(test_mutex_release_calls, 9U, "every lock is released");
    expect_u32(test_mutex_max_depth, 1U, "operations never nest locks");
    expect_u32(test_mutex_last_timeout, osWaitForever, "operations wait for the mutex holder");
    expect_u32(test_mutex_wrong_handle, 0U, "operations use the buffer mutex");
    expect_u32(test_error_handler_calls, 0U, "successful locking raises no fault");

    test_mutex_acquire_result = osErrorParameter;
    (void)plan_buffer_clear(&test_buffer);
    expect_u32(test_error_handler_calls, 1U, "failed lock is a fatal error");
    test_mutex_acquire_result = osOK;
    test_mutex_release_result = osErrorResource;
    (void)plan_buffer_clear(&test_buffer);
    expect_u32(test_error_handler_calls, 2U, "failed unlock is a fatal error");
}

static void test_initialization_and_preinit_results(void) {
    reset_buffer();
    const MotionPlanBlock_t block = make_block(1U);
    MotionPlanBlock_t output = {0};
    PlanBufferDiagnostics_t diagnostics = {0};

    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "push before init is rejected");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "pop before init is rejected");
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "clear before init is rejected");
    expect_u32(plan_buffer_try_push(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null push before init is rejected");
    expect_u32(plan_buffer_try_pop(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null pop before init is rejected");
    plan_buffer_get_diagnostics(&test_buffer, NULL);
    expect_u32(plan_buffer_get_diagnostics(&test_buffer, &diagnostics), PLAN_BUFFER_STATUS_OK, "preinit diagnostics are available");
    expect_u32(diagnostics.queued, 0U, "uninitialized buffer reports no queued blocks");

    expect_u32(plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY), PLAN_BUFFER_STATUS_OK, "buffer initializes");
    expect_u32(plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY), PLAN_BUFFER_STATUS_ALREADY_INITIALIZED, "buffer cannot be initialized twice");
    expect_u32(plan_buffer_get_diagnostics(&test_buffer, &diagnostics), PLAN_BUFFER_STATUS_OK, "initialized diagnostics are available");
    expect_u32(diagnostics.queued, 0U, "new buffer starts empty");
}

static void test_fifo_full_wraparound_and_copy(void) {
    reset_buffer();
    expect_u32(plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY), PLAN_BUFFER_STATUS_OK, "buffer initializes for FIFO test");

    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "buffer accepts every block while space remains");
        if (sequence == 1U) {
            block = make_block(999U);
        }
    }
    MotionPlanBlock_t overflow = make_block(PLAN_BUFFER_CAPACITY + 1U);
    expect_u32(plan_buffer_try_push(&test_buffer, &overflow), PLAN_BUFFER_STATUS_FULL, "full buffer rejects without waiting");

    MotionPlanBlock_t output = {0};
    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY / 2U; ++sequence) {
        expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "buffer returns head while populated");
        expect_block_equal(&output, sequence);
    }
    for (uint32_t sequence = PLAN_BUFFER_CAPACITY + 1U; sequence <= PLAN_BUFFER_CAPACITY + PLAN_BUFFER_CAPACITY / 2U; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "buffer reuses wrapped slots");
    }

    PlanBufferDiagnostics_t diagnostics = {0};
    expect_u32(plan_buffer_get_diagnostics(&test_buffer, &diagnostics), PLAN_BUFFER_STATUS_OK, "full-buffer diagnostics are available");
    expect_u32(diagnostics.queued, PLAN_BUFFER_CAPACITY, "buffer reports full occupancy after wraparound");
    expect_u32(diagnostics.max_queued, PLAN_BUFFER_CAPACITY, "buffer records maximum occupancy");
    expect_u32(diagnostics.enqueued_total, PLAN_BUFFER_CAPACITY + PLAN_BUFFER_CAPACITY / 2U, "buffer counts successful pushes");
    expect_u32(diagnostics.dequeued_total, PLAN_BUFFER_CAPACITY / 2U, "buffer counts successful pops");
    expect_u32(diagnostics.full_rejections, 1U, "buffer counts full rejections");

    for (uint32_t sequence = PLAN_BUFFER_CAPACITY / 2U + 1U;
         sequence <= PLAN_BUFFER_CAPACITY + PLAN_BUFFER_CAPACITY / 2U;
         ++sequence) {
        expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "wrapped FIFO retains all remaining blocks");
        expect_block_equal(&output, sequence);
    }
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_EMPTY, "empty buffer reports empty result");
    expect_u32(plan_buffer_get_diagnostics(&test_buffer, &diagnostics), PLAN_BUFFER_STATUS_OK, "drained diagnostics are available");
    expect_u32(diagnostics.queued, 0U, "drained buffer reports zero occupancy");
}

static void test_clear_cancels_only_pending_blocks(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    MotionPlanBlock_t first = make_block(10U);
    MotionPlanBlock_t second = make_block(11U);
    expect_u32(plan_buffer_try_push(&test_buffer, &first), PLAN_BUFFER_STATUS_OK, "first block queues before clear");
    expect_u32(plan_buffer_try_push(&test_buffer, &second), PLAN_BUFFER_STATUS_OK, "second block queues before clear");
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "clear cancels queued blocks");

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, 0U, "clear empties the buffer");
    expect_u32(diagnostics.cancelled_blocks, 2U, "clear counts cancelled blocks");

    const MotionPlanBlock_t after_clear = make_block(12U);
    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_push(&test_buffer, &after_clear), PLAN_BUFFER_STATUS_OK, "buffer accepts plans after clear");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "new plan can be read after clear");
    expect_block_equal(&output, 12U);
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "clearing an empty buffer succeeds");
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.cancelled_blocks, 2U, "empty clear cancels no additional blocks");
}

static void test_peek_reads_ahead_without_taking(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    const uint32_t epoch = plan_buffer_get_epoch(&test_buffer);
    MotionPlanBlock_t output = {0};
    expect_u32(
        plan_buffer_try_peek(&test_buffer, epoch, 0U, &output),
        PLAN_BUFFER_STATUS_EMPTY,
        "an empty buffer has nothing to peek"
    );

    for (uint32_t sequence = 1U; sequence <= 3U; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "buffer accepts blocks for the peek test");
    }
    for (uint32_t offset = 0U; offset < 3U; ++offset) {
        expect_u32(plan_buffer_try_peek(&test_buffer, epoch, offset, &output), PLAN_BUFFER_STATUS_OK, "peek reads a queued block");
        expect_block_equal(&output, offset + 1U);
    }
    expect_u32(
        plan_buffer_try_peek(&test_buffer, epoch, 3U, &output),
        PLAN_BUFFER_STATUS_EMPTY,
        "peek behind the last block finds none"
    );
    expect_u32(plan_buffer_try_peek(&test_buffer, epoch, UINT32_MAX, &output), PLAN_BUFFER_STATUS_EMPTY, "a huge offset finds none");

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, 3U, "peek takes nothing");
    expect_u32(diagnostics.dequeued_total, 0U, "peek is not a dequeue");

    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the front is still there after the peeks");
    expect_block_equal(&output, 1U);
}

static void test_discard_drops_the_front(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    const uint32_t epoch = plan_buffer_get_epoch(&test_buffer);
    expect_u32(plan_buffer_discard(&test_buffer, epoch, 0U), PLAN_BUFFER_STATUS_OK, "discarding nothing is fine");
    for (uint32_t sequence = 1U; sequence <= 5U; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        (void)plan_buffer_try_push(&test_buffer, &block);
    }
    expect_u32(plan_buffer_discard(&test_buffer, epoch, 2U), PLAN_BUFFER_STATUS_OK, "discard drops the first blocks");

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_peek(&test_buffer, epoch, 0U, &output), PLAN_BUFFER_STATUS_OK, "the front moved");
    expect_block_equal(&output, 3U);
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, 3U, "discard shrinks the queue");
    expect_u32(diagnostics.dequeued_total, 2U, "discarded blocks count as dequeued");

    expect_u32(plan_buffer_discard(&test_buffer, epoch, 4U), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "more than queued is refused");
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, 3U, "a refused discard changes nothing");
    expect_u32(diagnostics.rejected_operations, 1U, "a refused discard is counted");
    expect_u32(plan_buffer_discard(&test_buffer, epoch, 3U), PLAN_BUFFER_STATUS_OK, "everything queued can be discarded");
    expect_u32(plan_buffer_try_peek(&test_buffer, epoch, 0U, &output), PLAN_BUFFER_STATUS_EMPTY, "nothing is left after it");
}

static void test_peek_and_discard_wrap_around(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    const uint32_t epoch = plan_buffer_get_epoch(&test_buffer);
    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        (void)plan_buffer_try_push(&test_buffer, &block);
    }
    expect_u32(
        plan_buffer_discard(&test_buffer, epoch, PLAN_BUFFER_CAPACITY - 2U),
        PLAN_BUFFER_STATUS_OK,
        "discard nearly everything"
    );
    for (uint32_t sequence = PLAN_BUFFER_CAPACITY + 1U; sequence <= PLAN_BUFFER_CAPACITY + 3U; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "the queue wraps");
    }
    MotionPlanBlock_t output = {0};
    for (uint32_t offset = 0U; offset < 5U; ++offset) {
        expect_u32(plan_buffer_try_peek(&test_buffer, epoch, offset, &output), PLAN_BUFFER_STATUS_OK, "peek follows the wrap");
        expect_block_equal(&output, PLAN_BUFFER_CAPACITY - 1U + offset);
    }
    expect_u32(
        plan_buffer_try_peek(&test_buffer, epoch, 5U, &output),
        PLAN_BUFFER_STATUS_EMPTY,
        "the queue ends where it was pushed"
    );
}

static void test_clear_makes_old_positions_stale(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    const uint32_t old_epoch = plan_buffer_get_epoch(&test_buffer);
    const MotionPlanBlock_t first = make_block(1U);
    (void)plan_buffer_try_push(&test_buffer, &first);
    (void)plan_buffer_clear(&test_buffer);
    const uint32_t new_epoch = plan_buffer_get_epoch(&test_buffer);
    expect_u32(new_epoch != old_epoch, 1U, "a clear changes the epoch");
    (void)plan_buffer_clear(&test_buffer);
    expect_u32(plan_buffer_get_epoch(&test_buffer) != new_epoch, 1U, "even a clear of an empty buffer changes it");

    const MotionPlanBlock_t second = make_block(2U);
    (void)plan_buffer_try_push(&test_buffer, &second);
    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_peek(&test_buffer, old_epoch, 0U, &output), PLAN_BUFFER_STATUS_STALE, "an old epoch cannot peek");
    expect_u32(plan_buffer_discard(&test_buffer, old_epoch, 1U), PLAN_BUFFER_STATUS_STALE, "an old epoch cannot discard");
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, 1U, "a stale discard changes nothing");
    expect_u32(diagnostics.rejected_operations, 2U, "stale calls are counted");
    expect_u32(
        plan_buffer_try_peek(&test_buffer, plan_buffer_get_epoch(&test_buffer), 0U, &output),
        PLAN_BUFFER_STATUS_OK,
        "the new epoch works"
    );
    expect_block_equal(&output, 2U);
}

static void test_peek_and_discard_before_init_and_with_bad_arguments(void) {
    reset_buffer();
    MotionPlanBlock_t output = {0};
    expect_u32(
        plan_buffer_try_peek(&test_buffer, 0U, 0U, &output),
        PLAN_BUFFER_STATUS_NOT_INITIALIZED,
        "peek before init is rejected"
    );
    expect_u32(
        plan_buffer_try_peek(&test_buffer, 0U, 0U, NULL),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "null peek before init is rejected"
    );
    expect_u32(plan_buffer_discard(&test_buffer, 0U, 0U), PLAN_BUFFER_STATUS_NOT_INITIALIZED, "discard before init is rejected");
    expect_u32(plan_buffer_get_epoch(&test_buffer), 0U, "the epoch starts at zero");

    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    expect_u32(plan_buffer_try_peek(&test_buffer, 0U, 0U, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null peek output is rejected");
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.rejected_operations, 1U, "the null peek is counted");
}

static void test_invalid_arguments_are_reported(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    MotionPlanBlock_t block = make_block(1U);
    expect_u32(plan_buffer_try_push(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null block is rejected");
    expect_u32(plan_buffer_try_pop(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null output is rejected");
    expect_u32(plan_buffer_get_diagnostics(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "null diagnostics output is rejected");
    plan_buffer_test_seed_counters(&test_buffer, NULL);

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.rejected_operations, 2U, "invalid mutating operations are counted");
    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "valid operation succeeds after invalid arguments");
}

static void test_diagnostic_counters_saturate(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    MotionPlanBlock_t block = make_block(1U);
    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "buffer accepts first block before saturation test");
    block = make_block(2U);
    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "buffer accepts second block before saturation test");

    const PlanBufferDiagnostics_t saturated = {
        .max_queued = 2U,
        .enqueued_total = UINT32_MAX - 1U,
        .dequeued_total = UINT32_MAX - 1U,
        .full_rejections = UINT32_MAX,
        .rejected_operations = UINT32_MAX,
        .cancelled_blocks = UINT32_MAX - 1U,
    };
    plan_buffer_test_seed_counters(&test_buffer, &saturated);

    for (uint32_t sequence = 3U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        block = make_block(sequence);
        (void)plan_buffer_try_push(&test_buffer, &block);
    }
    block = make_block(PLAN_BUFFER_CAPACITY + 1U);
    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_FULL, "saturated full counter does not change result");

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "saturated dequeue counter does not change result");
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "saturated cancellation counters do not change clear result");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_EMPTY, "saturated empty counter does not change result");
    expect_u32(plan_buffer_try_push(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "saturated error counter does not change result");
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "clear succeeds when no blocks remain");

    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.enqueued_total, UINT32_MAX, "enqueued counter saturates");
    expect_u32(diagnostics.dequeued_total, UINT32_MAX, "dequeued counter saturates");
    expect_u32(diagnostics.full_rejections, UINT32_MAX, "full counter saturates");
    expect_u32(diagnostics.rejected_operations, UINT32_MAX, "rejected operation counter saturates");
    expect_u32(diagnostics.cancelled_blocks, UINT32_MAX, "cancelled block counter saturates");
}

#define SECOND_BUFFER_CAPACITY 3U

static PlanBuffer_t second_buffer;
static MotionPlanBlock_t second_buffer_entries[SECOND_BUFFER_CAPACITY];

static void reset_second_buffer(void) {
    (void)memset(&second_buffer, 0, sizeof(second_buffer));
    (void)memset(second_buffer_entries, 0, sizeof(second_buffer_entries));
}

static void test_init_rejects_bad_arguments(void) {
    reset_buffer();
    const MotionPlanBlock_t block = make_block(1U);
    test_buffer_entries[0].sequence = 0xA5A5A5A5U;

    expect_u32(
        plan_buffer_init(NULL, test_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "init without an object is rejected"
    );
    expect_u32(
        plan_buffer_init(&test_buffer, NULL, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "init without storage is rejected"
    );
    expect_u32(
        plan_buffer_init(&test_buffer, test_buffer_entries, 0U),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "init with zero capacity is rejected"
    );
    expect_u32(test_mutex_new_calls, 0U, "a rejected init creates no mutex");
    expect_u32(test_buffer_entries[0].sequence, 0xA5A5A5A5U, "a rejected init leaves the storage alone");
    expect_u32(
        plan_buffer_try_push(&test_buffer, &block),
        PLAN_BUFFER_STATUS_NOT_INITIALIZED,
        "an object whose init was rejected stays uninitialized"
    );
}

static void test_null_object_is_rejected_by_every_call(void) {
    reset_buffer();
    const MotionPlanBlock_t block = make_block(1U);
    MotionPlanBlock_t output = {0};
    PlanBufferDiagnostics_t diagnostics = {.queued = 77U};

    expect_u32(plan_buffer_try_push(NULL, &block), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "push to no object is rejected");
    expect_u32(plan_buffer_try_pop(NULL, &output), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "pop from no object is rejected");
    expect_u32(
        plan_buffer_try_peek(NULL, 0U, 0U, &output),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "peek into no object is rejected"
    );
    expect_u32(plan_buffer_discard(NULL, 0U, 0U), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "discard from no object is rejected");
    expect_u32(plan_buffer_clear(NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "clear of no object is rejected");
    expect_u32(
        plan_buffer_get_diagnostics(NULL, &diagnostics),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "diagnostics of no object are rejected"
    );
    expect_u32(diagnostics.queued, 77U, "diagnostics of no object write nothing");
    expect_u32(plan_buffer_get_epoch(NULL), 0U, "no object has epoch zero");
    expect_u32(test_mutex_acquire_calls, 0U, "calls on no object take no lock");
}

static void test_objects_are_independent(void) {
    reset_buffer();
    reset_second_buffer();
    expect_u32(
        plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "first object initializes"
    );
    expect_u32(
        plan_buffer_init(&second_buffer, second_buffer_entries, SECOND_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "second object initializes with its own storage"
    );

    MotionPlanBlock_t output = {0};
    PlanBufferDiagnostics_t first = {0};
    PlanBufferDiagnostics_t second = {0};
    const MotionPlanBlock_t block = make_block(1U);
    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "a block queues in the first object");
    expect_u32(
        plan_buffer_try_pop(&second_buffer, &output),
        PLAN_BUFFER_STATUS_EMPTY,
        "the block of the first object is not in the second"
    );
    (void)plan_buffer_get_diagnostics(&second_buffer, &second);
    expect_u32(second.queued, 0U, "the second object queued nothing");
    expect_u32(second.enqueued_total, 0U, "the second object counted no push");
    expect_u32(second.max_queued, 0U, "the second object recorded no occupancy");

    const MotionPlanBlock_t other = make_block(2U);
    expect_u32(plan_buffer_try_push(&second_buffer, &other), PLAN_BUFFER_STATUS_OK, "a block queues in the second object");
    (void)plan_buffer_get_diagnostics(&test_buffer, &first);
    expect_u32(first.queued, 1U, "the push to the second object does not reach the first");
    expect_u32(first.enqueued_total, 1U, "the first object counts only its own push");

    /* A clear of the first object leaves the second one's blocks, epoch and counters alone. */
    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "the first object clears");
    expect_u32(plan_buffer_get_epoch(&test_buffer), 1U, "the clear moves the epoch of the first object");
    expect_u32(plan_buffer_get_epoch(&second_buffer), 0U, "the epoch of the second object does not move");
    (void)plan_buffer_get_diagnostics(&test_buffer, &first);
    (void)plan_buffer_get_diagnostics(&second_buffer, &second);
    expect_u32(first.cancelled_blocks, 1U, "the first object cancelled its block");
    expect_u32(second.cancelled_blocks, 0U, "the second object cancelled nothing");
    expect_u32(second.queued, 1U, "the block of the second object survives the clear of the first");
    expect_u32(
        plan_buffer_try_pop(&second_buffer, &output),
        PLAN_BUFFER_STATUS_OK,
        "the second object still gives its block"
    );
    expect_block_equal(&output, 2U);

    /* A position of the second object's epoch is not valid for the first one after its clear. */
    expect_u32(
        plan_buffer_try_peek(&test_buffer, plan_buffer_get_epoch(&second_buffer), 0U, &output),
        PLAN_BUFFER_STATUS_STALE,
        "the epoch of one object does not fit the other"
    );
}

static void test_object_with_own_capacity_fills_and_wraps(void) {
    reset_buffer();
    reset_second_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    expect_u32(
        plan_buffer_init(&second_buffer, second_buffer_entries, SECOND_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "the object with a small capacity initializes"
    );

    for (uint32_t sequence = 1U; sequence <= SECOND_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&second_buffer, &block), PLAN_BUFFER_STATUS_OK, "the small buffer takes blocks up to its capacity");
    }
    const MotionPlanBlock_t overflow = make_block(4U);
    expect_u32(plan_buffer_try_push(&second_buffer, &overflow), PLAN_BUFFER_STATUS_FULL, "the fourth block does not fit three slots");

    MotionPlanBlock_t output = {0};
    for (uint32_t sequence = 1U; sequence <= 2U; ++sequence) {
        expect_u32(plan_buffer_try_pop(&second_buffer, &output), PLAN_BUFFER_STATUS_OK, "the small buffer gives the front");
        expect_block_equal(&output, sequence);
    }
    for (uint32_t sequence = 4U; sequence <= 5U; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(&second_buffer, &block), PLAN_BUFFER_STATUS_OK, "the small buffer reuses slots over its ring end");
    }
    for (uint32_t sequence = 3U; sequence <= 5U; ++sequence) {
        expect_u32(plan_buffer_try_pop(&second_buffer, &output), PLAN_BUFFER_STATUS_OK, "the wrapped small buffer keeps the order");
        expect_block_equal(&output, sequence);
    }
    expect_u32(plan_buffer_try_pop(&second_buffer, &output), PLAN_BUFFER_STATUS_EMPTY, "the small buffer drains");

    PlanBufferDiagnostics_t second = {0};
    PlanBufferDiagnostics_t first = {0};
    (void)plan_buffer_get_diagnostics(&second_buffer, &second);
    (void)plan_buffer_get_diagnostics(&test_buffer, &first);
    expect_u32(second.max_queued, SECOND_BUFFER_CAPACITY, "the small buffer peaks at its own capacity");
    expect_u32(second.full_rejections, 1U, "the small buffer counts its full rejection");
    expect_u32(second.enqueued_total, 5U, "the small buffer counts five accepted blocks");
    expect_u32(second.dequeued_total, 5U, "the small buffer counts five given blocks");
    expect_u32(first.enqueued_total, 0U, "the large buffer saw none of it");
    expect_u32(first.full_rejections, 0U, "the large buffer counts no rejection of the small one");
}

static void test_init_clears_storage_and_counters(void) {
    reset_buffer();
    for (uint32_t index = 0U; index < PLAN_BUFFER_CAPACITY; ++index) {
        (void)memset(&test_buffer_entries[index], 0xA5, sizeof(test_buffer_entries[index]));
    }
    const PlanBufferDiagnostics_t stale = {
        .queued = 0U,
        .max_queued = 9U,
        .enqueued_total = 8U,
        .dequeued_total = 7U,
        .full_rejections = 6U,
        .rejected_operations = 5U,
        .cancelled_blocks = 4U,
    };
    plan_buffer_test_seed_counters(&test_buffer, &stale);

    expect_u32(
        plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "init succeeds over dirty storage"
    );
    for (uint32_t index = 0U; index < PLAN_BUFFER_CAPACITY; ++index) {
        expect_u32(test_buffer_entries[index].sequence, 0U, "init clears the sequence of every slot");
        expect_u64(test_buffer_entries[index].path_length_um, 0U, "init clears the path of every slot");
    }

    PlanBufferDiagnostics_t diagnostics = {.queued = 1U, .max_queued = 1U, .enqueued_total = 1U};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, 0U, "init starts empty");
    expect_u32(diagnostics.max_queued, 0U, "init clears the peak");
    expect_u32(diagnostics.enqueued_total, 0U, "init clears the push counter");
    expect_u32(diagnostics.dequeued_total, 0U, "init clears the pop counter");
    expect_u32(diagnostics.full_rejections, 0U, "init clears the full counter");
    expect_u32(diagnostics.rejected_operations, 0U, "init clears the rejection counter");
    expect_u32(diagnostics.cancelled_blocks, 0U, "init clears the cancel counter");

    MotionPlanBlock_t output = {.sequence = 123U};
    expect_u32(
        plan_buffer_try_peek(&test_buffer, plan_buffer_get_epoch(&test_buffer), 0U, &output),
        PLAN_BUFFER_STATUS_EMPTY,
        "no old data is visible after init"
    );
    expect_u32(output.sequence, 123U, "an empty peek leaves the output alone");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_EMPTY, "no old data is popped after init");

    const MotionPlanBlock_t block = make_block(7U);
    expect_u32(plan_buffer_try_push(&test_buffer, &block), PLAN_BUFFER_STATUS_OK, "a block queues after init");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the block is popped");
    expect_block_equal(&output, 7U);
}

static void test_is_full_follows_the_occupancy(void) {
    reset_buffer();
    expect_u32(
        plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "buffer initializes for the fullness test"
    );
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "an empty buffer is not full");

    for (uint32_t sequence = 1U; sequence < PLAN_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        (void)plan_buffer_try_push(&test_buffer, &block);
    }
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "a buffer with one place left is not full");

    const MotionPlanBlock_t last = make_block(PLAN_BUFFER_CAPACITY);
    expect_u32(plan_buffer_try_push(&test_buffer, &last), PLAN_BUFFER_STATUS_OK, "the last place is taken");
    expect_u32(plan_buffer_is_full(&test_buffer), 1U, "a buffer with every place taken is full");
    expect_u32(
        plan_buffer_try_push(&test_buffer, &last),
        PLAN_BUFFER_STATUS_FULL,
        "a push into the buffer that reports full is refused as FULL"
    );
    expect_u32(plan_buffer_is_full(&test_buffer), 1U, "a refused push does not change the fullness");

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the front is popped");
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "a pop makes room");
    expect_u32(plan_buffer_try_push(&test_buffer, &last), PLAN_BUFFER_STATUS_OK, "the freed place is taken again");
    expect_u32(plan_buffer_is_full(&test_buffer), 1U, "the buffer is full again after the wrap around the ring end");

    expect_u32(
        plan_buffer_discard(&test_buffer, plan_buffer_get_epoch(&test_buffer), 1U),
        PLAN_BUFFER_STATUS_OK,
        "the front is discarded"
    );
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "a discard of one block makes room");
    expect_u32(plan_buffer_try_push(&test_buffer, &last), PLAN_BUFFER_STATUS_OK, "the place after the discard is taken");
    expect_u32(plan_buffer_is_full(&test_buffer), 1U, "the buffer is full once more");

    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "the full buffer is cleared");
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "a cleared buffer is not full");
}

static void test_is_full_uses_the_capacity_of_its_own_object(void) {
    reset_buffer();
    reset_second_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    expect_u32(
        plan_buffer_init(&second_buffer, second_buffer_entries, SECOND_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "the small buffer initializes"
    );
    for (uint32_t sequence = 1U; sequence < SECOND_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        (void)plan_buffer_try_push(&second_buffer, &block);
    }
    expect_u32(plan_buffer_is_full(&second_buffer), 0U, "the small buffer with a place left is not full");
    const MotionPlanBlock_t last = make_block(SECOND_BUFFER_CAPACITY);
    (void)plan_buffer_try_push(&second_buffer, &last);
    expect_u32(plan_buffer_is_full(&second_buffer), 1U, "the small buffer is full at its own capacity");
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "the large buffer is not full for the three blocks of the small one");
}

static void test_is_full_is_zero_without_a_usable_buffer(void) {
    reset_buffer();
    expect_u32(plan_buffer_is_full(NULL), 0U, "no object is not full");
    /* A zeroed object has count 0 and capacity 0: equal, but it is not a buffer that refuses a push as FULL. */
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "a buffer that was not initialized is not full");
    test_mutex_new_fails = 1U;
    expect_u32(
        plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_NO_MEMORY,
        "init without a mutex fails"
    );
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "a buffer whose init failed is not full");
    expect_u32(test_mutex_acquire_calls, 0U, "without a usable buffer nothing is locked");
}

static void test_is_full_locks_the_buffer_once(void) {
    reset_buffer();
    (void)plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY);
    expect_u32(plan_buffer_is_full(&test_buffer), 0U, "an empty buffer is not full");
    expect_u32(test_mutex_acquire_calls, 1U, "the fullness is read under the lock");
    expect_u32(test_mutex_release_calls, 1U, "the lock is released");
    expect_u32(test_mutex_max_depth, 1U, "the lock is not nested");
    expect_u32(test_mutex_wrong_handle, 0U, "the buffer mutex is used");
    expect_u32(test_error_handler_calls, 0U, "a successful lock raises no fault");

    test_mutex_acquire_result = osErrorParameter;
    (void)plan_buffer_is_full(&test_buffer);
    expect_u32(test_error_handler_calls, 1U, "a failed lock is a fatal error");
    test_mutex_acquire_result = osOK;
    test_mutex_release_result = osErrorResource;
    (void)plan_buffer_is_full(&test_buffer);
    expect_u32(test_error_handler_calls, 2U, "a failed unlock is a fatal error");
}

static void space_listener(void) {
    ++test_listener_calls;
    test_listener_mutex_depth = test_mutex_depth;
    test_listener_queued = test_buffer.count;
}

static void second_space_listener(void) { ++test_second_listener_calls; }

/* A listener that does what the task it wakes does: it uses the buffer at once. */
static void refilling_space_listener(void) {
    space_listener();
    ++test_listener_refills;
    const MotionPlanBlock_t refill = make_block(99U);
    test_listener_refill_status = plan_buffer_try_push(&test_buffer, &refill);
}

static void init_test_buffer_with_listener(void) {
    reset_buffer();
    expect_u32(
        plan_buffer_init(&test_buffer, test_buffer_entries, PLAN_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "buffer initializes for the listener test"
    );
    plan_buffer_set_space_listener(&test_buffer, space_listener);
}

static void push_blocks(PlanBuffer_t *buffer, uint32_t count) {
    for (uint32_t sequence = 1U; sequence <= count; ++sequence) {
        const MotionPlanBlock_t block = make_block(sequence);
        expect_u32(plan_buffer_try_push(buffer, &block), PLAN_BUFFER_STATUS_OK, "a block is queued for the listener test");
    }
}

static void test_space_listener_is_called_once_per_pop_after_the_room_is_free(void) {
    init_test_buffer_with_listener();
    push_blocks(&test_buffer, 2U);
    expect_u32(test_listener_calls, 0U, "queueing blocks takes room and calls no listener");

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the first block is taken");
    expect_block_equal(&output, 1U);
    expect_u32(test_listener_calls, 1U, "a pop that frees room calls the listener once");
    expect_u32(test_listener_queued, 1U, "the listener is called after the block has left the buffer");
    expect_u32(test_listener_mutex_depth, 0U, "the listener is called outside the lock of the buffer");

    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the second block is taken");
    expect_u32(test_listener_calls, 2U, "each pop calls the listener once");
    expect_u32(test_listener_queued, 0U, "the listener sees the buffer empty after the last pop");

    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_EMPTY, "an empty buffer gives nothing");
    expect_u32(test_listener_calls, 2U, "a pop that frees nothing calls no listener");
}

static void test_space_listener_is_called_once_per_discard_after_the_room_is_free(void) {
    init_test_buffer_with_listener();
    push_blocks(&test_buffer, 5U);
    const uint32_t epoch = plan_buffer_get_epoch(&test_buffer);

    expect_u32(plan_buffer_discard(&test_buffer, epoch, 2U), PLAN_BUFFER_STATUS_OK, "two blocks are dropped");
    expect_u32(test_listener_calls, 1U, "a discard of several blocks calls the listener once");
    expect_u32(test_listener_queued, 3U, "the listener is called after the blocks have left the buffer");
    expect_u32(test_listener_mutex_depth, 0U, "the listener is called outside the lock of the buffer");

    expect_u32(plan_buffer_discard(&test_buffer, epoch, 0U), PLAN_BUFFER_STATUS_OK, "dropping nothing succeeds");
    expect_u32(test_listener_calls, 1U, "a discard that frees nothing calls no listener");

    expect_u32(plan_buffer_discard(&test_buffer, epoch, 3U), PLAN_BUFFER_STATUS_OK, "the rest is dropped");
    expect_u32(test_listener_calls, 2U, "the next discard calls the listener again");
    expect_u32(test_listener_queued, 0U, "the listener sees the buffer empty");
}

static void test_space_listener_is_not_called_when_no_room_is_freed(void) {
    init_test_buffer_with_listener();
    push_blocks(&test_buffer, PLAN_BUFFER_CAPACITY);
    const MotionPlanBlock_t extra = make_block(PLAN_BUFFER_CAPACITY + 1U);
    expect_u32(plan_buffer_try_push(&test_buffer, &extra), PLAN_BUFFER_STATUS_FULL, "a full buffer refuses a block");
    expect_u32(plan_buffer_try_push(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "a null block is refused");
    expect_u32(test_listener_calls, 0U, "a push, accepted or refused, calls no listener");

    MotionPlanBlock_t output = {0};
    const uint32_t old_epoch = plan_buffer_get_epoch(&test_buffer);
    expect_u32(plan_buffer_try_pop(&test_buffer, NULL), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "a pop without output is refused");
    expect_u32(test_listener_calls, 0U, "a refused pop frees nothing and calls no listener");
    expect_u32(
        plan_buffer_discard(&test_buffer, old_epoch, PLAN_BUFFER_CAPACITY + 1U),
        PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
        "more blocks than queued cannot be dropped"
    );
    expect_u32(test_listener_calls, 0U, "a refused discard frees nothing and calls no listener");
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_buffer, &diagnostics);
    expect_u32(diagnostics.queued, PLAN_BUFFER_CAPACITY, "the refused calls changed nothing");

    expect_u32(plan_buffer_clear(&test_buffer), PLAN_BUFFER_STATUS_OK, "the full buffer is cleared");
    expect_u32(test_listener_calls, 0U, "a clear calls no listener: whoever clears knows");
    expect_u32(plan_buffer_discard(&test_buffer, old_epoch, 1U), PLAN_BUFFER_STATUS_STALE, "an old epoch cannot drop");
    expect_u32(test_listener_calls, 0U, "a stale discard calls no listener");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_EMPTY, "the cleared buffer is empty");
    expect_u32(test_listener_calls, 0U, "a pop of an empty buffer calls no listener");
}

static void test_space_listener_is_not_called_by_an_uninitialized_or_missing_buffer(void) {
    reset_buffer();
    plan_buffer_set_space_listener(&test_buffer, space_listener);
    MotionPlanBlock_t output = {0};
    expect_u32(
        plan_buffer_try_pop(&test_buffer, &output),
        PLAN_BUFFER_STATUS_NOT_INITIALIZED,
        "a buffer that was not initialized gives nothing"
    );
    expect_u32(
        plan_buffer_discard(&test_buffer, 0U, 1U),
        PLAN_BUFFER_STATUS_NOT_INITIALIZED,
        "a buffer that was not initialized drops nothing"
    );
    expect_u32(plan_buffer_try_pop(NULL, &output), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "no buffer gives nothing");
    expect_u32(plan_buffer_discard(NULL, 0U, 1U), PLAN_BUFFER_STATUS_INVALID_ARGUMENT, "no buffer drops nothing");
    expect_u32(test_listener_calls, 0U, "refused calls on an unusable buffer call no listener");
}

static void test_space_listener_can_be_replaced_removed_and_ignores_a_missing_buffer(void) {
    init_test_buffer_with_listener();
    push_blocks(&test_buffer, 4U);
    MotionPlanBlock_t output = {0};

    /* A missing buffer is ignored: the listener of the real one stays, and the one passed is not called. */
    plan_buffer_set_space_listener(NULL, second_space_listener);
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "a block is taken");
    expect_u32(test_listener_calls, 1U, "the listener of the buffer is unchanged by a call for no buffer");
    expect_u32(test_second_listener_calls, 0U, "the listener passed for no buffer is never called");

    plan_buffer_set_space_listener(&test_buffer, second_space_listener);
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "a block is taken after the replacement");
    expect_u32(test_listener_calls, 1U, "the replaced listener is no longer called");
    expect_u32(test_second_listener_calls, 1U, "the new listener is called");

    plan_buffer_set_space_listener(&test_buffer, NULL);
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "a block is taken without a listener");
    expect_u32(test_second_listener_calls, 1U, "a removed listener is not called");
    expect_u32(plan_buffer_discard(&test_buffer, plan_buffer_get_epoch(&test_buffer), 1U), PLAN_BUFFER_STATUS_OK, "a block is dropped");
    expect_u32(test_second_listener_calls, 1U, "a discard without a listener calls nothing");
    expect_u32(test_listener_calls, 1U, "the first listener stays removed");
}

static void test_space_listener_belongs_to_its_own_buffer(void) {
    init_test_buffer_with_listener();
    reset_second_buffer();
    expect_u32(
        plan_buffer_init(&second_buffer, second_buffer_entries, SECOND_BUFFER_CAPACITY),
        PLAN_BUFFER_STATUS_OK,
        "the second buffer initializes"
    );
    plan_buffer_set_space_listener(&second_buffer, second_space_listener);
    push_blocks(&test_buffer, 1U);
    push_blocks(&second_buffer, 1U);

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_pop(&second_buffer, &output), PLAN_BUFFER_STATUS_OK, "the second buffer gives its block");
    expect_u32(test_second_listener_calls, 1U, "the listener of the second buffer is called");
    expect_u32(test_listener_calls, 0U, "the listener of the first buffer is not");
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the first buffer gives its block");
    expect_u32(test_listener_calls, 1U, "the listener of the first buffer is called");
    expect_u32(test_second_listener_calls, 1U, "the listener of the second buffer is not");
}

static void test_space_listener_may_use_the_buffer_at_once(void) {
    init_test_buffer_with_listener();
    plan_buffer_set_space_listener(&test_buffer, refilling_space_listener);
    push_blocks(&test_buffer, PLAN_BUFFER_CAPACITY);
    test_mutex_max_depth = 0U;

    MotionPlanBlock_t output = {0};
    expect_u32(plan_buffer_try_pop(&test_buffer, &output), PLAN_BUFFER_STATUS_OK, "the front is taken from the full buffer");
    expect_u32(test_listener_refills, 1U, "the listener ran once");
    expect_u32(test_listener_refill_status, PLAN_BUFFER_STATUS_OK, "the freed place is free for the listener's push");
    expect_u32(test_mutex_max_depth, 1U, "the listener's push did not run inside the lock of the pop");
    expect_u32(plan_buffer_is_full(&test_buffer), 1U, "the listener took the freed place");
}

int main(void) {
    test_space_listener_is_called_once_per_pop_after_the_room_is_free();
    test_space_listener_is_called_once_per_discard_after_the_room_is_free();
    test_space_listener_is_not_called_when_no_room_is_freed();
    test_space_listener_is_not_called_by_an_uninitialized_or_missing_buffer();
    test_space_listener_can_be_replaced_removed_and_ignores_a_missing_buffer();
    test_space_listener_belongs_to_its_own_buffer();
    test_space_listener_may_use_the_buffer_at_once();
    test_is_full_follows_the_occupancy();
    test_is_full_uses_the_capacity_of_its_own_object();
    test_is_full_is_zero_without_a_usable_buffer();
    test_is_full_locks_the_buffer_once();
    test_init_rejects_bad_arguments();
    test_null_object_is_rejected_by_every_call();
    test_objects_are_independent();
    test_object_with_own_capacity_fills_and_wraps();
    test_init_clears_storage_and_counters();
    test_initialization_and_preinit_results();
    test_fifo_full_wraparound_and_copy();
    test_clear_cancels_only_pending_blocks();
    test_peek_reads_ahead_without_taking();
    test_discard_drops_the_front();
    test_peek_and_discard_wrap_around();
    test_clear_makes_old_positions_stale();
    test_peek_and_discard_before_init_and_with_bad_arguments();
    test_invalid_arguments_are_reported();
    test_diagnostic_counters_saturate();
    test_mutex_guards_every_operation();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Plan buffer unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Plan buffer unit tests passed.\n");
    return 0;
}

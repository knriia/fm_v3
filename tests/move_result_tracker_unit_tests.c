#include "motion/move_result_tracker.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_MAX_EMITS 256U

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

/* Fake emit target: records every result in order and the context that the tracker passed. */
typedef struct {
    MoveResult_t results[TEST_MAX_EMITS];
    uint32_t count;
} EmitLog_t;

static EmitLog_t emit_log;
static uint32_t emit_wrong_context_calls;

static void fake_emit(void *context, const MoveResult_t *result) {
    if (context != &emit_log) {
        ++emit_wrong_context_calls;
        return;
    }
    if (emit_log.count < TEST_MAX_EMITS) {
        emit_log.results[emit_log.count] = *result;
    }
    ++emit_log.count;
}

static MoveResultTracker_t tracker;

static void reset_all(void) {
    (void)memset(&tracker, 0, sizeof(tracker));
    (void)memset(&emit_log, 0, sizeof(emit_log));
    emit_wrong_context_calls = 0U;
}

static void update(const MoveResultProgress_t *progress) {
    move_result_tracker_update(&tracker, progress, fake_emit, &emit_log);
}

static void expect_result(uint32_t index, uint32_t sequence, MoveResultKind_t kind, const char *message) {
    if (index >= emit_log.count) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (no emit at index %lu)\n", message, (unsigned long)index);
        return;
    }
    expect_u32(emit_log.results[index].sequence, sequence, message);
    expect_u32((uint32_t)emit_log.results[index].kind, (uint32_t)kind, message);
}

static void add_moves(uint32_t first_sequence, uint32_t amount) {
    for (uint32_t i = 0U; i < amount; ++i) {
        (void)move_result_tracker_add(&tracker, first_sequence + i);
    }
}

static MoveResultProgress_t stop_progress(void) {
    MoveResultProgress_t progress;
    (void)memset(&progress, 0, sizeof(progress));
    progress.stop_resolved = 1U;
    progress.report_stop = 1U;
    progress.cause = 7U;
    progress.axis = 2U;
    progress.detail = 9U;
    return progress;
}

static void test_add_orders_and_pending(void) {
    reset_all();
    expect_u32(move_result_tracker_add(&tracker, 10U), 1U, "first order is 1");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "pending after 1 add");
    expect_u32(move_result_tracker_add(&tracker, 11U), 2U, "second order is 2");
    expect_u32(move_result_tracker_add(&tracker, 12U), 3U, "third order is 3");
    expect_u32(move_result_tracker_pending(&tracker), 3U, "pending after 3 adds");
    expect_u32(move_result_tracker_has_room(&tracker), 1U, "room with 3 moves");
}

static void test_add_capacity(void) {
    reset_all();
    for (uint32_t i = 0U; i < MOVE_RESULT_TRACKER_CAPACITY; ++i) {
        expect_u32(move_result_tracker_has_room(&tracker), 1U, "room before capacity");
        expect_u32(move_result_tracker_add(&tracker, 100U + i), i + 1U, "order of the move up to capacity");
    }
    expect_u32(MOVE_RESULT_TRACKER_CAPACITY, 128U, "capacity is 128");
    expect_u32(move_result_tracker_pending(&tracker), 128U, "pending at capacity");
    expect_u32(move_result_tracker_has_room(&tracker), 0U, "no room at capacity");
    expect_u32(move_result_tracker_add(&tracker, 999U), 0U, "add beyond capacity returns 0");
    expect_u32(move_result_tracker_pending(&tracker), 128U, "refused add keeps pending");
    expect_u32(tracker.last_order, 128U, "refused add keeps last_order");
}

static void test_null_arguments(void) {
    reset_all();
    add_moves(10U, 2U);
    MoveResultProgress_t progress = stop_progress();
    progress.played_blocks = 2U;
    progress.played_order = 2U;

    expect_u32(move_result_tracker_add(NULL, 1U), 0U, "add NULL tracker");
    expect_u32(move_result_tracker_has_room(NULL), 0U, "has_room NULL tracker");
    expect_u32(move_result_tracker_pending(NULL), 0U, "pending NULL tracker");
    move_result_tracker_reset(NULL, 3U);
    move_result_tracker_remove_last(NULL);
    move_result_tracker_update(NULL, &progress, fake_emit, &emit_log);
    move_result_tracker_update(&tracker, NULL, fake_emit, &emit_log);
    move_result_tracker_update(&tracker, &progress, NULL, &emit_log);
    expect_u32(emit_log.count, 0U, "NULL arguments emit nothing");
    expect_u32(move_result_tracker_pending(&tracker), 2U, "NULL arguments keep pending");
    expect_u32(tracker.last_order, 2U, "NULL arguments keep last_order");
    expect_u32(tracker.stop_reported, 0U, "NULL arguments do not touch stop_reported");
    expect_u32(tracker.generation, 0U, "NULL arguments keep generation");
}

static void test_remove_last(void) {
    reset_all();
    add_moves(10U, 3U);
    move_result_tracker_remove_last(&tracker);
    expect_u32(move_result_tracker_pending(&tracker), 2U, "remove_last drops one");
    expect_u32(move_result_tracker_add(&tracker, 50U), 3U, "next add gets the same order");

    const MoveResultProgress_t progress = {.played_blocks = 3U, .played_order = 3U};
    update(&progress);
    expect_u32(emit_log.count, 3U, "all three complete");
    expect_result(2U, 50U, MOVE_RESULT_COMPLETED, "re-added move keeps the new sequence");
}

static void test_remove_last_empty(void) {
    reset_all();
    tracker.last_order = 5U;
    move_result_tracker_remove_last(&tracker);
    expect_u32(move_result_tracker_pending(&tracker), 0U, "remove on empty keeps pending");
    expect_u32(tracker.last_order, 5U, "remove on empty keeps last_order");
}

static void test_remove_only_first(void) {
    reset_all();
    expect_u32(move_result_tracker_add(&tracker, 10U), 1U, "first add");
    move_result_tracker_remove_last(&tracker);
    expect_u32(tracker.last_order, UINT32_MAX, "last_order skips 0 going back");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "empty again");
    expect_u32(move_result_tracker_add(&tracker, 11U), 1U, "add after removal gives 1 again");
}

static void test_order_wrap_skips_zero(void) {
    reset_all();
    tracker.last_order = UINT32_MAX;
    expect_u32(move_result_tracker_add(&tracker, 10U), 1U, "add after UINT32_MAX gives 1, not 0");
    move_result_tracker_remove_last(&tracker);
    expect_u32(tracker.last_order, UINT32_MAX, "remove_last returns last_order to UINT32_MAX");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "no move left");
}

static void test_update_completed(void) {
    reset_all();
    add_moves(10U, 3U);
    MoveResultProgress_t progress;
    (void)memset(&progress, 0, sizeof(progress));
    progress.played_blocks = 2U;
    progress.played_order = 2U;
    update(&progress);
    expect_u32(emit_log.count, 2U, "two moves completed");
    expect_result(0U, 10U, MOVE_RESULT_COMPLETED, "first completed is sequence 10");
    expect_result(1U, 11U, MOVE_RESULT_COMPLETED, "second completed is sequence 11");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "one move still waits");
    expect_u32(emit_wrong_context_calls, 0U, "emit gets the context");

    update(&progress);
    expect_u32(emit_log.count, 2U, "same progress emits nothing new");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "same progress keeps pending");
}

static void test_zero_played_blocks_ignores_order(void) {
    reset_all();
    add_moves(10U, 3U);
    MoveResultProgress_t progress;
    (void)memset(&progress, 0, sizeof(progress));
    progress.played_blocks = 0U;
    progress.played_order = 2U;
    update(&progress);
    expect_u32(emit_log.count, 0U, "played_blocks 0 emits nothing");
    expect_u32(move_result_tracker_pending(&tracker), 3U, "played_blocks 0 keeps pending");
}

static void test_modular_order(void) {
    reset_all();
    tracker.last_order = 0xFFFFFFFEU;
    expect_u32(move_result_tracker_add(&tracker, 10U), 0xFFFFFFFFU, "order before wrap");
    expect_u32(move_result_tracker_add(&tracker, 11U), 1U, "order after wrap skips 0");
    expect_u32(move_result_tracker_add(&tracker, 12U), 2U, "order after wrap continues");
    MoveResultProgress_t progress;
    (void)memset(&progress, 0, sizeof(progress));
    progress.played_blocks = 2U;
    progress.played_order = 1U;
    update(&progress);
    expect_u32(emit_log.count, 2U, "watermark 1 covers two moves across the wrap");
    expect_result(0U, 10U, MOVE_RESULT_COMPLETED, "move before the wrap completes first");
    expect_result(1U, 11U, MOVE_RESULT_COMPLETED, "move after the wrap completes second");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "order 2 still waits");
}

static void test_stop_interrupts_first(void) {
    reset_all();
    add_moves(10U, 2U);
    MoveResultProgress_t progress = stop_progress();
    update(&progress);
    expect_u32(emit_log.count, 1U, "exactly one INTERRUPTED");
    expect_result(0U, 10U, MOVE_RESULT_INTERRUPTED, "oldest move is interrupted");
    expect_u32(emit_log.results[0].cause, 7U, "cause copied");
    expect_u32(emit_log.results[0].axis, 2U, "axis copied");
    expect_u32(emit_log.results[0].detail, 9U, "detail copied");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "second move stays");
    expect_u32(tracker.stop_reported, 1U, "stop marked as reported");

    update(&progress);
    expect_u32(emit_log.count, 1U, "same stop is not reported twice");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "same stop keeps pending");

    progress.stop_resolved = 0U;
    update(&progress);
    expect_u32(tracker.stop_reported, 0U, "running stream clears stop_reported");
    expect_u32(emit_log.count, 1U, "running stream emits nothing");

    progress.stop_resolved = 1U;
    update(&progress);
    expect_u32(emit_log.count, 2U, "new stop reports again");
    expect_result(1U, 11U, MOVE_RESULT_INTERRUPTED, "new stop interrupts the next move");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "no moves left");
}

static void test_stop_with_unplayed_order(void) {
    reset_all();
    add_moves(10U, 3U);
    MoveResultProgress_t progress = stop_progress();
    progress.has_unplayed = 1U;
    progress.unplayed_order = 2U;
    update(&progress);
    expect_u32(emit_log.count, 1U, "one INTERRUPTED for the named block");
    expect_result(0U, 11U, MOVE_RESULT_INTERRUPTED, "the move with unplayed order is interrupted");
    expect_u32(move_result_tracker_pending(&tracker), 2U, "other moves stay");

    const MoveResultProgress_t done = {.played_blocks = 3U, .played_order = 3U};
    update(&done);
    expect_u32(emit_log.count, 3U, "remaining moves complete");
    expect_result(1U, 10U, MOVE_RESULT_COMPLETED, "move before the cut-off completes first");
    expect_result(2U, 12U, MOVE_RESULT_COMPLETED, "queue closed the gap, next is sequence 12");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "all done");
}

static void test_stop_interrupts_last_without_shift(void) {
    reset_all();
    add_moves(10U, 3U);
    MoveResultProgress_t progress = stop_progress();
    progress.has_unplayed = 1U;
    progress.unplayed_order = 3U;
    update(&progress);
    expect_result(0U, 12U, MOVE_RESULT_INTERRUPTED, "last move is interrupted");
    expect_u32(emit_log.count, 1U, "one emit");
    expect_u32(move_result_tracker_pending(&tracker), 2U, "two moves stay");
    progress.stop_resolved = 0U;
    progress.played_blocks = 2U;
    progress.played_order = 2U;
    update(&progress);
    expect_result(1U, 10U, MOVE_RESULT_COMPLETED, "first stays in order");
    expect_result(2U, 11U, MOVE_RESULT_COMPLETED, "second stays in order");
}

static void test_stop_unplayed_beyond_all(void) {
    reset_all();
    add_moves(10U, 3U);
    MoveResultProgress_t progress = stop_progress();
    progress.has_unplayed = 1U;
    progress.unplayed_order = 4U;
    update(&progress);
    expect_u32(emit_log.count, 0U, "unplayed order beyond all moves emits nothing");
    expect_u32(move_result_tracker_pending(&tracker), 3U, "pending unchanged");
    expect_u32(tracker.stop_reported, 1U, "stop still marked as reported");
}

static void test_stop_not_reported(void) {
    reset_all();
    add_moves(10U, 2U);
    MoveResultProgress_t progress = stop_progress();
    progress.report_stop = 0U;
    update(&progress);
    expect_u32(emit_log.count, 0U, "report_stop 0 emits nothing");
    expect_u32(move_result_tracker_pending(&tracker), 2U, "report_stop 0 keeps pending");
    expect_u32(tracker.stop_reported, 1U, "report_stop 0 marks stop as reported");
}

static void test_stop_with_empty_pending(void) {
    reset_all();
    MoveResultProgress_t progress = stop_progress();
    update(&progress);
    expect_u32(emit_log.count, 0U, "empty tracker emits nothing on stop");
    expect_u32(tracker.stop_reported, 1U, "stop marked as reported");

    progress.has_unplayed = 1U;
    progress.unplayed_order = 1U;
    progress.stop_resolved = 0U;
    update(&progress);
    progress.stop_resolved = 1U;
    update(&progress);
    expect_u32(emit_log.count, 0U, "empty tracker with unplayed order emits nothing");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "still empty");
}

static void test_completed_then_interrupted(void) {
    reset_all();
    add_moves(10U, 3U);
    MoveResultProgress_t progress = stop_progress();
    progress.played_blocks = 1U;
    progress.played_order = 1U;
    progress.has_unplayed = 1U;
    progress.unplayed_order = 2U;
    update(&progress);
    expect_u32(emit_log.count, 2U, "one completed and one interrupted");
    expect_result(0U, 10U, MOVE_RESULT_COMPLETED, "COMPLETED comes first");
    expect_result(1U, 11U, MOVE_RESULT_INTERRUPTED, "INTERRUPTED comes second");
    expect_u32(emit_log.results[1].cause, 7U, "INTERRUPTED carries cause");
    expect_u32(move_result_tracker_pending(&tracker), 1U, "the third move stays");
}

static void test_generation_change(void) {
    reset_all();
    add_moves(10U, 3U);
    tracker.stop_reported = 1U;
    MoveResultProgress_t progress = stop_progress();
    progress.generation = 5U;
    progress.played_blocks = 3U;
    progress.played_order = 3U;
    update(&progress);
    expect_u32(emit_log.count, 0U, "new generation emits nothing");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "new generation clears pending");
    expect_u32(tracker.generation, 5U, "generation accepted");
    expect_u32(tracker.stop_reported, 0U, "stop_reported cleared");
    expect_u32(tracker.last_order, 3U, "numbering goes on");
}

static void test_reset_keeps_numbering(void) {
    reset_all();
    add_moves(10U, 3U);
    tracker.stop_reported = 1U;
    tracker.head = 5U;
    move_result_tracker_reset(&tracker, 8U);
    expect_u32(move_result_tracker_pending(&tracker), 0U, "reset clears pending");
    expect_u32(tracker.generation, 8U, "reset takes generation");
    expect_u32(tracker.stop_reported, 0U, "reset clears stop_reported");
    expect_u32(tracker.head, 0U, "reset clears head");
    expect_u32(move_result_tracker_add(&tracker, 20U), 4U, "numbering continues after reset");
}

static void test_ring_wrap_single_moves(void) {
    reset_all();
    for (uint32_t i = 1U; i <= 200U; ++i) {
        expect_u32(move_result_tracker_add(&tracker, 1000U + i), i, "order in the cycle");
        MoveResultProgress_t progress;
        (void)memset(&progress, 0, sizeof(progress));
        progress.played_blocks = i;
        progress.played_order = i;
        update(&progress);
        expect_u32(emit_log.count, i, "one result per cycle");
    }
    for (uint32_t i = 0U; i < 200U; ++i) {
        expect_result(i, 1001U + i, MOVE_RESULT_COMPLETED, "cycle result in order");
    }
    expect_u32(tracker.head, 200U % MOVE_RESULT_TRACKER_CAPACITY, "head wrapped");
    expect_u32(move_result_tracker_pending(&tracker), 0U, "nothing pending");
}

static void test_ring_wrap_batch(void) {
    reset_all();
    add_moves(1000U, 100U);
    MoveResultProgress_t progress;
    (void)memset(&progress, 0, sizeof(progress));
    progress.played_blocks = 100U;
    progress.played_order = 100U;
    update(&progress);
    expect_u32(emit_log.count, 100U, "first batch completes");

    add_moves(2000U, 60U);
    expect_u32(move_result_tracker_pending(&tracker), 60U, "batch across the ring end");
    progress.played_blocks = 160U;
    progress.played_order = 160U;
    update(&progress);
    expect_u32(emit_log.count, 160U, "second batch completes");
    expect_result(100U, 2000U, MOVE_RESULT_COMPLETED, "first of the wrapped batch");
    expect_result(159U, 2059U, MOVE_RESULT_COMPLETED, "last of the wrapped batch");

    add_moves(3000U, 3U);
    progress = stop_progress();
    progress.has_unplayed = 1U;
    progress.unplayed_order = 162U;
    update(&progress);
    expect_result(160U, 3001U, MOVE_RESULT_INTERRUPTED, "shift works on a wrapped ring");
    expect_u32(move_result_tracker_pending(&tracker), 2U, "two moves stay on the wrapped ring");
    progress.stop_resolved = 0U;
    progress.played_blocks = 163U;
    progress.played_order = 163U;
    update(&progress);
    expect_result(161U, 3000U, MOVE_RESULT_COMPLETED, "wrapped ring keeps the first");
    expect_result(162U, 3002U, MOVE_RESULT_COMPLETED, "wrapped ring closed the gap");
}

int main(void) {
    test_add_orders_and_pending();
    test_add_capacity();
    test_null_arguments();
    test_remove_last();
    test_remove_last_empty();
    test_remove_only_first();
    test_order_wrap_skips_zero();
    test_update_completed();
    test_zero_played_blocks_ignores_order();
    test_modular_order();
    test_stop_interrupts_first();
    test_stop_with_unplayed_order();
    test_stop_interrupts_last_without_shift();
    test_stop_unplayed_beyond_all();
    test_stop_not_reported();
    test_stop_with_empty_pending();
    test_completed_then_interrupted();
    test_generation_change();
    test_reset_keeps_numbering();
    test_ring_wrap_single_moves();
    test_ring_wrap_batch();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "%lu check(s) failed\n", (unsigned long)test_failures);
        return 1;
    }
    (void)printf("move_result_tracker_unit_tests: all passed\n");
    return 0;
}

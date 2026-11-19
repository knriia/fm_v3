#include "motion/move_result_tracker.h"

#include <stddef.h>

/* Orders wrap; the distance between two live orders stays far below half the range. */
static uint8_t move_result_tracker_order_reached(uint32_t order, uint32_t watermark) {
    return (int32_t)(watermark - order) >= 0 ? 1U : 0U;
}

static MoveResultPending_t *move_result_tracker_at(MoveResultTracker_t *tracker, uint32_t index) {
    return &tracker->pending[(tracker->head + index) % MOVE_RESULT_TRACKER_CAPACITY];
}

static void move_result_tracker_pop_front(MoveResultTracker_t *tracker) {
    tracker->head = (tracker->head + 1U) % MOVE_RESULT_TRACKER_CAPACITY;
    --tracker->count;
}

void move_result_tracker_reset(MoveResultTracker_t *tracker, uint32_t generation) {
    if (tracker == NULL) {
        return;
    }
    tracker->head = 0U;
    tracker->count = 0U;
    tracker->generation = generation;
    tracker->stop_reported = 0U;
}

uint8_t move_result_tracker_has_room(const MoveResultTracker_t *tracker) {
    return tracker != NULL && tracker->count < MOVE_RESULT_TRACKER_CAPACITY ? 1U : 0U;
}

uint32_t move_result_tracker_add(MoveResultTracker_t *tracker, uint32_t sequence) {
    if (move_result_tracker_has_room(tracker) == 0U) {
        return 0U;
    }
    ++tracker->last_order;
    if (tracker->last_order == 0U) {
        ++tracker->last_order;
    }
    MoveResultPending_t *const entry = move_result_tracker_at(tracker, tracker->count);
    entry->order = tracker->last_order;
    entry->sequence = sequence;
    ++tracker->count;
    return tracker->last_order;
}

void move_result_tracker_remove_last(MoveResultTracker_t *tracker) {
    if (tracker == NULL || tracker->count == 0U) {
        return;
    }
    --tracker->count;
    --tracker->last_order;
    if (tracker->last_order == 0U) {
        --tracker->last_order;
    }
}

uint32_t move_result_tracker_pending(const MoveResultTracker_t *tracker) {
    return tracker == NULL ? 0U : tracker->count;
}

static void move_result_tracker_emit_completed(
    MoveResultTracker_t *tracker,
    uint32_t watermark,
    MoveResultEmit_t emit,
    void *context
) {
    while (
        tracker->count != 0U &&
        move_result_tracker_order_reached(move_result_tracker_at(tracker, 0U)->order, watermark) != 0U
    ) {
        const MoveResult_t result = {
            .sequence = move_result_tracker_at(tracker, 0U)->sequence,
            .kind = MOVE_RESULT_COMPLETED,
        };
        move_result_tracker_pop_front(tracker);
        emit(context, &result);
    }
}

/* The move that was cut off: the one whose block the stream names, else the oldest that is still waiting. */
static uint8_t
move_result_tracker_find_cut_off(MoveResultTracker_t *tracker, const MoveResultProgress_t *progress, uint32_t *index) {
    if (progress->has_unplayed == 0U) {
        *index = 0U;
        return tracker->count != 0U ? 1U : 0U;
    }
    for (uint32_t position = 0U; position < tracker->count; ++position) {
        const uint32_t order = move_result_tracker_at(tracker, position)->order;
        if (move_result_tracker_order_reached(progress->unplayed_order, order) != 0U) {
            *index = position;
            return 1U;
        }
    }
    return 0U;
}

void move_result_tracker_update(
    MoveResultTracker_t *tracker,
    const MoveResultProgress_t *progress,
    MoveResultEmit_t emit,
    void *context
) {
    if (tracker == NULL || progress == NULL || emit == NULL) {
        return;
    }
    if (progress->generation != tracker->generation) {
        move_result_tracker_reset(tracker, progress->generation);
        return;
    }
    if (progress->played_blocks != 0U) {
        move_result_tracker_emit_completed(tracker, progress->played_order, emit, context);
    }
    if (progress->stop_resolved == 0U) {
        tracker->stop_reported = 0U;
        return;
    }
    if (tracker->stop_reported != 0U) {
        return;
    }
    tracker->stop_reported = 1U;
    uint32_t index = 0U;
    if (progress->report_stop == 0U || move_result_tracker_find_cut_off(tracker, progress, &index) == 0U) {
        return;
    }
    const MoveResultPending_t cut_off = *move_result_tracker_at(tracker, index);
    /* The moves queued behind it stay: STOP drops them, and a move that was played before it already got its result. */
    for (uint32_t position = index; position + 1U < tracker->count; ++position) {
        *move_result_tracker_at(tracker, position) = *move_result_tracker_at(tracker, position + 1U);
    }
    --tracker->count;
    const MoveResult_t result = {
        .sequence = cut_off.sequence,
        .kind = MOVE_RESULT_INTERRUPTED,
        .cause = progress->cause,
        .axis = progress->axis,
        .detail = progress->detail,
    };
    emit(context, &result);
}

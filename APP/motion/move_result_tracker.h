#ifndef FM_V3_MOVE_RESULT_TRACKER_H
#define FM_V3_MOVE_RESULT_TRACKER_H

#include <stdint.h>

/* Turns the progress of the step stream into the final answers to the moves of the client. Pure logic: the owner task
 * feeds it what the stream reports and posts what it emits.
 *
 * A move is registered when MotionTask puts its block into the plan queue and gets the number (order) that the block
 * carries. The stream reports the order of the last block it has played completely; every move up to it is done. When
 * the stream stops, the first move that was cut off gets the error, and only the first: the others wait behind it in
 * the queue and STOP drops them. Not reentrant; one owner task calls it. */

/* Moves that can wait for their result at the same time. The step feeder remembers the end of this many blocks; a move
 * beyond the capacity waits for room before MotionTask accepts it, so that neither queue can overflow. */
#define MOVE_RESULT_TRACKER_CAPACITY 128U

typedef struct {
    uint32_t order;
    uint32_t sequence;
} MoveResultPending_t;

/* What the stream reports; the owner fills it from the step generator snapshot. */
typedef struct {
    /* Increases when a STOP or a disconnect cancelled the stream. */
    uint32_t generation;
    /* Blocks played completely since the stream was reset: 0 means none, and played_order is not valid then. */
    uint32_t played_blocks;
    uint32_t played_order;
    /* 1 when the stream is halted, every channel is at rest and the played blocks are final. */
    uint8_t stop_resolved;
    /* 1 when the stop should be told to the client: not for a STOP or a disconnect, which answer themselves. */
    uint8_t report_stop;
    /* 1 when a block was cut off and unplayed_order is the order of the first such block. */
    uint8_t has_unplayed;
    uint32_t unplayed_order;
    /* Why the stream stopped, copied into the error. */
    uint8_t cause;
    uint8_t axis;
    uint8_t detail;
} MoveResultProgress_t;

typedef enum {
    /* The move was played to its end. */
    MOVE_RESULT_COMPLETED = 0,
    /* The stream stopped before the move was played. */
    MOVE_RESULT_INTERRUPTED
} MoveResultKind_t;

typedef struct {
    uint32_t sequence;
    MoveResultKind_t kind;
    uint8_t cause;
    uint8_t axis;
    uint8_t detail;
} MoveResult_t;

typedef void (*MoveResultEmit_t)(void *context, const MoveResult_t *result);

typedef struct {
    MoveResultPending_t pending[MOVE_RESULT_TRACKER_CAPACITY];
    uint32_t head;
    uint32_t count;
    /* Last order handed out; orders start at 1, so 0 never names a block. */
    uint32_t last_order;
    /* Generation of the progress that the tracker has seen last. */
    uint32_t generation;
    /* The error of the current stop has been given (or has none to give). */
    uint8_t stop_reported;
} MoveResultTracker_t;

/* Empties the tracker and takes `generation` as the current one. Order numbering goes on. */
void move_result_tracker_reset(MoveResultTracker_t *tracker, uint32_t generation);
/* 1 while another move can be registered. */
uint8_t move_result_tracker_has_room(const MoveResultTracker_t *tracker);
/* Registers a move and returns the order for its block, or 0 when the tracker is full or NULL. */
uint32_t move_result_tracker_add(MoveResultTracker_t *tracker, uint32_t sequence);
/* Takes back the last registered move: its block did not get into the queue. */
void move_result_tracker_remove_last(MoveResultTracker_t *tracker);
/* Number of moves waiting for a result. */
uint32_t move_result_tracker_pending(const MoveResultTracker_t *tracker);
/* Applies the progress: emits COMPLETED for every move up to the played order, in order, then, once per stop, the
 * INTERRUPTED of the first move that was cut off. A progress of another generation cancels the waiting moves without
 * an answer (STOP and a disconnect answer for themselves). */
void move_result_tracker_update(
    MoveResultTracker_t *tracker,
    const MoveResultProgress_t *progress,
    MoveResultEmit_t emit,
    void *context
);

#endif /* FM_V3_MOVE_RESULT_TRACKER_H */

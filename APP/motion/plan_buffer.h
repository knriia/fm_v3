#ifndef FM_V3_PLAN_BUFFER_H
#define FM_V3_PLAN_BUFFER_H

#include "motion_planner.h"

#include <stdint.h>

#define PLAN_BUFFER_CAPACITY 100U

typedef enum {
    PLAN_BUFFER_STATUS_OK = 0,
    PLAN_BUFFER_STATUS_FULL,
    PLAN_BUFFER_STATUS_EMPTY,
    PLAN_BUFFER_STATUS_INVALID_ARGUMENT,
    PLAN_BUFFER_STATUS_NOT_INITIALIZED,
    PLAN_BUFFER_STATUS_ALREADY_INITIALIZED,
    PLAN_BUFFER_STATUS_NO_MEMORY,
    /* The buffer was cleared since the epoch that the caller read: its positions no longer mean anything. */
    PLAN_BUFFER_STATUS_STALE
} PlanBufferStatus_t;

/* Sent as is in the diagnostic frame. */
typedef struct {
    uint32_t queued;
    uint32_t max_queued;
    uint32_t enqueued_total;
    uint32_t dequeued_total;
    uint32_t full_rejections;
    uint32_t rejected_operations;
    uint32_t cancelled_blocks;
} PlanBufferDiagnostics_t;

/* Shared by MotionTask (push, clear) and the step generator task (pop); a mutex guards every operation.
 * Calls never wait for space or data, only for the short critical part of another call. Not for ISRs.
 * Call init before the scheduler starts tasks that use the buffer. */
PlanBufferStatus_t plan_buffer_init(void);
PlanBufferStatus_t plan_buffer_try_push(const MotionPlanBlock_t *block);
/* Takes the front block. The step generator reads ahead with peek and discard below instead; this is the plain queue
 * operation. */
PlanBufferStatus_t plan_buffer_try_pop(MotionPlanBlock_t *block);
/* Reading ahead without taking: the one reader of the buffer may look at the blocks behind the first one and drop the
 * front once every part of its work has passed it. A position is an offset from the front; it moves when blocks are
 * discarded, so the reader keeps its offsets relative to the front it knows. A clear invalidates all offsets: it
 * changes the epoch, and a call that carries an older one gets STALE and changes nothing.
 * peek copies the block at `offset` (0 is the front); EMPTY when the buffer holds no block there. */
PlanBufferStatus_t plan_buffer_try_peek(uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block);
/* Drops the first `count` blocks; INVALID_ARGUMENT when fewer are queued. Counted as dequeued. */
PlanBufferStatus_t plan_buffer_discard(uint32_t epoch, uint32_t count);
/* Changes with every clear. Read it when the reader starts working on the buffer. */
uint32_t plan_buffer_get_epoch(void);
/* Cancels all pending blocks; call for STOP, motion errors, and future FAULT transitions. */
PlanBufferStatus_t plan_buffer_clear(void);
PlanBufferStatus_t plan_buffer_get_diagnostics(PlanBufferDiagnostics_t *diagnostics);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void plan_buffer_test_reset(void);
void plan_buffer_test_seed_counters(const PlanBufferDiagnostics_t *diagnostics);
#endif

#endif /* FM_V3_PLAN_BUFFER_H */

#ifndef FM_V3_PLAN_BUFFER_H
#define FM_V3_PLAN_BUFFER_H

#include "motion_planner.h"

#include "cmsis_os.h"

#include <stdint.h>

/* Capacity of the printing queue; a queue of another use brings its own storage. */
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

/* One queue of plan blocks. The caller owns the object and its storage and passes them to init; every function takes
 * the object. The fields are private: only plan_buffer.c touches them. */
typedef struct {
    osMutexId_t mutex;
    MotionPlanBlock_t *entries;
    uint32_t capacity;
    volatile uint32_t initialized;
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
    volatile uint32_t epoch;
    volatile uint32_t max_queued;
    volatile uint32_t enqueued_total;
    volatile uint32_t dequeued_total;
    volatile uint32_t full_rejections;
    volatile uint32_t rejected_operations;
    volatile uint32_t cancelled_blocks;
} PlanBuffer_t;

/* Shared by one writer task (push, clear) and one reader task (pop, peek, discard); a mutex guards every operation.
 * Calls never wait for space or data, only for the short critical part of another call. Not for ISRs.
 * Call init before the scheduler starts tasks that use the buffer. `self` must start zeroed. `entries` holds
 * `capacity` blocks and outlives the buffer; a NULL `self` or `entries` or a zero capacity is INVALID_ARGUMENT, and
 * so is a NULL `self` in every other call. */
PlanBufferStatus_t plan_buffer_init(PlanBuffer_t *self, MotionPlanBlock_t *entries, uint32_t capacity);
PlanBufferStatus_t plan_buffer_try_push(PlanBuffer_t *self, const MotionPlanBlock_t *block);
/* Takes the front block. The step generator reads ahead with peek and discard below instead; this is the plain queue
 * operation. */
PlanBufferStatus_t plan_buffer_try_pop(PlanBuffer_t *self, MotionPlanBlock_t *block);
/* Reading ahead without taking: the one reader of the buffer may look at the blocks behind the first one and drop the
 * front once every part of its work has passed it. A position is an offset from the front; it moves when blocks are
 * discarded, so the reader keeps its offsets relative to the front it knows. A clear invalidates all offsets: it
 * changes the epoch, and a call that carries an older one gets STALE and changes nothing.
 * peek copies the block at `offset` (0 is the front); EMPTY when the buffer holds no block there. */
PlanBufferStatus_t plan_buffer_try_peek(PlanBuffer_t *self, uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block);
/* Drops the first `count` blocks; INVALID_ARGUMENT when fewer are queued. Counted as dequeued. */
PlanBufferStatus_t plan_buffer_discard(PlanBuffer_t *self, uint32_t epoch, uint32_t count);
/* Changes with every clear. Read it when the reader starts working on the buffer. 0 for a NULL `self`. */
uint32_t plan_buffer_get_epoch(const PlanBuffer_t *self);
/* Cancels all pending blocks; call for STOP, motion errors, and future FAULT transitions. */
PlanBufferStatus_t plan_buffer_clear(PlanBuffer_t *self);
PlanBufferStatus_t plan_buffer_get_diagnostics(PlanBuffer_t *self, PlanBufferDiagnostics_t *diagnostics);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void plan_buffer_test_seed_counters(PlanBuffer_t *self, const PlanBufferDiagnostics_t *diagnostics);
#endif

#endif /* FM_V3_PLAN_BUFFER_H */

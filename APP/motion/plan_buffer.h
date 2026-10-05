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
    PLAN_BUFFER_STATUS_ALREADY_INITIALIZED
} PlanBufferStatus_t;

typedef struct {
    uint32_t initialized;
    uint32_t capacity;
    uint32_t queued;
    uint32_t available;
    uint32_t max_queued;
    uint32_t enqueued_total;
    uint32_t dequeued_total;
    uint32_t full_rejections;
    uint32_t empty_reads;
    uint32_t rejected_operations;
    uint32_t clear_calls;
    uint32_t cancelled_blocks;
} PlanBufferDiagnostics_t;

/* The MotionTask pipeline owns queue operations; calls are bounded and never wait for space. */
PlanBufferStatus_t plan_buffer_init(void);
PlanBufferStatus_t plan_buffer_try_push(const MotionPlanBlock_t *block);
PlanBufferStatus_t plan_buffer_try_pop(MotionPlanBlock_t *block);
/* Cancels all pending blocks; call for STOP, motion errors, and future FAULT transitions. */
PlanBufferStatus_t plan_buffer_clear(void);
PlanBufferStatus_t plan_buffer_get_diagnostics(PlanBufferDiagnostics_t *diagnostics);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void plan_buffer_test_reset(void);
void plan_buffer_test_seed_counters(const PlanBufferDiagnostics_t *diagnostics);
#endif

#endif /* FM_V3_PLAN_BUFFER_H */

#ifndef FM_V3_EXECUTION_BUFFER_H
#define FM_V3_EXECUTION_BUFFER_H

#include "motion/motion_event.h"

#include <stdint.h>

#define EXECUTION_BUFFER_CAPACITY 512U

typedef enum {
    EXECUTION_BUFFER_STATUS_OK = 0,
    EXECUTION_BUFFER_STATUS_FULL,
    EXECUTION_BUFFER_STATUS_EMPTY,
    EXECUTION_BUFFER_STATUS_INVALID_ARGUMENT
} ExecutionBufferStatus_t;

/* The producer is woken once per drain to this level, not on every step. */
#define EXECUTION_BUFFER_LOW_WATERMARK (EXECUTION_BUFFER_CAPACITY / 2U)

/* Runs in the consumer ISR when a pop leaves exactly EXECUTION_BUFFER_LOW_WATERMARK events; must be ISR-safe. */
typedef void (*ExecutionBufferSpaceListener_t)(void);

/* One producer task and one timer-ISR consumer. Clear only while both are quiescent. */
void execution_buffer_init(void);
/* Set once at startup, before the consumer runs. */
void execution_buffer_set_space_listener(ExecutionBufferSpaceListener_t listener);
ExecutionBufferStatus_t execution_buffer_try_push(const MotionExecutionEvent_t *event);
ExecutionBufferStatus_t execution_buffer_try_pop(MotionExecutionEvent_t *event);
uint32_t execution_buffer_count(void);
void execution_buffer_clear(void);

#endif /* FM_V3_EXECUTION_BUFFER_H */

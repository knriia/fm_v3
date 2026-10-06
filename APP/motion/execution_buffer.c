#include "motion/execution_buffer.h"

#include <stdatomic.h>
#include <stddef.h>

#define EXECUTION_BUFFER_INDEX_MASK (EXECUTION_BUFFER_CAPACITY - 1U)

_Static_assert((EXECUTION_BUFFER_CAPACITY & EXECUTION_BUFFER_INDEX_MASK) == 0U, "capacity must be a power of two");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "32-bit atomics must be lock-free for ISR use");

static MotionExecutionEvent_t execution_buffer_entries[EXECUTION_BUFFER_CAPACITY];
static _Atomic uint32_t execution_buffer_write_position;
static _Atomic uint32_t execution_buffer_read_position;

void execution_buffer_init(void) {
    atomic_store_explicit(&execution_buffer_read_position, 0U, memory_order_relaxed);
    atomic_store_explicit(&execution_buffer_write_position, 0U, memory_order_relaxed);
}

ExecutionBufferStatus_t execution_buffer_try_push(const MotionExecutionEvent_t *event) {
    if (event == NULL) {
        return EXECUTION_BUFFER_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t write_position = atomic_load_explicit(&execution_buffer_write_position, memory_order_relaxed);
    const uint32_t read_position = atomic_load_explicit(&execution_buffer_read_position, memory_order_acquire);
    if ((uint32_t)(write_position - read_position) >= EXECUTION_BUFFER_CAPACITY) {
        return EXECUTION_BUFFER_STATUS_FULL;
    }

    execution_buffer_entries[write_position & EXECUTION_BUFFER_INDEX_MASK] = *event;
    atomic_store_explicit(&execution_buffer_write_position, write_position + 1U, memory_order_release);
    return EXECUTION_BUFFER_STATUS_OK;
}

ExecutionBufferStatus_t execution_buffer_try_pop(MotionExecutionEvent_t *event) {
    if (event == NULL) {
        return EXECUTION_BUFFER_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t read_position = atomic_load_explicit(&execution_buffer_read_position, memory_order_relaxed);
    const uint32_t write_position = atomic_load_explicit(&execution_buffer_write_position, memory_order_acquire);
    if (read_position == write_position) {
        return EXECUTION_BUFFER_STATUS_EMPTY;
    }

    *event = execution_buffer_entries[read_position & EXECUTION_BUFFER_INDEX_MASK];
    atomic_store_explicit(&execution_buffer_read_position, read_position + 1U, memory_order_release);
    return EXECUTION_BUFFER_STATUS_OK;
}

uint32_t execution_buffer_count(void) {
    const uint32_t write_position = atomic_load_explicit(&execution_buffer_write_position, memory_order_acquire);
    const uint32_t read_position = atomic_load_explicit(&execution_buffer_read_position, memory_order_acquire);
    return (uint32_t)(write_position - read_position);
}

void execution_buffer_clear(void) {
    atomic_store_explicit(&execution_buffer_read_position, 0U, memory_order_relaxed);
    atomic_store_explicit(&execution_buffer_write_position, 0U, memory_order_relaxed);
}

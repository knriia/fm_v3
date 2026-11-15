#include "motion/plan_buffer.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include <stddef.h>
#include <string.h>

/* Priority inheritance keeps a low-priority holder from stalling the higher-priority step generator. */
static const osMutexAttr_t plan_buffer_mutex_attributes = {
    .name = "PlanBuffer",
    .attr_bits = osMutexPrioInherit,
};

static void plan_buffer_counter_add(volatile uint32_t *counter, uint32_t increment) {
    if (increment > (UINT32_MAX - *counter)) {
        *counter = UINT32_MAX;
    } else {
        *counter += increment;
    }
}

static void plan_buffer_counter_increment(volatile uint32_t *counter) { plan_buffer_counter_add(counter, 1U); }

static void plan_buffer_lock(PlanBuffer_t *self) {
    const osStatus_t status = osMutexAcquire(self->mutex, osWaitForever);
    configASSERT(status == osOK);
}

static void plan_buffer_unlock(PlanBuffer_t *self) {
    const osStatus_t status = osMutexRelease(self->mutex);
    configASSERT(status == osOK);
}

PlanBufferStatus_t plan_buffer_init(PlanBuffer_t *self, MotionPlanBlock_t *entries, uint32_t capacity) {
    if (self == NULL || entries == NULL || capacity == 0U) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }
    if (self->initialized != 0U) {
        return PLAN_BUFFER_STATUS_ALREADY_INITIALIZED;
    }
    self->mutex = osMutexNew(&plan_buffer_mutex_attributes);
    if (self->mutex == NULL) {
        return PLAN_BUFFER_STATUS_NO_MEMORY;
    }

    (void)memset(entries, 0, (size_t)capacity * sizeof(entries[0]));
    self->entries = entries;
    self->capacity = capacity;
    self->head = 0U;
    self->tail = 0U;
    self->count = 0U;
    self->epoch = 0U;
    self->max_queued = 0U;
    self->enqueued_total = 0U;
    self->dequeued_total = 0U;
    self->full_rejections = 0U;
    self->rejected_operations = 0U;
    self->cancelled_blocks = 0U;
    self->initialized = 1U;
    return PLAN_BUFFER_STATUS_OK;
}

/* Before init no task uses the buffer yet, so rejections there are counted without the mutex. A NULL buffer has no
 * counter to count into. */
PlanBufferStatus_t plan_buffer_try_push(PlanBuffer_t *self, const MotionPlanBlock_t *block) {
    if (self == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }
    if (self->initialized == 0U) {
        plan_buffer_counter_increment(&self->rejected_operations);
        return block == NULL ? PLAN_BUFFER_STATUS_INVALID_ARGUMENT : PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    PlanBufferStatus_t status = PLAN_BUFFER_STATUS_OK;
    plan_buffer_lock(self);
    if (block == NULL) {
        plan_buffer_counter_increment(&self->rejected_operations);
        status = PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    } else if (self->count == self->capacity) {
        plan_buffer_counter_increment(&self->full_rejections);
        status = PLAN_BUFFER_STATUS_FULL;
    } else {
        self->entries[self->tail] = *block;
        self->tail = (self->tail + 1U) % self->capacity;
        ++self->count;
        plan_buffer_counter_increment(&self->enqueued_total);
        if (self->count > self->max_queued) {
            self->max_queued = self->count;
        }
    }
    plan_buffer_unlock(self);
    return status;
}

PlanBufferStatus_t plan_buffer_try_pop(PlanBuffer_t *self, MotionPlanBlock_t *block) {
    if (self == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }
    if (self->initialized == 0U) {
        plan_buffer_counter_increment(&self->rejected_operations);
        return block == NULL ? PLAN_BUFFER_STATUS_INVALID_ARGUMENT : PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    PlanBufferStatus_t status = PLAN_BUFFER_STATUS_OK;
    plan_buffer_lock(self);
    if (block == NULL) {
        plan_buffer_counter_increment(&self->rejected_operations);
        status = PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    } else if (self->count == 0U) {
        status = PLAN_BUFFER_STATUS_EMPTY;
    } else {
        *block = self->entries[self->head];
        self->head = (self->head + 1U) % self->capacity;
        --self->count;
        plan_buffer_counter_increment(&self->dequeued_total);
    }
    plan_buffer_unlock(self);
    return status;
}

PlanBufferStatus_t plan_buffer_try_peek(PlanBuffer_t *self, uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block) {
    if (self == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }
    if (self->initialized == 0U) {
        plan_buffer_counter_increment(&self->rejected_operations);
        return block == NULL ? PLAN_BUFFER_STATUS_INVALID_ARGUMENT : PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    PlanBufferStatus_t status = PLAN_BUFFER_STATUS_OK;
    plan_buffer_lock(self);
    if (block == NULL) {
        plan_buffer_counter_increment(&self->rejected_operations);
        status = PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    } else if (epoch != self->epoch) {
        plan_buffer_counter_increment(&self->rejected_operations);
        status = PLAN_BUFFER_STATUS_STALE;
    } else if (offset >= self->count) {
        status = PLAN_BUFFER_STATUS_EMPTY;
    } else {
        *block = self->entries[(self->head + offset) % self->capacity];
    }
    plan_buffer_unlock(self);
    return status;
}

PlanBufferStatus_t plan_buffer_discard(PlanBuffer_t *self, uint32_t epoch, uint32_t count) {
    if (self == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }
    if (self->initialized == 0U) {
        plan_buffer_counter_increment(&self->rejected_operations);
        return PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    PlanBufferStatus_t status = PLAN_BUFFER_STATUS_OK;
    plan_buffer_lock(self);
    if (epoch != self->epoch) {
        plan_buffer_counter_increment(&self->rejected_operations);
        status = PLAN_BUFFER_STATUS_STALE;
    } else if (count > self->count) {
        plan_buffer_counter_increment(&self->rejected_operations);
        status = PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    } else {
        self->head = (self->head + count) % self->capacity;
        self->count -= count;
        plan_buffer_counter_add(&self->dequeued_total, count);
    }
    plan_buffer_unlock(self);
    return status;
}

uint32_t plan_buffer_get_epoch(const PlanBuffer_t *self) { return self == NULL ? 0U : self->epoch; }

PlanBufferStatus_t plan_buffer_clear(PlanBuffer_t *self) {
    if (self == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }
    if (self->initialized == 0U) {
        plan_buffer_counter_increment(&self->rejected_operations);
        return PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    plan_buffer_lock(self);
    plan_buffer_counter_add(&self->cancelled_blocks, self->count);
    ++self->epoch;
    self->head = 0U;
    self->tail = 0U;
    self->count = 0U;
    plan_buffer_unlock(self);
    return PLAN_BUFFER_STATUS_OK;
}

PlanBufferStatus_t plan_buffer_get_diagnostics(PlanBuffer_t *self, PlanBufferDiagnostics_t *diagnostics) {
    if (self == NULL || diagnostics == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }

    const uint8_t locked = self->initialized != 0U ? 1U : 0U;
    if (locked != 0U) {
        plan_buffer_lock(self);
    }
    diagnostics->queued = self->count;
    diagnostics->max_queued = self->max_queued;
    diagnostics->enqueued_total = self->enqueued_total;
    diagnostics->dequeued_total = self->dequeued_total;
    diagnostics->full_rejections = self->full_rejections;
    diagnostics->rejected_operations = self->rejected_operations;
    diagnostics->cancelled_blocks = self->cancelled_blocks;
    if (locked != 0U) {
        plan_buffer_unlock(self);
    }
    return PLAN_BUFFER_STATUS_OK;
}

#ifdef FM_V3_ENABLE_TEST_HOOKS
void plan_buffer_test_seed_counters(PlanBuffer_t *self, const PlanBufferDiagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    self->max_queued = diagnostics->max_queued;
    self->enqueued_total = diagnostics->enqueued_total;
    self->dequeued_total = diagnostics->dequeued_total;
    self->full_rejections = diagnostics->full_rejections;
    self->rejected_operations = diagnostics->rejected_operations;
    self->cancelled_blocks = diagnostics->cancelled_blocks;
}
#endif

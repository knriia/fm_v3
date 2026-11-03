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

static osMutexId_t plan_buffer_mutex;
static MotionPlanBlock_t plan_buffer_entries[PLAN_BUFFER_CAPACITY];
static volatile uint32_t plan_buffer_initialized;
static volatile uint32_t plan_buffer_head;
static volatile uint32_t plan_buffer_tail;
static volatile uint32_t plan_buffer_count;
static volatile uint32_t plan_buffer_max_queued;
static volatile uint32_t plan_buffer_enqueued_total;
static volatile uint32_t plan_buffer_dequeued_total;
static volatile uint32_t plan_buffer_full_rejections;
static volatile uint32_t plan_buffer_empty_reads;
static volatile uint32_t plan_buffer_rejected_operations;
static volatile uint32_t plan_buffer_clear_calls;
static volatile uint32_t plan_buffer_cancelled_blocks;

static void plan_buffer_counter_add(volatile uint32_t *counter, uint32_t increment) {
    if (increment > (UINT32_MAX - *counter)) {
        *counter = UINT32_MAX;
    } else {
        *counter += increment;
    }
}

static void plan_buffer_counter_increment(volatile uint32_t *counter) { plan_buffer_counter_add(counter, 1U); }

static void plan_buffer_lock(void) {
    const osStatus_t status = osMutexAcquire(plan_buffer_mutex, osWaitForever);
    configASSERT(status == osOK);
}

static void plan_buffer_unlock(void) {
    const osStatus_t status = osMutexRelease(plan_buffer_mutex);
    configASSERT(status == osOK);
}

PlanBufferStatus_t plan_buffer_init(void) {
    if (plan_buffer_initialized != 0U) {
        return PLAN_BUFFER_STATUS_ALREADY_INITIALIZED;
    }
    plan_buffer_mutex = osMutexNew(&plan_buffer_mutex_attributes);
    if (plan_buffer_mutex == NULL) {
        return PLAN_BUFFER_STATUS_NO_MEMORY;
    }

    (void)memset(plan_buffer_entries, 0, sizeof(plan_buffer_entries));
    plan_buffer_head = 0U;
    plan_buffer_tail = 0U;
    plan_buffer_count = 0U;
    plan_buffer_max_queued = 0U;
    plan_buffer_enqueued_total = 0U;
    plan_buffer_dequeued_total = 0U;
    plan_buffer_full_rejections = 0U;
    plan_buffer_empty_reads = 0U;
    plan_buffer_rejected_operations = 0U;
    plan_buffer_clear_calls = 0U;
    plan_buffer_cancelled_blocks = 0U;
    plan_buffer_initialized = 1U;
    return PLAN_BUFFER_STATUS_OK;
}

/* Before init no task uses the buffer yet, so rejections there are counted without the mutex. */
PlanBufferStatus_t plan_buffer_try_push(const MotionPlanBlock_t *block) {
    if (plan_buffer_initialized == 0U) {
        plan_buffer_counter_increment(&plan_buffer_rejected_operations);
        return block == NULL ? PLAN_BUFFER_STATUS_INVALID_ARGUMENT : PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    PlanBufferStatus_t status = PLAN_BUFFER_STATUS_OK;
    plan_buffer_lock();
    if (block == NULL) {
        plan_buffer_counter_increment(&plan_buffer_rejected_operations);
        status = PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    } else if (plan_buffer_count == PLAN_BUFFER_CAPACITY) {
        plan_buffer_counter_increment(&plan_buffer_full_rejections);
        status = PLAN_BUFFER_STATUS_FULL;
    } else {
        plan_buffer_entries[plan_buffer_tail] = *block;
        plan_buffer_tail = (plan_buffer_tail + 1U) % PLAN_BUFFER_CAPACITY;
        ++plan_buffer_count;
        plan_buffer_counter_increment(&plan_buffer_enqueued_total);
        if (plan_buffer_count > plan_buffer_max_queued) {
            plan_buffer_max_queued = plan_buffer_count;
        }
    }
    plan_buffer_unlock();
    return status;
}

PlanBufferStatus_t plan_buffer_try_pop(MotionPlanBlock_t *block) {
    if (plan_buffer_initialized == 0U) {
        plan_buffer_counter_increment(&plan_buffer_rejected_operations);
        return block == NULL ? PLAN_BUFFER_STATUS_INVALID_ARGUMENT : PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    PlanBufferStatus_t status = PLAN_BUFFER_STATUS_OK;
    plan_buffer_lock();
    if (block == NULL) {
        plan_buffer_counter_increment(&plan_buffer_rejected_operations);
        status = PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    } else if (plan_buffer_count == 0U) {
        plan_buffer_counter_increment(&plan_buffer_empty_reads);
        status = PLAN_BUFFER_STATUS_EMPTY;
    } else {
        *block = plan_buffer_entries[plan_buffer_head];
        plan_buffer_head = (plan_buffer_head + 1U) % PLAN_BUFFER_CAPACITY;
        --plan_buffer_count;
        plan_buffer_counter_increment(&plan_buffer_dequeued_total);
    }
    plan_buffer_unlock();
    return status;
}

PlanBufferStatus_t plan_buffer_clear(void) {
    if (plan_buffer_initialized == 0U) {
        plan_buffer_counter_increment(&plan_buffer_rejected_operations);
        return PLAN_BUFFER_STATUS_NOT_INITIALIZED;
    }

    plan_buffer_lock();
    plan_buffer_counter_increment(&plan_buffer_clear_calls);
    plan_buffer_counter_add(&plan_buffer_cancelled_blocks, plan_buffer_count);
    plan_buffer_head = 0U;
    plan_buffer_tail = 0U;
    plan_buffer_count = 0U;
    plan_buffer_unlock();
    return PLAN_BUFFER_STATUS_OK;
}

PlanBufferStatus_t plan_buffer_get_diagnostics(PlanBufferDiagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return PLAN_BUFFER_STATUS_INVALID_ARGUMENT;
    }

    const uint8_t locked = plan_buffer_initialized != 0U ? 1U : 0U;
    if (locked != 0U) {
        plan_buffer_lock();
    }
    diagnostics->initialized = plan_buffer_initialized;
    diagnostics->capacity = PLAN_BUFFER_CAPACITY;
    diagnostics->queued = plan_buffer_initialized != 0U ? plan_buffer_count : 0U;
    diagnostics->available = plan_buffer_initialized != 0U ? PLAN_BUFFER_CAPACITY - plan_buffer_count : 0U;
    diagnostics->max_queued = plan_buffer_max_queued;
    diagnostics->enqueued_total = plan_buffer_enqueued_total;
    diagnostics->dequeued_total = plan_buffer_dequeued_total;
    diagnostics->full_rejections = plan_buffer_full_rejections;
    diagnostics->empty_reads = plan_buffer_empty_reads;
    diagnostics->rejected_operations = plan_buffer_rejected_operations;
    diagnostics->clear_calls = plan_buffer_clear_calls;
    diagnostics->cancelled_blocks = plan_buffer_cancelled_blocks;
    if (locked != 0U) {
        plan_buffer_unlock();
    }
    return PLAN_BUFFER_STATUS_OK;
}

#ifdef FM_V3_ENABLE_TEST_HOOKS
void plan_buffer_test_reset(void) {
    (void)memset(plan_buffer_entries, 0, sizeof(plan_buffer_entries));
    plan_buffer_mutex = NULL;
    plan_buffer_initialized = 0U;
    plan_buffer_head = 0U;
    plan_buffer_tail = 0U;
    plan_buffer_count = 0U;
    plan_buffer_max_queued = 0U;
    plan_buffer_enqueued_total = 0U;
    plan_buffer_dequeued_total = 0U;
    plan_buffer_full_rejections = 0U;
    plan_buffer_empty_reads = 0U;
    plan_buffer_rejected_operations = 0U;
    plan_buffer_clear_calls = 0U;
    plan_buffer_cancelled_blocks = 0U;
}

void plan_buffer_test_seed_counters(const PlanBufferDiagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    plan_buffer_max_queued = diagnostics->max_queued;
    plan_buffer_enqueued_total = diagnostics->enqueued_total;
    plan_buffer_dequeued_total = diagnostics->dequeued_total;
    plan_buffer_full_rejections = diagnostics->full_rejections;
    plan_buffer_empty_reads = diagnostics->empty_reads;
    plan_buffer_rejected_operations = diagnostics->rejected_operations;
    plan_buffer_clear_calls = diagnostics->clear_calls;
    plan_buffer_cancelled_blocks = diagnostics->cancelled_blocks;
}
#endif

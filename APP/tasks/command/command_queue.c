#include "command/command_queue.h"

#include <stddef.h>

static osMessageQueueId_t command_queue_handle;
static volatile uint32_t command_queue_enqueued_total;
static volatile uint32_t command_queue_full_rejections;
static volatile uint32_t command_queue_enqueue_errors;
static volatile uint32_t command_queue_max_queued;

static void command_queue_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void command_queue_update_max_queued(uint32_t queued) {
    if (queued > command_queue_max_queued) {
        command_queue_max_queued = queued;
    }
}

osStatus_t command_queue_init(void) {
    if (command_queue_handle != NULL) {
        return osErrorResource;
    }

    command_queue_handle = osMessageQueueNew(COMMAND_QUEUE_CAPACITY, sizeof(CommandDTO_t), NULL);
    return command_queue_handle == NULL ? osErrorNoMemory : osOK;
}

CommandQueuePutResult_t command_queue_try_send(const CommandDTO_t *command) {
    if (command == NULL) {
        command_queue_counter_increment(&command_queue_enqueue_errors);
        return COMMAND_QUEUE_PUT_ERROR;
    }
    if (command_queue_handle == NULL) {
        command_queue_counter_increment(&command_queue_enqueue_errors);
        return COMMAND_QUEUE_PUT_ERROR;
    }

    const osStatus_t status = osMessageQueuePut(command_queue_handle, command, 0U, 0U);
    if (status == osOK) {
        command_queue_counter_increment(&command_queue_enqueued_total);
        command_queue_update_max_queued(osMessageQueueGetCount(command_queue_handle));
        return COMMAND_QUEUE_PUT_OK;
    }
    if (status == osErrorResource) {
        command_queue_counter_increment(&command_queue_full_rejections);
        return COMMAND_QUEUE_PUT_FULL;
    }
    command_queue_counter_increment(&command_queue_enqueue_errors);
    return COMMAND_QUEUE_PUT_ERROR;
}

osStatus_t command_queue_receive(CommandDTO_t *command, uint32_t timeout_ms) {
    if (command == NULL) {
        return osErrorParameter;
    }
    if (command_queue_handle == NULL) {
        return osErrorResource;
    }

    return osMessageQueueGet(command_queue_handle, command, NULL, timeout_ms);
}

void command_queue_get_diagnostics(CommandQueueDiagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->initialized = command_queue_handle != NULL ? 1U : 0U;
    diagnostics->capacity = COMMAND_QUEUE_CAPACITY;
    diagnostics->queued = command_queue_handle == NULL ? 0U : osMessageQueueGetCount(command_queue_handle);
    diagnostics->available = diagnostics->initialized != 0U ? COMMAND_QUEUE_CAPACITY - diagnostics->queued : 0U;
    diagnostics->max_queued = command_queue_max_queued;
    diagnostics->enqueued_total = command_queue_enqueued_total;
    diagnostics->full_rejections = command_queue_full_rejections;
    diagnostics->enqueue_errors = command_queue_enqueue_errors;
}

#ifdef FM_V3_ENABLE_TEST_HOOKS
void command_queue_test_seed_counters(uint32_t enqueued_total, uint32_t full_rejections, uint32_t enqueue_errors) {
    command_queue_enqueued_total = enqueued_total;
    command_queue_full_rejections = full_rejections;
    command_queue_enqueue_errors = enqueue_errors;
}
#endif

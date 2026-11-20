#include "command/command_result_queue.h"

#include <stddef.h>

static osMessageQueueId_t command_result_queue_handle;
/* Written once by the consumer at startup and read by the producer; a pointer store is atomic on Cortex-M7. */
static volatile CommandResultListener_t command_result_queue_listener;

osStatus_t command_result_queue_init(void) {
    if (command_result_queue_handle != NULL) {
        return osErrorResource;
    }

    command_result_queue_handle = osMessageQueueNew(COMMAND_RESULT_QUEUE_CAPACITY, sizeof(CommandResult_t), NULL);
    return command_result_queue_handle == NULL ? osErrorNoMemory : osOK;
}

static void command_result_queue_put(const CommandResult_t *result);

void command_result_queue_post_in_epoch(uint32_t epoch, uint32_t sequence, uint8_t response_type, uint16_t code) {
    const CommandResult_t result = {
        .sequence = sequence,
        .response_type = response_type,
        .code = code,
        .axis = COMMAND_ERROR_AXIS_NONE,
        .epoch = epoch,
    };
    command_result_queue_put(&result);
}

void command_result_queue_post_detailed_in_epoch(
    uint32_t epoch,
    uint32_t sequence,
    uint8_t response_type,
    uint16_t code,
    uint8_t cause,
    uint8_t axis,
    uint8_t detail
) {
    const CommandResult_t result = {
        .sequence = sequence,
        .response_type = response_type,
        .code = code,
        .cause = cause,
        .axis = axis,
        .detail = detail,
        .epoch = epoch,
    };
    command_result_queue_put(&result);
}

static void command_result_queue_put(const CommandResult_t *result) {
    if (command_result_queue_handle == NULL || osMessageQueuePut(command_result_queue_handle, result, 0U, 0U) != osOK) {
        return;
    }

    const CommandResultListener_t listener = command_result_queue_listener;
    if (listener != NULL) {
        listener();
    }
}

osStatus_t command_result_queue_try_receive(CommandResult_t *result) {
    if (result == NULL) {
        return osErrorParameter;
    }
    if (command_result_queue_handle == NULL) {
        return osErrorResource;
    }
    /* A result of an older epoch answers a command that a STOP has cancelled since: the client must not see it. */
    for (;;) {
        const osStatus_t status = osMessageQueueGet(command_result_queue_handle, result, NULL, 0U);
        if (status != osOK || result->epoch == command_queue_epoch()) {
            return status;
        }
    }
}

void command_result_queue_discard_all(void) {
    if (command_result_queue_handle != NULL) {
        (void)osMessageQueueReset(command_result_queue_handle);
    }
}

void command_result_queue_set_listener(CommandResultListener_t listener) { command_result_queue_listener = listener; }

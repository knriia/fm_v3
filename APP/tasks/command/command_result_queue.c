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

void command_result_queue_post(uint32_t sequence, uint8_t response_type, uint16_t code) {
    const CommandResult_t result = {.sequence = sequence, .response_type = response_type, .code = code};
    if (command_result_queue_handle == NULL ||
        osMessageQueuePut(command_result_queue_handle, &result, 0U, 0U) != osOK) {
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
    return osMessageQueueGet(command_result_queue_handle, result, NULL, 0U);
}

void command_result_queue_discard_all(void) {
    if (command_result_queue_handle != NULL) {
        (void)osMessageQueueReset(command_result_queue_handle);
    }
}

void command_result_queue_set_listener(CommandResultListener_t listener) { command_result_queue_listener = listener; }

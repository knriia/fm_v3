#ifndef FM_V3_TEST_QUEUE_H
#define FM_V3_TEST_QUEUE_H

#include "FreeRTOS.h"

QueueHandle_t xQueueCreateStatic(
    UBaseType_t queue_length,
    UBaseType_t item_size,
    uint8_t *queue_storage,
    StaticQueue_t *queue_buffer
);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t timeout);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout);

#endif /* FM_V3_TEST_QUEUE_H */
#ifndef FM_V3_TEST_CMSIS_OS_H
#define FM_V3_TEST_CMSIS_OS_H

#include <stdint.h>

typedef void *osEventFlagsId_t;
typedef void *osMessageQueueId_t;
typedef int32_t osStatus_t;

#define osPriorityError (-1)
#define osThreadError 0xFFFFFFFFU
#define osFlagsError 0x80000000U
#define osFlagsErrorParameter 0xFFFFFFFCU
#define osFlagsWaitAll 0x00000001U
#define osWaitForever 0xFFFFFFFFU
#define osOK ((osStatus_t)0)
#define osErrorResource ((osStatus_t)-3)
#define osErrorParameter ((osStatus_t)-4)
#define osErrorNoMemory ((osStatus_t)-5)

uint32_t osEventFlagsWait(osEventFlagsId_t event_flags_id, uint32_t flags, uint32_t options, uint32_t timeout);
uint32_t osEventFlagsSet(osEventFlagsId_t event_flags_id, uint32_t flags);
uint32_t osDelay(uint32_t milliseconds);
osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout);
uint32_t osMessageQueueGetCount(osMessageQueueId_t queue_id);

#endif /* FM_V3_TEST_CMSIS_OS_H */

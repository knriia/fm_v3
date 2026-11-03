#ifndef FM_V3_TEST_CMSIS_OS_H
#define FM_V3_TEST_CMSIS_OS_H

#include <stdint.h>

typedef void *osEventFlagsId_t;
typedef void *osMessageQueueId_t;
typedef void *osThreadId_t;
typedef void *osMutexId_t;
typedef int32_t osStatus_t;

typedef struct {
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
} osMutexAttr_t;

#define osMutexRecursive 0x00000001U
#define osMutexPrioInherit 0x00000002U

#define osPriorityError (-1)
#define osThreadError 0xFFFFFFFFU
#define osFlagsError 0x80000000U
#define osFlagsErrorTimeout 0xFFFFFFFEU
#define osFlagsErrorResource 0xFFFFFFFDU
#define osFlagsErrorParameter 0xFFFFFFFCU
#define osFlagsWaitAny 0x00000000U
#define osFlagsWaitAll 0x00000001U
#define osFlagsNoClear 0x00000002U
#define osWaitForever 0xFFFFFFFFU
#define osOK ((osStatus_t)0)
#define osErrorTimeout ((osStatus_t)-2)
#define osErrorResource ((osStatus_t)-3)
#define osErrorParameter ((osStatus_t)-4)
#define osErrorNoMemory ((osStatus_t)-5)

uint32_t osEventFlagsWait(osEventFlagsId_t event_flags_id, uint32_t flags, uint32_t options, uint32_t timeout);
uint32_t osEventFlagsSet(osEventFlagsId_t event_flags_id, uint32_t flags);
int32_t osKernelLock(void);
int32_t osKernelUnlock(void);
osThreadId_t osThreadGetId(void);
uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags);
uint32_t osThreadFlagsClear(uint32_t flags);
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout);
osMutexId_t osMutexNew(const osMutexAttr_t *attr);
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex_id);
uint32_t osDelay(uint32_t milliseconds);
osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout);
uint32_t osMessageQueueGetCount(osMessageQueueId_t queue_id);
uint32_t osMessageQueueGetSpace(osMessageQueueId_t queue_id);
osStatus_t osMessageQueueReset(osMessageQueueId_t queue_id);

#endif /* FM_V3_TEST_CMSIS_OS_H */

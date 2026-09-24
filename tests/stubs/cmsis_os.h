#ifndef FM_V3_TEST_CMSIS_OS_H
#define FM_V3_TEST_CMSIS_OS_H

#include <stdint.h>

typedef void *osEventFlagsId_t;
typedef uint32_t osStatus_t;

#define osPriorityError (-1)
#define osThreadError 0xFFFFFFFFU
#define osFlagsError 0x80000000U
#define osFlagsErrorParameter 0xFFFFFFFCU
#define osFlagsWaitAll 0x00000001U
#define osWaitForever 0xFFFFFFFFU

uint32_t osEventFlagsWait(osEventFlagsId_t event_flags_id, uint32_t flags, uint32_t options, uint32_t timeout);
uint32_t osDelay(uint32_t milliseconds);

#endif /* FM_V3_TEST_CMSIS_OS_H */

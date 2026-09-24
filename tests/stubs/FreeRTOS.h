#ifndef FM_V3_TEST_FREERTOS_H
#define FM_V3_TEST_FREERTOS_H

#include <stdint.h>

#define configTOTAL_HEAP_SIZE 32768U
#define pdFALSE 0
#define pdTRUE 1
#define eInvalid 0

typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef void *TaskHandle_t;

typedef struct {
    StackType_t *pxStackBase;
    UBaseType_t uxCurrentPriority;
    UBaseType_t uxBasePriority;
    uint32_t eCurrentState;
    uint32_t usStackHighWaterMark;
    uint32_t ulRunTimeCounter;
} TaskStatus_t;

uint32_t xPortGetFreeHeapSize(void);
uint32_t xPortGetMinimumEverFreeHeapSize(void);
UBaseType_t uxTaskGetNumberOfTasks(void);
UBaseType_t uxTaskGetSystemState(TaskStatus_t *task_statuses, UBaseType_t array_size, uint32_t *total_runtime_ticks);
uint32_t xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
TaskHandle_t xTaskGetIdleTaskHandle(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
TaskHandle_t xTaskGetHandle(const char *name);
void vTaskGetInfo(TaskHandle_t task_handle, TaskStatus_t *task_status, int get_free_stack, int state);

#endif /* FM_V3_TEST_FREERTOS_H */

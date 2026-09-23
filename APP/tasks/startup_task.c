#include "startup_task.h"
#include "network_events.h"

#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

void StartStartupTask(void *argument) {
    MX_LWIP_Init();
    osEventFlagsId_t *lwip_flags = argument;
    if ((lwip_flags == NULL) || (*lwip_flags == NULL)) {
        Error_Handler();
    }
    if (osEventFlagsSet(*lwip_flags, LWIP_READY_FLAG) & osFlagsError) {
        Error_Handler();
    }

    for (;;) {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        osDelay(500);
    }
}

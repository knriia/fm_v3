#include <stddef.h>

#include "network_events.h"
#include "startup_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

static volatile StartupTaskDiagnostics startup_task_diagnostics;

void startup_task_get_diagnostics(StartupTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->lwip_init_completed = startup_task_diagnostics.lwip_init_completed;
    diagnostics->lwip_ready_flag_set_attempts = startup_task_diagnostics.lwip_ready_flag_set_attempts;
    diagnostics->lwip_ready_flag_set_errors = startup_task_diagnostics.lwip_ready_flag_set_errors;
}

void StartStartupTask(void *argument) {
    MX_LWIP_Init();
    startup_task_diagnostics.lwip_init_completed = 1U;
    osEventFlagsId_t *lwip_flags = argument;
    if ((lwip_flags == NULL) || (*lwip_flags == NULL)) {
        Error_Handler();
    }
    ++startup_task_diagnostics.lwip_ready_flag_set_attempts;
    if (osEventFlagsSet(*lwip_flags, LWIP_READY_FLAG) & osFlagsError) {
        ++startup_task_diagnostics.lwip_ready_flag_set_errors;
        Error_Handler();
    }

    for (;;) {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        osDelay(500);
    }
}

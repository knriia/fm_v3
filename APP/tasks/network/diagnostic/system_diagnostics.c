#include "system_diagnostics.h"

#include "FreeRTOS.h"
#include "freertos_hooks.h"
#include "portable.h"
#include "stm32h723xx.h"
#include "task.h"

#include <stddef.h>

#define SYSTEM_DIAGNOSTICS_MAX_TASKS 32U

static uint32_t previous_total_runtime_ticks;
static uint32_t previous_idle_runtime_ticks;
static uint8_t runtime_sample_initialized;
static uint8_t runtime_sample_has_idle_task;

/* CPU load is the share of run time the idle task did not get since the previous snapshot. */
static uint32_t system_diagnostics_cpu_usage(uint32_t total_runtime_ticks) {
    TaskStatus_t idle_task_status = {0};
    const uint8_t has_idle_task = xTaskGetIdleTaskHandle() != NULL ? 1U : 0U;
    if (has_idle_task != 0U) {
        vTaskGetInfo(xTaskGetIdleTaskHandle(), &idle_task_status, pdFALSE, eInvalid);
    }
    const uint32_t idle_runtime_ticks = idle_task_status.ulRunTimeCounter;

    uint32_t cpu_usage_percent = 0U;
    if (runtime_sample_initialized != 0U && runtime_sample_has_idle_task != 0U && has_idle_task != 0U) {
        const uint32_t total_runtime_delta = total_runtime_ticks - previous_total_runtime_ticks;
        const uint32_t idle_runtime_delta = idle_runtime_ticks - previous_idle_runtime_ticks;
        if (total_runtime_delta != 0U) {
            const uint32_t busy_runtime_delta =
                idle_runtime_delta <= total_runtime_delta ? total_runtime_delta - idle_runtime_delta : 0U;
            cpu_usage_percent = (uint32_t)(((uint64_t)busy_runtime_delta * 100U) / total_runtime_delta);
        }
    }

    previous_total_runtime_ticks = total_runtime_ticks;
    previous_idle_runtime_ticks = idle_runtime_ticks;
    runtime_sample_has_idle_task = has_idle_task;
    runtime_sample_initialized = 1U;
    return cpu_usage_percent;
}

uint32_t system_diagnostics_collect(SystemDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return 0U;
    }

    /* FreeRTOS only reports the run-time total together with a snapshot of every task. */
    static TaskStatus_t task_statuses[SYSTEM_DIAGNOSTICS_MAX_TASKS];
    uint32_t total_runtime_ticks = 0U;
    (void)uxTaskGetSystemState(task_statuses, SYSTEM_DIAGNOSTICS_MAX_TASKS, &total_runtime_ticks);

    uint32_t stack_overflow_count = 0U;
    uint32_t malloc_failed_count = 0U;
    freertos_hooks_get_diagnostics(&stack_overflow_count, &malloc_failed_count);

    diagnostics->tick_count = (uint32_t)xTaskGetTickCount();
    diagnostics->reset_flags = RCC->RSR;
    diagnostics->heap_free_bytes = (uint32_t)xPortGetFreeHeapSize();
    diagnostics->heap_min_free_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    diagnostics->task_count = (uint32_t)uxTaskGetNumberOfTasks();
    diagnostics->cpu_usage_percent = system_diagnostics_cpu_usage(total_runtime_ticks);
    diagnostics->stack_overflow_count = stack_overflow_count;
    diagnostics->malloc_failed_count = malloc_failed_count;
    return total_runtime_ticks;
}

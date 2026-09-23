#include "system_diagnostics.h"

#include "FreeRTOS.h"
#include "freertos_hooks.h"
#include "portable.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include <stddef.h>

#define SYSTEM_DIAGNOSTICS_ITCM_BASE_ADDRESS 0x00000000U
#define SYSTEM_DIAGNOSTICS_ITCM_SIZE_BYTES 0x00010000U
#define SYSTEM_DIAGNOSTICS_DTCM_BASE_ADDRESS 0x20000000U
#define SYSTEM_DIAGNOSTICS_DTCM_SIZE_BYTES 0x00020000U
#define SYSTEM_DIAGNOSTICS_D1_BASE_ADDRESS 0x24000000U
#define SYSTEM_DIAGNOSTICS_D1_SIZE_BYTES 0x00050000U
#define SYSTEM_DIAGNOSTICS_D2_BASE_ADDRESS 0x30000000U
#define SYSTEM_DIAGNOSTICS_D2_SIZE_BYTES 0x00008000U
#define SYSTEM_DIAGNOSTICS_D3_BASE_ADDRESS 0x38000000U
#define SYSTEM_DIAGNOSTICS_D3_SIZE_BYTES 0x00004000U
#define SYSTEM_DIAGNOSTICS_MAX_TASKS 32U

extern uint8_t _end;
extern uint8_t _Min_Heap_Size;
extern uint8_t _Min_Stack_Size;
extern uint8_t __lwip_heap_end__;

static uint32_t previous_total_runtime_ticks;
static uint32_t previous_idle_runtime_ticks;
static uint8_t runtime_sample_initialized;
static uint8_t runtime_sample_has_idle_task;

static void system_diagnostics_fill_memory_region(
    MemoryRegionDiagnostics *diagnostics,
    uint32_t base_address,
    uint32_t total_bytes,
    uintptr_t used_end,
    uint32_t reserved_bytes
) {
    const uintptr_t region_end = (uintptr_t)base_address + total_bytes;
    const uintptr_t clamped_used_end = used_end < region_end ? used_end : region_end;
    const uint32_t used_bytes = clamped_used_end > base_address ? (uint32_t)(clamped_used_end - base_address) : 0U;
    const uint32_t clamped_reserved_bytes =
        reserved_bytes < (total_bytes - used_bytes) ? reserved_bytes : total_bytes - used_bytes;

    diagnostics->base_address = base_address;
    diagnostics->total_bytes = total_bytes;
    diagnostics->used_bytes = used_bytes;
    diagnostics->reserved_bytes = clamped_reserved_bytes;
    diagnostics->free_bytes = total_bytes - used_bytes - clamped_reserved_bytes;
}

static void system_diagnostics_collect_memory(MemoryDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    system_diagnostics_fill_memory_region(
        &diagnostics->itcm,
        SYSTEM_DIAGNOSTICS_ITCM_BASE_ADDRESS,
        SYSTEM_DIAGNOSTICS_ITCM_SIZE_BYTES,
        SYSTEM_DIAGNOSTICS_ITCM_BASE_ADDRESS,
        0U
    );
    system_diagnostics_fill_memory_region(
        &diagnostics->dtcm,
        SYSTEM_DIAGNOSTICS_DTCM_BASE_ADDRESS,
        SYSTEM_DIAGNOSTICS_DTCM_SIZE_BYTES,
        SYSTEM_DIAGNOSTICS_DTCM_BASE_ADDRESS,
        0U
    );
    system_diagnostics_fill_memory_region(
        &diagnostics->d1,
        SYSTEM_DIAGNOSTICS_D1_BASE_ADDRESS,
        SYSTEM_DIAGNOSTICS_D1_SIZE_BYTES,
        (uintptr_t)&_end,
        (uint32_t)((uintptr_t)&_Min_Heap_Size + (uintptr_t)&_Min_Stack_Size)
    );
    system_diagnostics_fill_memory_region(
        &diagnostics->d2,
        SYSTEM_DIAGNOSTICS_D2_BASE_ADDRESS,
        SYSTEM_DIAGNOSTICS_D2_SIZE_BYTES,
        (uintptr_t)&__lwip_heap_end__,
        0U
    );
    system_diagnostics_fill_memory_region(
        &diagnostics->d3,
        SYSTEM_DIAGNOSTICS_D3_BASE_ADDRESS,
        SYSTEM_DIAGNOSTICS_D3_SIZE_BYTES,
        SYSTEM_DIAGNOSTICS_D3_BASE_ADDRESS,
        0U
    );
}

static void system_diagnostics_collect_freertos(FreeRtosDiagnostics *diagnostics) {
    static TaskStatus_t task_statuses[SYSTEM_DIAGNOSTICS_MAX_TASKS];
    uint32_t total_runtime_ticks = 0U;
    const UBaseType_t task_count = uxTaskGetNumberOfTasks();
    const UBaseType_t snapshot_count =
        uxTaskGetSystemState(task_statuses, SYSTEM_DIAGNOSTICS_MAX_TASKS, &total_runtime_ticks);
    TaskStatus_t idle_task_status = {0};
    uint32_t stack_overflow_count = 0U;
    uint32_t malloc_failed_count = 0U;

    freertos_hooks_get_diagnostics(&stack_overflow_count, &malloc_failed_count);

    diagnostics->total_heap = configTOTAL_HEAP_SIZE;
    diagnostics->free_heap = (uint32_t)xPortGetFreeHeapSize();
    diagnostics->used_heap =
        diagnostics->total_heap > diagnostics->free_heap ? diagnostics->total_heap - diagnostics->free_heap : 0U;
    diagnostics->minimum_free_heap = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    diagnostics->task_count = task_count;
    diagnostics->task_snapshot_count = snapshot_count;
    diagnostics->task_snapshot_capacity = SYSTEM_DIAGNOSTICS_MAX_TASKS;
    diagnostics->task_snapshot_truncated = task_count > SYSTEM_DIAGNOSTICS_MAX_TASKS ? 1U : 0U;
    diagnostics->scheduler_state = (uint32_t)xTaskGetSchedulerState();
    diagnostics->tick_count = (uint32_t)xTaskGetTickCount();
    diagnostics->runtime_counter_hz = SystemCoreClock;
    diagnostics->total_runtime_ticks = total_runtime_ticks;
    diagnostics->idle_runtime_ticks = 0U;
    diagnostics->cpu_usage_percent = 0U;
    diagnostics->stack_overflow_count = stack_overflow_count;
    diagnostics->malloc_failed_count = malloc_failed_count;

    if (xTaskGetIdleTaskHandle() != NULL) {
        vTaskGetInfo(xTaskGetIdleTaskHandle(), &idle_task_status, pdFALSE, eInvalid);
        diagnostics->idle_runtime_ticks = idle_task_status.ulRunTimeCounter;
    }

    const uint8_t has_idle_task = xTaskGetIdleTaskHandle() != NULL ? 1U : 0U;
    if (runtime_sample_initialized && runtime_sample_has_idle_task != 0U && has_idle_task != 0U) {
        const uint32_t total_runtime_delta = total_runtime_ticks - previous_total_runtime_ticks;
        const uint32_t idle_runtime_delta = diagnostics->idle_runtime_ticks - previous_idle_runtime_ticks;

        if (total_runtime_delta != 0U) {
            const uint32_t busy_runtime_delta =
                idle_runtime_delta <= total_runtime_delta ? total_runtime_delta - idle_runtime_delta : 0U;
            diagnostics->cpu_usage_percent = (uint32_t)(((uint64_t)busy_runtime_delta * 100U) / total_runtime_delta);
        }
    }

    previous_total_runtime_ticks = total_runtime_ticks;
    previous_idle_runtime_ticks = diagnostics->idle_runtime_ticks;
    runtime_sample_has_idle_task = has_idle_task;
    runtime_sample_initialized = 1U;
}

void system_diagnostics_collect(SystemDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->free_heap = xPortGetFreeHeapSize();
    diagnostics->minimum_free_heap = xPortGetMinimumEverFreeHeapSize();
    diagnostics->uid0 = HAL_GetUIDw0();
    diagnostics->uid1 = HAL_GetUIDw1();
    diagnostics->uid2 = HAL_GetUIDw2();
    diagnostics->device_id = DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID;
    diagnostics->revision_id = (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID) >> DBGMCU_IDCODE_REV_ID_Pos;
    diagnostics->reset_flags = RCC->RSR;
    diagnostics->sys_clock_hz = HAL_RCC_GetSysClockFreq();
    diagnostics->hclk_hz = HAL_RCC_GetHCLKFreq();
    diagnostics->pclk1_hz = HAL_RCC_GetPCLK1Freq();
    diagnostics->pclk2_hz = HAL_RCC_GetPCLK2Freq();
    diagnostics->systick_ms = HAL_GetTick();
    diagnostics->scb_cfsr = SCB->CFSR;
    diagnostics->scb_hfsr = SCB->HFSR;
    diagnostics->scb_dfsr = SCB->DFSR;
    diagnostics->scb_mmfar = SCB->MMFAR;
    diagnostics->scb_bfar = SCB->BFAR;
    diagnostics->scb_afsr = SCB->AFSR;
    diagnostics->scb_shcsr = SCB->SHCSR;
    diagnostics->scb_ccr = SCB->CCR;
    diagnostics->scb_icsr = SCB->ICSR;
    diagnostics->scb_vtor = SCB->VTOR;
    diagnostics->mpu_ctrl = MPU->CTRL;
    diagnostics->dwt_ctrl = DWT->CTRL;
    diagnostics->dwt_cyccnt = DWT->CYCCNT;
    system_diagnostics_collect_memory(&diagnostics->memory);
    system_diagnostics_collect_freertos(&diagnostics->freertos);
    diagnostics->ipsr = __get_IPSR();
    diagnostics->control = __get_CONTROL();
    diagnostics->basepri = __get_BASEPRI();
    diagnostics->primask = __get_PRIMASK();
    diagnostics->faultmask = __get_FAULTMASK();
    diagnostics->msp = __get_MSP();
    diagnostics->psp = __get_PSP();
    diagnostics->systick_ctrl = SysTick->CTRL;
    diagnostics->systick_load = SysTick->LOAD;
    diagnostics->systick_val = SysTick->VAL;
    diagnostics->nvic_iser0 = NVIC->ISER[0];
    diagnostics->nvic_ispr0 = NVIC->ISPR[0];
    diagnostics->nvic_iabr0 = NVIC->IABR[0];
}

#include "system_diagnostics.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"

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
}

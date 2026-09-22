#include "mpu_config.h"

#include "stm32h7xx_hal.h"

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    const uint32_t ram_d2_base_address = 0x30000000U;
    const uint32_t ram_d2_mpu_size = MPU_REGION_SIZE_32KB;

    /* RAM_D2 contains ETH descriptors and the linker-reserved LwIP heap.
     * Keep it non-cacheable. LwIP pools and RX/TX pbufs remain in cacheable D1
     * and are handled with explicit cache maintenance in the ETH driver. */
    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = ram_d2_base_address;
    MPU_InitStruct.Size = ram_d2_mpu_size;
    MPU_InitStruct.SubRegionDisable = 0x00U;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

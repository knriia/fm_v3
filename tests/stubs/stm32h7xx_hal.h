#ifndef FM_V3_TEST_STM32H7XX_HAL_H
#define FM_V3_TEST_STM32H7XX_HAL_H

#include "stm32h723xx.h"

#include <stdint.h>

uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_GetTick(void);

extern uint32_t SystemCoreClock;

#endif /* FM_V3_TEST_STM32H7XX_HAL_H */

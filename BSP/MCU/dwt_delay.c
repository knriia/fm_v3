#include "dwt_delay.h"

#include "stm32h723xx.h"
#include "system_stm32h7xx.h"

#define DWT_DELAY_UNLOCK_KEY 0xC5ACCE55U

void dwt_delay_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = DWT_DELAY_UNLOCK_KEY;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void dwt_delay_ns(uint32_t ns) {
    const uint64_t cycles = ((uint64_t)ns * SystemCoreClock + 999999999ULL) / 1000000000ULL;
    const uint32_t start_cycles = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start_cycles) < (uint32_t)cycles) {
    }
}

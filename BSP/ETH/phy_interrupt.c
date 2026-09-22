#include "phy_interrupt.h"

#include "stm32h723xx.h"

void PHY_INT_GPIO_Init(void) {
    /* DP83848 PWR_DOWN/INT is an open-drain active-low interrupt output.
     * The board provides the pull-up, so PB0 remains without an internal pull. */
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;

    GPIOB->MODER &= ~(3UL << (0U * 2U));
    GPIOB->PUPDR &= ~(3UL << (0U * 2U));

    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PB;

    EXTI->RTSR1 &= ~EXTI_RTSR1_TR0;
    EXTI->FTSR1 |= EXTI_FTSR1_TR0;
    EXTI->PR1 = EXTI_PR1_PR0;
    EXTI->IMR1 |= EXTI_IMR1_IM0;

    NVIC_SetPriority(EXTI0_IRQn, 10U);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

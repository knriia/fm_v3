#include "encoder.h"

#include "stm32h723xx.h"

#include <stdint.h>

#define ENCODER_GPIO_MODE_ALTERNATE_FUNCTION 2U
#define ENCODER_GPIO_SPEED_HIGH 3U

#define ENCODER_TIM2_ALTERNATE_FUNCTION 1U
#define ENCODER_TIM23_ALTERNATE_FUNCTION 13U
#define ENCODER_TIM24_ALTERNATE_FUNCTION 14U

static void encoder_configure_gpio(GPIO_TypeDef *port, uint32_t pin, uint32_t alternate_function) {
    const uint32_t pin_shift = pin * 2U;
    const uint32_t alternate_shift = (pin % 8U) * 4U;
    const uint32_t alternate_register = pin / 8U;

    port->MODER = (port->MODER & ~(3UL << pin_shift)) | (ENCODER_GPIO_MODE_ALTERNATE_FUNCTION << pin_shift);
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << pin_shift)) | (ENCODER_GPIO_SPEED_HIGH << pin_shift);
    port->PUPDR &= ~(3UL << pin_shift);
    port->AFR[alternate_register] =
        (port->AFR[alternate_register] & ~(0xFUL << alternate_shift)) | (alternate_function << alternate_shift);
}

static void encoder_configure_timer(TIM_TypeDef *timer) {
    timer->CR1 = 0U;
    timer->CR2 = 0U;
    timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    timer->DIER = 0U;
    timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    timer->CCMR2 = 0U;
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->PSC = 0U;
    timer->ARR = UINT32_MAX;
    timer->CNT = 0U;
    timer->EGR = TIM_EGR_UG;
    timer->CR1 = TIM_CR1_CEN;
}

void encoder_init(void) {
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOFEN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
    RCC->APB1HENR |= RCC_APB1HENR_TIM23EN | RCC_APB1HENR_TIM24EN;
    __DSB();

    /* TIM2: OZ, CH1 = PA5, CH2 = PB3. PA1 is occupied by ETH_REF_CLK. */
    encoder_configure_gpio(GPIOA, 5U, ENCODER_TIM2_ALTERNATE_FUNCTION);
    encoder_configure_gpio(GPIOB, 3U, ENCODER_TIM2_ALTERNATE_FUNCTION);

    /* TIM23: OY, CH1 = PF0, CH2 = PF1. */
    encoder_configure_gpio(GPIOF, 0U, ENCODER_TIM23_ALTERNATE_FUNCTION);
    encoder_configure_gpio(GPIOF, 1U, ENCODER_TIM23_ALTERNATE_FUNCTION);

    /* TIM24: OX, CH1 = PF11, CH2 = PF12. */
    encoder_configure_gpio(GPIOF, 11U, ENCODER_TIM24_ALTERNATE_FUNCTION);
    encoder_configure_gpio(GPIOF, 12U, ENCODER_TIM24_ALTERNATE_FUNCTION);

    encoder_configure_timer(TIM2);
    encoder_configure_timer(TIM23);
    encoder_configure_timer(TIM24);
}

static void encoder_reset_timer(TIM_TypeDef *timer) {
    timer->CR1 &= ~TIM_CR1_CEN;
    timer->CNT = 0U;
    timer->CR1 |= TIM_CR1_CEN;
}

int32_t encoder_ox_get_coordinate(void) { return (int32_t)TIM24->CNT; }

void encoder_ox_reset_coordinate(void) { encoder_reset_timer(TIM24); }

int32_t encoder_oy_get_coordinate(void) { return (int32_t)TIM23->CNT; }

void encoder_oy_reset_coordinate(void) { encoder_reset_timer(TIM23); }

int32_t encoder_oz_get_coordinate(void) { return (int32_t)TIM2->CNT; }

void encoder_oz_reset_coordinate(void) { encoder_reset_timer(TIM2); }

#include "syringe_driver.h"

#include "dwt_delay.h"
#include "stm32h723xx.h"
#include "syringe_driver_logic.h"

#include <stddef.h>

#define SYRINGE_DRIVER_DIRECTION_PORT GPIOC
#define SYRINGE_DRIVER_DIRECTION_PIN 7U
#define SYRINGE_DRIVER_ENABLE_PORT GPIOG
#define SYRINGE_DRIVER_ENABLE_PIN 8U

#define SYRINGE_DRIVER_GPIO_MODE_OUTPUT 1UL
/* DIR and ENA hold a level for a whole stream, so the slowest edge is enough and radiates the least. */
#define SYRINGE_DRIVER_GPIO_SPEED_LOW 0UL

/* Written once by syringe_driver_init before the scheduler starts, then read by the generator task. */
static SyringeDriverConfig_t syringe_driver_config;
static volatile uint8_t syringe_driver_ready;

static void syringe_driver_write_pin(GPIO_TypeDef *port, uint8_t pin, uint8_t high) {
    const uint32_t mask = 1UL << pin;
    port->BSRR = high != 0U ? mask : mask << 16U;
}

static void syringe_driver_configure_output(GPIO_TypeDef *port, uint8_t pin, uint8_t high) {
    const uint32_t shift = (uint32_t)pin * 2U;
    syringe_driver_write_pin(port, pin, high);
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << shift)) | (SYRINGE_DRIVER_GPIO_SPEED_LOW << shift);
    port->PUPDR &= ~(3UL << shift);
    port->MODER = (port->MODER & ~(3UL << shift)) | (SYRINGE_DRIVER_GPIO_MODE_OUTPUT << shift);
}

static void syringe_driver_set_enabled(uint8_t enabled) {
    syringe_driver_write_pin(
        SYRINGE_DRIVER_ENABLE_PORT,
        SYRINGE_DRIVER_ENABLE_PIN,
        syringe_driver_logic_enable_level(enabled, syringe_driver_config.enable_active_high)
    );
}

uint8_t syringe_driver_init(const SyringeDriverConfig_t *config) {
    if (config == NULL) {
        return 0U;
    }
    syringe_driver_config = *config;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIOGEN;
    /* Read back so the clock is on before the first register access. */
    (void)RCC->AHB4ENR;
    dwt_delay_init();
    syringe_driver_configure_output(
        SYRINGE_DRIVER_DIRECTION_PORT,
        SYRINGE_DRIVER_DIRECTION_PIN,
        syringe_driver_logic_direction_level(1U, config->positive_direction_high)
    );
    syringe_driver_configure_output(
        SYRINGE_DRIVER_ENABLE_PORT,
        SYRINGE_DRIVER_ENABLE_PIN,
        syringe_driver_logic_enable_level(0U, config->enable_active_high)
    );
    syringe_driver_ready = 1U;
    return 1U;
}

uint8_t syringe_driver_set_stream_outputs(uint8_t positive_direction) {
    if (syringe_driver_ready == 0U) {
        return 0U;
    }
    syringe_driver_set_enabled(1U);
    dwt_delay_ns(syringe_driver_config.enable_setup_ns);
    syringe_driver_write_pin(
        SYRINGE_DRIVER_DIRECTION_PORT,
        SYRINGE_DRIVER_DIRECTION_PIN,
        syringe_driver_logic_direction_level(positive_direction, syringe_driver_config.positive_direction_high)
    );
    return 1U;
}

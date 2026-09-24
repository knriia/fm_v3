#ifndef FM_V3_TEST_MAIN_H
#define FM_V3_TEST_MAIN_H

#include <stdint.h>

#define LED_STATUS_GPIO_Port ((void *)0x1U)
#define LED_STATUS_Pin 13U

void HAL_GPIO_TogglePin(void *gpio_port, uint16_t gpio_pin);
void Error_Handler(void);

#endif /* FM_V3_TEST_MAIN_H */

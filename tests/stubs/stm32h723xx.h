#ifndef FM_V3_TEST_STM32H723XX_H
#define FM_V3_TEST_STM32H723XX_H

#include <stdint.h>

typedef struct {
    uint32_t RSR;
} RCC_TypeDef;

extern RCC_TypeDef test_rcc;

#define RCC (&test_rcc)

#endif /* FM_V3_TEST_STM32H723XX_H */

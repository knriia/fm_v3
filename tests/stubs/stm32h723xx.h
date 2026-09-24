#ifndef FM_V3_TEST_STM32H723XX_H
#define FM_V3_TEST_STM32H723XX_H

#include <stdint.h>

typedef struct {
    uint32_t IDCODE;
} DBGMCU_TypeDef;

typedef struct {
    uint32_t RSR;
} RCC_TypeDef;

typedef struct {
    uint32_t CFSR;
    uint32_t HFSR;
    uint32_t DFSR;
    uint32_t MMFAR;
    uint32_t BFAR;
    uint32_t AFSR;
    uint32_t SHCSR;
    uint32_t CCR;
    uint32_t ICSR;
    uint32_t VTOR;
} SCB_Type;

typedef struct {
    uint32_t CTRL;
} MPU_Type;

typedef struct {
    uint32_t CTRL;
    uint32_t CYCCNT;
} DWT_Type;

typedef struct {
    uint32_t CTRL;
    uint32_t LOAD;
    uint32_t VAL;
} SysTick_Type;

typedef struct {
    uint32_t ISER[1];
    uint32_t ISPR[1];
    uint32_t IABR[1];
} NVIC_Type;

extern DBGMCU_TypeDef test_dbgmcu;
extern RCC_TypeDef test_rcc;
extern SCB_Type test_scb;
extern MPU_Type test_mpu;
extern DWT_Type test_dwt;
extern SysTick_Type test_systick;
extern NVIC_Type test_nvic;

#define DBGMCU (&test_dbgmcu)
#define RCC (&test_rcc)
#define SCB (&test_scb)
#define MPU (&test_mpu)
#define DWT (&test_dwt)
#define SysTick (&test_systick)
#define NVIC (&test_nvic)

#define DBGMCU_IDCODE_DEV_ID 0x0FFFU
#define DBGMCU_IDCODE_REV_ID 0xFFFF0000U
#define DBGMCU_IDCODE_REV_ID_Pos 16U

uint32_t __get_IPSR(void);
uint32_t __get_CONTROL(void);
uint32_t __get_BASEPRI(void);
uint32_t __get_PRIMASK(void);
uint32_t __get_FAULTMASK(void);
uint32_t __get_MSP(void);
uint32_t __get_PSP(void);

#endif /* FM_V3_TEST_STM32H723XX_H */

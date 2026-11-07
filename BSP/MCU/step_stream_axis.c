#include "step_stream_axis.h"

#include "motion_axis_hardware.h"
#include "step_stream_format.h"

#include "stm32h723xx.h"
#include "system_stm32h7xx.h"

#include <stdint.h>

#define STEP_STREAM_AXIS_TIMER_IRQ_PRIORITY 5U
#define STEP_STREAM_AXIS_TIMER_APB1_ENABLE RCC_APB1LENR_TIM4EN
#define STEP_STREAM_AXIS_GPIOB_ENABLE RCC_AHB4ENR_GPIOBEN
#define STEP_STREAM_AXIS_DMA1_ENABLE RCC_AHB1ENR_DMA1EN
#define STEP_STREAM_AXIS_DMA_REQUEST_TIM4_UP 32U
#define STEP_STREAM_AXIS_DMA_STREAM DMA1_Stream0
#define STEP_STREAM_AXIS_DMA_CHANNEL DMAMUX1_Channel0
#define STEP_STREAM_AXIS_STEP_PIN 6U
#define STEP_STREAM_AXIS_STEP_PIN_MASK (1UL << STEP_STREAM_AXIS_STEP_PIN)
#define STEP_STREAM_AXIS_TIMER_DBA_ARR 11U
#define STEP_STREAM_AXIS_TIMER_DBL 2U
#define STEP_STREAM_AXIS_RECORD_COUNT 2730U
#define STEP_STREAM_AXIS_HALF_RECORD_COUNT (STEP_STREAM_AXIS_RECORD_COUNT / 2U)
#define STEP_STREAM_AXIS_BUFFER_WORD_COUNT (STEP_STREAM_AXIS_RECORD_COUNT * STEP_STREAM_FORMAT_DMA_BURST_LENGTH)
#define STEP_STREAM_AXIS_NANOSECONDS_PER_SECOND 1000000000ULL
#define STEP_STREAM_AXIS_MAX_PERIOD_TICKS 65536U

static uint16_t step_stream_axis_dma_buffer[STEP_STREAM_AXIS_BUFFER_WORD_COUNT]
    __attribute__((section(".stepper_dma_buffers"), aligned(64)));

static volatile StepStreamAxisState_t step_stream_axis_state = STEP_STREAM_AXIS_STATE_IDLE;
static volatile uint32_t step_stream_axis_completed_half_buffers;
static volatile uint32_t step_stream_axis_last_steps_done;

extern uint32_t SystemCoreClock;

static uint32_t step_stream_axis_apb_divider(uint32_t prescaler) {
    switch (prescaler) {
    case 0U:
    case 1U:
    case 2U:
    case 3U:
        return 1U;
    case 4U:
        return 2U;
    case 5U:
        return 4U;
    case 6U:
        return 8U;
    case 7U:
        return 16U;
    default:
        return 0U;
    }
}

static uint32_t step_stream_axis_ahb_divider(void) {
    static const uint16_t ahb_dividers[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U, 2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
    const uint32_t prescaler = (RCC->D1CFGR & RCC_D1CFGR_HPRE_Msk) >> RCC_D1CFGR_HPRE_Pos;
    return ahb_dividers[prescaler];
}

/* Keep this calculation identical to motion_axis_hardware.c for the TIM4 kernel clock. */
static uint32_t step_stream_axis_timer_clock_hz(void) {
    const uint32_t prescaler = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_Msk) >> RCC_D2CFGR_D2PPRE1_Pos;
    const uint32_t ahb_divider = step_stream_axis_ahb_divider();
    const uint32_t apb_divider = step_stream_axis_apb_divider(prescaler);
    if (SystemCoreClock == 0U || ahb_divider == 0U || apb_divider == 0U) {
        return 0U;
    }

    const uint32_t hclk = SystemCoreClock / ahb_divider;
    const uint32_t pclk = hclk / apb_divider;
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U) {
        return apb_divider <= 4U ? hclk : pclk * 4U;
    }
    return apb_divider == 1U ? pclk : pclk * 2U;
}

static uint64_t step_stream_axis_ceil_div_u64(uint64_t numerator, uint64_t denominator) {
    return numerator / denominator + (numerator % denominator != 0U ? 1U : 0U);
}

static StepStreamAxisStatus_t step_stream_axis_calculate_ticks(
    uint32_t frequency_hz,
    uint32_t pulse_width_ns,
    uint32_t timer_clock_hz,
    uint32_t *period_ticks,
    uint32_t *pulse_ticks
) {
    if (frequency_hz == 0U || pulse_width_ns == 0U || timer_clock_hz == 0U || period_ticks == NULL ||
        pulse_ticks == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }

    const uint64_t period_ns = step_stream_axis_ceil_div_u64(STEP_STREAM_AXIS_NANOSECONDS_PER_SECOND, frequency_hz);
    const uint64_t period =
        step_stream_axis_ceil_div_u64(period_ns * timer_clock_hz, STEP_STREAM_AXIS_NANOSECONDS_PER_SECOND);
    const uint64_t pulse = step_stream_axis_ceil_div_u64(
        (uint64_t)pulse_width_ns * timer_clock_hz,
        STEP_STREAM_AXIS_NANOSECONDS_PER_SECOND
    );
    if (period == 0U || period > STEP_STREAM_AXIS_MAX_PERIOD_TICKS || pulse >= period) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }

    *period_ticks = (uint32_t)period;
    *pulse_ticks = (uint32_t)pulse;
    return STEP_STREAM_AXIS_STATUS_OK;
}

static uint32_t step_stream_axis_enter_critical(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void step_stream_axis_exit_critical(uint32_t primask) {
    __DMB();
    __set_PRIMASK(primask);
}

static uint32_t step_stream_axis_pending_half_buffers(void) {
    const uint32_t flags = DMA1->LISR;
    return ((flags & DMA_LISR_HTIF0) != 0U ? 1U : 0U) + ((flags & DMA_LISR_TCIF0) != 0U ? 1U : 0U);
}

static void step_stream_axis_clear_dma_flags(void) {
    DMA1->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
}

static void step_stream_axis_configure_gpio(void) {
    RCC->AHB4ENR |= STEP_STREAM_AXIS_GPIOB_ENABLE;
    (void)RCC->AHB4ENR;
    __DSB();

    GPIOB->MODER =
        (GPIOB->MODER & ~(3UL << (STEP_STREAM_AXIS_STEP_PIN * 2U))) | (2UL << (STEP_STREAM_AXIS_STEP_PIN * 2U));
    GPIOB->OTYPER &= ~STEP_STREAM_AXIS_STEP_PIN_MASK;
    GPIOB->OSPEEDR =
        (GPIOB->OSPEEDR & ~(3UL << (STEP_STREAM_AXIS_STEP_PIN * 2U))) | (3UL << (STEP_STREAM_AXIS_STEP_PIN * 2U));
    GPIOB->PUPDR &= ~(3UL << (STEP_STREAM_AXIS_STEP_PIN * 2U));
    GPIOB->AFR[0] =
        (GPIOB->AFR[0] & ~(0xFUL << (STEP_STREAM_AXIS_STEP_PIN * 4U))) | (2UL << (STEP_STREAM_AXIS_STEP_PIN * 4U));
}

static void step_stream_axis_configure_timer(uint32_t period_ticks, uint32_t pulse_ticks) {
    TIM4->CR1 = TIM_CR1_ARPE;
    TIM4->CR2 = 0U;
    TIM4->SMCR = 0U;
    TIM4->DIER = 0U;
    TIM4->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM4->CCER = 0U;
    TIM4->PSC = 0U;
    TIM4->DCR = (STEP_STREAM_AXIS_TIMER_DBA_ARR << TIM_DCR_DBA_Pos) | (STEP_STREAM_AXIS_TIMER_DBL << TIM_DCR_DBL_Pos);
    TIM4->CNT = 0U;
    TIM4->ARR = (uint16_t)(period_ticks - 1U);
    TIM4->CCR1 = (uint16_t)pulse_ticks;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
}

static void step_stream_axis_configure_dma(void) {
    RCC->AHB1ENR |= STEP_STREAM_AXIS_DMA1_ENABLE;
    (void)RCC->AHB1ENR;
    __DSB();

    STEP_STREAM_AXIS_DMA_CHANNEL->CCR = STEP_STREAM_AXIS_DMA_REQUEST_TIM4_UP;
    STEP_STREAM_AXIS_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while ((STEP_STREAM_AXIS_DMA_STREAM->CR & DMA_SxCR_EN) != 0U) {
    }

    step_stream_axis_clear_dma_flags();
    STEP_STREAM_AXIS_DMA_STREAM->PAR = (uint32_t)&TIM4->DMAR;
    STEP_STREAM_AXIS_DMA_STREAM->M0AR = (uint32_t)step_stream_axis_dma_buffer;
    STEP_STREAM_AXIS_DMA_STREAM->NDTR = STEP_STREAM_AXIS_BUFFER_WORD_COUNT;
    STEP_STREAM_AXIS_DMA_STREAM->FCR = 0U;
    STEP_STREAM_AXIS_DMA_STREAM->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC |
                                      DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
    NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream0_IRQn, STEP_STREAM_AXIS_TIMER_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    STEP_STREAM_AXIS_DMA_STREAM->CR |= DMA_SxCR_EN;
}

static uint32_t step_stream_axis_current_steps(void) {
    const uint32_t primask = step_stream_axis_enter_critical();
    const uint32_t completed_half_buffers =
        step_stream_axis_completed_half_buffers + step_stream_axis_pending_half_buffers();
    const uint32_t ndtr = STEP_STREAM_AXIS_DMA_STREAM->NDTR;
    const uint32_t steps = step_stream_format_steps_done(completed_half_buffers, STEP_STREAM_AXIS_RECORD_COUNT, ndtr);
    step_stream_axis_exit_critical(primask);
    return steps;
}

StepStreamAxisStatus_t step_stream_axis_start(uint32_t frequency_hz, uint32_t pulse_width_ns) {
    if (step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    if (frequency_hz == 0U || pulse_width_ns == 0U) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t timer_clock_hz = step_stream_axis_timer_clock_hz();
    uint32_t period_ticks = 0U;
    uint32_t pulse_ticks = 0U;
    const StepStreamAxisStatus_t tick_status =
        step_stream_axis_calculate_ticks(frequency_hz, pulse_width_ns, timer_clock_hz, &period_ticks, &pulse_ticks);
    if (tick_status != STEP_STREAM_AXIS_STATUS_OK) {
        return timer_clock_hz == 0U ? STEP_STREAM_AXIS_STATUS_CLOCK_ERROR : tick_status;
    }

    if (step_stream_format_fill(
            step_stream_axis_dma_buffer,
            STEP_STREAM_AXIS_RECORD_COUNT,
            period_ticks,
            pulse_ticks
        ) != STEP_STREAM_FORMAT_STATUS_OK) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t primask = step_stream_axis_enter_critical();
    if (step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }

    const MotionAxisHardwareStatus_t motion_status = motion_axis_hardware_get_status(MOTION_AXIS_X);
    if (motion_status == MOTION_AXIS_HARDWARE_STATUS_STARTING || motion_status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
        motion_status == MOTION_AXIS_HARDWARE_STATUS_RUNNING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_AXIS_BUSY;
    }

    RCC->APB1LENR |= STEP_STREAM_AXIS_TIMER_APB1_ENABLE;
    (void)RCC->APB1LENR;
    __DSB();
    step_stream_axis_configure_gpio();
    step_stream_axis_configure_timer(period_ticks, pulse_ticks);
    step_stream_axis_completed_half_buffers = 0U;
    step_stream_axis_last_steps_done = 0U;
    step_stream_axis_configure_dma();
    TIM4->DIER = TIM_DIER_UDE;
    step_stream_axis_state = STEP_STREAM_AXIS_STATE_STREAMING;
    __DMB();
    /* Enable the output only now: with CNT below CCR1 it would drive STEP high before the first period starts. */
    TIM4->CCER = TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN;
    step_stream_axis_exit_critical(primask);
    return STEP_STREAM_AXIS_STATUS_OK;
}

void step_stream_axis_stop(void) {
    const uint32_t primask = step_stream_axis_enter_critical();
    if (step_stream_axis_state != STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return;
    }

    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~TIM_DIER_UDE;
    STEP_STREAM_AXIS_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while ((STEP_STREAM_AXIS_DMA_STREAM->CR & DMA_SxCR_EN) != 0U) {
    }

    __DSB();
    const uint32_t pending_half_buffers = step_stream_axis_pending_half_buffers();
    const uint32_t completed_half_buffers = step_stream_axis_completed_half_buffers + pending_half_buffers;
    const uint32_t ndtr = STEP_STREAM_AXIS_DMA_STREAM->NDTR;
    step_stream_axis_last_steps_done =
        step_stream_format_steps_done(completed_half_buffers, STEP_STREAM_AXIS_RECORD_COUNT, ndtr);
    STEP_STREAM_AXIS_DMA_STREAM->CR &= ~(DMA_SxCR_HTIE | DMA_SxCR_TCIE);
    step_stream_axis_clear_dma_flags();
    NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    TIM4->CCER &= ~TIM_CCER_CC1E;
    TIM4->SR = 0U;
    step_stream_axis_state = STEP_STREAM_AXIS_STATE_IDLE;
    step_stream_axis_exit_critical(primask);
}

uint32_t step_stream_axis_steps_done(void) {
    if (step_stream_axis_state != STEP_STREAM_AXIS_STATE_STREAMING) {
        return step_stream_axis_last_steps_done;
    }
    return step_stream_axis_current_steps();
}

StepStreamAxisState_t step_stream_axis_get_state(void) { return step_stream_axis_state; }

void DMA1_Stream0_IRQHandler(void) {
    const uint32_t status = DMA1->LISR;
    uint32_t clear_flags = 0U;
    if ((status & DMA_LISR_HTIF0) != 0U) {
        ++step_stream_axis_completed_half_buffers;
        clear_flags |= DMA_LIFCR_CHTIF0;
    }
    if ((status & DMA_LISR_TCIF0) != 0U) {
        ++step_stream_axis_completed_half_buffers;
        clear_flags |= DMA_LIFCR_CTCIF0;
    }
    if (clear_flags != 0U) {
        DMA1->LIFCR = clear_flags;
    }
}

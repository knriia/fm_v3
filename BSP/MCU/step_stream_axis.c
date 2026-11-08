#include "step_stream_axis.h"

#include "motion/step_stream_brake.h"
#include "motion/step_stream_ring.h"
#include "motion_axis_hardware.h"
#include "step_stream_axis_logic.h"
#include "step_stream_format.h"

#include "stm32h723xx.h"
#include "system_stm32h7xx.h"

#include <stddef.h>
#include <stdint.h>

/* Must not be numerically below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5): the interrupt calls FreeRTOS. */
#define STEP_STREAM_AXIS_DMA_IRQ_PRIORITY 5U
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
#define STEP_STREAM_AXIS_BUFFER_WORD_COUNT (STEP_STREAM_AXIS_RECORD_COUNT * STEP_STREAM_FORMAT_DMA_BURST_LENGTH)
#define STEP_STREAM_AXIS_NANOSECONDS_PER_SECOND 1000000000ULL
#define STEP_STREAM_AXIS_MAX_PERIOD_TICKS 65536U
/* DMA disable takes a few AHB cycles; the bound only keeps a broken DMA from hanging the caller (or the ISR). */
#define STEP_STREAM_AXIS_DMA_DISABLE_MAX_POLLS 10000U
/* Entries of the braking ramp: (50000^2 - 4197^2) / (2 * deceleration), about 2070 at 600000 steps/s^2. 4096 is
 * enough down to a deceleration of about 305000; a gentler one is refused and the axis then stops at once. */
#define STEP_STREAM_AXIS_BRAKE_CAPACITY 4096U
/* The ramp starts this many records after the DMA position: the interrupt must finish the first writes before the DMA
 * arrives (8 records are 160 us at 50 kHz). Not measured on the hardware yet: check it with a GPIO and the scope. */
#define STEP_STREAM_AXIS_BRAKE_MARGIN_RECORDS 8U
/* A half event that is still unhandled when the braking tries to begin is handled by the retry; this bounds it. */
#define STEP_STREAM_AXIS_BRAKE_MAX_RETRIES 4U

static uint16_t step_stream_axis_dma_buffer[STEP_STREAM_AXIS_BUFFER_WORD_COUNT]
    __attribute__((section(".stepper_dma_buffers"), aligned(64)));

static volatile StepStreamAxisState_t step_stream_axis_state = STEP_STREAM_AXIS_STATE_IDLE;
static volatile uint32_t step_stream_axis_completed_half_buffers;
static volatile uint32_t step_stream_axis_last_steps_done;
static volatile uint32_t step_stream_axis_last_stale_steps;
static StepStreamRing_t step_stream_axis_ring;
static volatile StepStreamAxisFault_t step_stream_axis_fault = STEP_STREAM_AXIS_FAULT_NONE;
static volatile uint8_t step_stream_axis_auto_refill;
static void (*volatile step_stream_axis_on_half_released)(void);
static void (*volatile step_stream_axis_on_fault)(StepStreamAxisFault_t fault);
static void (*volatile step_stream_axis_on_brake)(void);

static uint16_t step_stream_axis_brake_storage[STEP_STREAM_AXIS_BRAKE_CAPACITY];
static StepStreamBrake_t step_stream_axis_brake_table;
static volatile uint8_t step_stream_axis_brake_ready;
/* Written by the DMA interrupt and, under a critical section, by the task. Read through
 * step_stream_axis_brake_active() wherever the other side may change it. */
static StepStreamAxisBrake_t step_stream_axis_brake;
/* Set by step_stream_axis_request_brake from any interrupt. Cleared by the DMA interrupt, and by launch and prepare
 * while the axis is IDLE (no interrupt of the stream runs then). A lost update is harmless: a request is either already
 * handled or pends the DMA interrupt once more. */
static volatile uint8_t step_stream_axis_brake_requested;
static uint8_t step_stream_axis_brake_attempt;
/* The EMPTY half that the task acquired and has not committed yet, or NO_HALF. */
static volatile uint8_t step_stream_axis_held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
static volatile StepStreamAxisBrakeStats_t step_stream_axis_brake_stats;

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

static uint32_t step_stream_axis_enter_critical(void);

static uint8_t step_stream_axis_brake_active(void) { return *(const volatile uint8_t *)&step_stream_axis_brake.active; }

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

static uint32_t step_stream_axis_dma_ndtr(void) { return STEP_STREAM_AXIS_DMA_STREAM->NDTR; }

static uint8_t step_stream_axis_dma_disabled(void) {
    return (STEP_STREAM_AXIS_DMA_STREAM->CR & DMA_SxCR_EN) == 0U ? 1U : 0U;
}

static const StepStreamAxisSnapshotSource_t step_stream_axis_snapshot_source = {
    step_stream_axis_pending_half_buffers,
    step_stream_axis_dma_ndtr,
};

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

/* Returns 0 when the stream did not disable; the DMA registers are then left untouched. */
static uint8_t step_stream_axis_configure_dma(void) {
    RCC->AHB1ENR |= STEP_STREAM_AXIS_DMA1_ENABLE;
    (void)RCC->AHB1ENR;
    __DSB();

    STEP_STREAM_AXIS_DMA_CHANNEL->CCR = STEP_STREAM_AXIS_DMA_REQUEST_TIM4_UP;
    STEP_STREAM_AXIS_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    if (step_stream_axis_logic_poll(step_stream_axis_dma_disabled, STEP_STREAM_AXIS_DMA_DISABLE_MAX_POLLS) == 0U) {
        return 0U;
    }

    step_stream_axis_clear_dma_flags();
    STEP_STREAM_AXIS_DMA_STREAM->PAR = (uint32_t)&TIM4->DMAR;
    STEP_STREAM_AXIS_DMA_STREAM->M0AR = (uint32_t)step_stream_axis_dma_buffer;
    STEP_STREAM_AXIS_DMA_STREAM->NDTR = STEP_STREAM_AXIS_BUFFER_WORD_COUNT;
    STEP_STREAM_AXIS_DMA_STREAM->FCR = 0U;
    STEP_STREAM_AXIS_DMA_STREAM->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC |
                                      DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
    NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream0_IRQn, STEP_STREAM_AXIS_DMA_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    STEP_STREAM_AXIS_DMA_STREAM->CR |= DMA_SxCR_EN;
    return 1U;
}

static uint32_t step_stream_axis_current_steps(void) {
    const uint32_t primask = step_stream_axis_enter_critical();
    const uint32_t steps = step_stream_axis_logic_read_steps(
        &step_stream_axis_snapshot_source,
        step_stream_axis_completed_half_buffers,
        STEP_STREAM_AXIS_RECORD_COUNT
    );
    step_stream_axis_exit_critical(primask);
    return steps;
}

/* Common start: the ring is already prepared (stream) or prepared here (stand). The timer idles at period_ticks with
 * the first pulse at pulse_ticks (0 = none) until the DMA delivers the first record. */
static StepStreamAxisStatus_t step_stream_axis_launch(uint32_t period_ticks, uint32_t pulse_ticks, uint8_t stand) {
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

    /* Decide on the ring before any register is touched, so a refusal leaves the hardware as it was. */
    if (stand != 0U) {
        if (step_stream_axis_logic_prepare_stand_ring(&step_stream_axis_ring) != 0U) {
            step_stream_axis_exit_critical(primask);
            return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
        }
    } else {
        const StepStreamRingStatus_t ring_status = step_stream_ring_start(&step_stream_axis_ring);
        if (ring_status != STEP_STREAM_RING_STATUS_OK) {
            step_stream_axis_exit_critical(primask);
            return ring_status == STEP_STREAM_RING_STATUS_NOT_READY ? STEP_STREAM_AXIS_STATUS_NOT_READY
                                                                    : STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
        }
    }

    RCC->APB1LENR |= STEP_STREAM_AXIS_TIMER_APB1_ENABLE;
    (void)RCC->APB1LENR;
    __DSB();
    step_stream_axis_configure_gpio();
    step_stream_axis_configure_timer(period_ticks, pulse_ticks);
    step_stream_axis_completed_half_buffers = 0U;
    step_stream_axis_last_steps_done = 0U;
    step_stream_axis_last_stale_steps = 0U;
    step_stream_axis_fault = STEP_STREAM_AXIS_FAULT_NONE;
    step_stream_axis_auto_refill = stand;
    step_stream_axis_logic_brake_reset(&step_stream_axis_brake);
    step_stream_axis_brake_requested = 0U;
    step_stream_axis_brake_attempt = 0U;
    step_stream_axis_held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    if (step_stream_axis_configure_dma() == 0U) {
        /* The ring was started above; return it to the start so that the next attempt begins clean. */
        step_stream_ring_init(&step_stream_axis_ring);
        step_stream_axis_fault = STEP_STREAM_AXIS_FAULT_DMA;
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_DMA_ERROR;
    }
    TIM4->DIER = TIM_DIER_UDE;
    step_stream_axis_state = STEP_STREAM_AXIS_STATE_STREAMING;
    __DMB();
    /* Enable the output only now: with CNT below CCR1 it would drive STEP high before the first period starts. */
    TIM4->CCER = TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN;
    step_stream_axis_exit_critical(primask);
    if (stand == 0U) {
        /* A switch edge that came between the direction output and this start was seen by nobody: look at the level. */
        motion_axis_hardware_poll_stream_limit();
    }
    return STEP_STREAM_AXIS_STATUS_OK;
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
    return step_stream_axis_launch(period_ticks, pulse_ticks, 1U);
}

uint32_t step_stream_axis_get_timer_clock_hz(void) { return step_stream_axis_timer_clock_hz(); }

StepStreamAxisStatus_t step_stream_axis_prepare_stream(void) {
    const uint32_t primask = step_stream_axis_enter_critical();
    if (step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    /* The DMA is stopped and the interrupt ignores events while IDLE, so nobody else touches the ring. */
    step_stream_ring_init(&step_stream_axis_ring);
    step_stream_axis_logic_brake_reset(&step_stream_axis_brake);
    step_stream_axis_brake_requested = 0U;
    step_stream_axis_held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    step_stream_axis_fault = STEP_STREAM_AXIS_FAULT_NONE;
    step_stream_axis_last_steps_done = 0U;
    step_stream_axis_last_stale_steps = 0U;
    step_stream_axis_exit_critical(primask);
    return STEP_STREAM_AXIS_STATUS_OK;
}

/* An IDLE axis whose ring still holds an earlier run must be prepared before the task touches the halves. */
static uint8_t step_stream_axis_ring_needs_prepare(void) {
    return (step_stream_axis_state == STEP_STREAM_AXIS_STATE_IDLE &&
               step_stream_ring_get_state(&step_stream_axis_ring) != STEP_STREAM_RING_STATE_NOT_STARTED)
               ? 1U
               : 0U;
}

StepStreamAxisStatus_t step_stream_axis_acquire_half(uint8_t *half, uint16_t **records) {
    if (half == NULL || records == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    if (step_stream_axis_ring_needs_prepare() != 0U) {
        return STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
    }
    /* The braking must see the held half and the half itself in one piece, so it cannot start in between. */
    const uint32_t primask = step_stream_axis_enter_critical();
    uint8_t candidate = 0U;
    /* While braking, the interrupt refills every released half itself. */
    if (step_stream_axis_brake.active != 0U ||
        step_stream_ring_next_fill_request(&step_stream_axis_ring, &candidate) != STEP_STREAM_RING_STATUS_OK) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST;
    }
    step_stream_axis_held_half = candidate;
    step_stream_axis_exit_critical(primask);
    *half = candidate;
    *records = &step_stream_axis_dma_buffer
                   [(size_t)candidate * STEP_STREAM_AXIS_HALF_RECORD_COUNT * STEP_STREAM_FORMAT_DMA_BURST_LENGTH];
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_commit_half(uint8_t half) {
    if (step_stream_axis_ring_needs_prepare() != 0U) {
        return STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
    }
    /* Release of the half and its announcement as FULL are one step: a braking that began in between would fill the
     * half itself and the announcement would then fail. */
    const uint32_t primask = step_stream_axis_enter_critical();
    step_stream_axis_held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    StepStreamAxisStatus_t status = STEP_STREAM_AXIS_STATUS_OK;
    if (step_stream_axis_brake.active != 0U) {
        /* The braking began while the task wrote this half: the ramp replaces its records, any other commit is void. */
        (void)step_stream_axis_logic_brake_commit(
            &step_stream_axis_brake,
            &step_stream_axis_ring,
            step_stream_axis_dma_buffer,
            STEP_STREAM_AXIS_RECORD_COUNT,
            half
        );
    } else if (step_stream_ring_mark_filled(&step_stream_axis_ring, half) != STEP_STREAM_RING_STATUS_OK) {
        status = STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    step_stream_axis_exit_critical(primask);
    return status;
}

StepStreamAxisStatus_t step_stream_axis_start_stream(uint32_t startup_period_ns) {
    if (step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    const uint32_t timer_clock_hz = step_stream_axis_timer_clock_hz();
    if (timer_clock_hz == 0U) {
        return STEP_STREAM_AXIS_STATUS_CLOCK_ERROR;
    }
    const uint64_t ticks = step_stream_axis_ceil_div_u64(
        (uint64_t)startup_period_ns * timer_clock_hz,
        STEP_STREAM_AXIS_NANOSECONDS_PER_SECOND
    );
    if (startup_period_ns < STEP_STREAM_AXIS_MIN_STARTUP_NS || ticks < 2U ||
        ticks > STEP_STREAM_AXIS_MAX_PERIOD_TICKS) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    return step_stream_axis_launch((uint32_t)ticks, 0U, 0U);
}

/* Caller holds the critical section (or is the DMA interrupt itself) and the axis is STREAMING. */
static void step_stream_axis_halt(StepStreamAxisFault_t fault) {
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~TIM_DIER_UDE;
    STEP_STREAM_AXIS_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    if (step_stream_axis_logic_poll(step_stream_axis_dma_disabled, STEP_STREAM_AXIS_DMA_DISABLE_MAX_POLLS) == 0U &&
        fault == STEP_STREAM_AXIS_FAULT_NONE) {
        fault = STEP_STREAM_AXIS_FAULT_DMA;
    }

    __DSB();
    const uint32_t raw_steps = step_stream_axis_logic_read_steps(
        &step_stream_axis_snapshot_source,
        step_stream_axis_completed_half_buffers,
        STEP_STREAM_AXIS_RECORD_COUNT
    );
    uint32_t valid_steps = raw_steps;
    if (fault == STEP_STREAM_AXIS_FAULT_UNDERRUN) {
        /* The DMA had already entered a half that was never filled; only steps up to the half boundary count. */
        valid_steps = step_stream_axis_logic_boundary_steps(
            step_stream_axis_completed_half_buffers,
            STEP_STREAM_AXIS_RECORD_COUNT
        );
    }
    step_stream_axis_last_steps_done = valid_steps;
    step_stream_axis_last_stale_steps = step_stream_axis_logic_stale_steps(raw_steps, valid_steps);
    step_stream_axis_fault = fault;
    STEP_STREAM_AXIS_DMA_STREAM->CR &= ~(DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE);
    step_stream_axis_clear_dma_flags();
    NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    TIM4->CCER &= ~TIM_CCER_CC1E;
    TIM4->SR = 0U;
    step_stream_axis_state = STEP_STREAM_AXIS_STATE_IDLE;
    /* A limit interrupt that preempted the halt may have asked for a braking meanwhile; nothing is left to brake. The
     * DMA interrupt it pended is disabled, and the next launch clears that pending bit before it enables it. */
    step_stream_axis_brake_requested = 0U;
}

void step_stream_axis_stop(void) {
    const uint32_t primask = step_stream_axis_enter_critical();
    if (step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_halt(STEP_STREAM_AXIS_FAULT_NONE);
    }
    step_stream_axis_exit_critical(primask);
}

StepStreamAxisStatus_t step_stream_axis_set_listener(const StepStreamAxisListener_t *listener) {
    const uint32_t primask = step_stream_axis_enter_critical();
    if (step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    step_stream_axis_on_half_released = listener != NULL ? listener->on_half_released : NULL;
    step_stream_axis_on_fault = listener != NULL ? listener->on_fault : NULL;
    step_stream_axis_on_brake = listener != NULL ? listener->on_brake : NULL;
    step_stream_axis_exit_critical(primask);
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisFault_t step_stream_axis_get_fault(void) { return step_stream_axis_fault; }

uint32_t step_stream_axis_stale_steps(void) { return step_stream_axis_last_stale_steps; }

uint32_t step_stream_axis_steps_done(void) {
    if (step_stream_axis_state != STEP_STREAM_AXIS_STATE_STREAMING) {
        return step_stream_axis_last_steps_done;
    }
    return step_stream_axis_current_steps();
}

StepStreamAxisState_t step_stream_axis_get_state(void) { return step_stream_axis_state; }

/* Stops the axis from the interrupt and reports the fault. */
static void step_stream_axis_fail(StepStreamAxisFault_t fault) {
    step_stream_axis_halt(fault);
    if (step_stream_axis_on_fault != NULL) {
        step_stream_axis_on_fault(fault);
    }
}

StepStreamAxisStatus_t step_stream_axis_configure_brake(const StepStreamBrakeConfig_t *config) {
    const uint32_t primask = step_stream_axis_enter_critical();
    const uint8_t streaming = step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING ? 1U : 0U;
    /* Not ready from here on: a request that comes while the ramp is built stops the axis at once. */
    step_stream_axis_brake_ready = 0U;
    step_stream_axis_exit_critical(primask);
    if (streaming != 0U) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    /* Building the ramp takes milliseconds, so it runs with the interrupts on. Only the task that starts the streams
     * calls this, before the first start, so no stream begins meanwhile. */
    const StepStreamBrakeStatus_t status = step_stream_brake_init(
        &step_stream_axis_brake_table,
        config,
        step_stream_axis_brake_storage,
        STEP_STREAM_AXIS_BRAKE_CAPACITY
    );
    step_stream_axis_brake_ready = status == STEP_STREAM_BRAKE_STATUS_OK ? 1U : 0U;
    return status == STEP_STREAM_BRAKE_STATUS_OK ? STEP_STREAM_AXIS_STATUS_OK
                                                 : STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
}

void step_stream_axis_request_brake(void) {
    if (step_stream_axis_state != STEP_STREAM_AXIS_STATE_STREAMING) {
        return;
    }
    step_stream_axis_brake_requested = 1U;
    __DMB();
    NVIC_SetPendingIRQ(DMA1_Stream0_IRQn);
}

uint8_t step_stream_axis_is_braking(void) {
    return (uint8_t)(step_stream_axis_state == STEP_STREAM_AXIS_STATE_STREAMING &&
                     step_stream_axis_brake_active() != 0U);
}

void step_stream_axis_get_brake_stats(StepStreamAxisBrakeStats_t *stats) {
    if (stats == NULL) {
        return;
    }
    stats->brakes_started = step_stream_axis_brake_stats.brakes_started;
    stats->brakes_completed = step_stream_axis_brake_stats.brakes_completed;
    stats->stopped_at_once = step_stream_axis_brake_stats.stopped_at_once;
    stats->retries = step_stream_axis_brake_stats.retries;
}

/* A brake that cannot be placed stops the axis at once; the feeder is told first so that it stops feeding. */
static void step_stream_axis_brake_stop_now(void) {
    step_stream_axis_brake_requested = 0U;
    ++step_stream_axis_brake_stats.stopped_at_once;
    /* The stand has no feeder to tell. */
    if (step_stream_axis_on_brake != NULL && step_stream_axis_auto_refill == 0U) {
        step_stream_axis_on_brake();
    }
    step_stream_axis_halt(STEP_STREAM_AXIS_FAULT_NONE);
}

/* DMA interrupt, STREAMING, the pending half events already handled. */
static void step_stream_axis_run_brake(void) {
    if (step_stream_axis_brake_active() != 0U) {
        step_stream_axis_brake_requested = 0U;
        return;
    }
    /* The stand pattern is not a plan, and without a ramp there is nothing to brake with. */
    if (step_stream_axis_auto_refill != 0U || step_stream_axis_brake_ready == 0U) {
        step_stream_axis_brake_stop_now();
        return;
    }

    const StepStreamAxisBrakeBegin_t result = step_stream_axis_logic_brake_begin(
        &step_stream_axis_brake,
        &step_stream_axis_brake_table,
        &step_stream_axis_ring,
        step_stream_axis_dma_buffer,
        STEP_STREAM_AXIS_RECORD_COUNT,
        step_stream_axis_current_steps(),
        STEP_STREAM_AXIS_BRAKE_MARGIN_RECORDS,
        step_stream_axis_held_half
    );
    switch (result) {
    case STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED:
        step_stream_axis_brake_requested = 0U;
        step_stream_axis_brake_attempt = 0U;
        ++step_stream_axis_brake_stats.brakes_started;
        if (step_stream_axis_on_brake != NULL) {
            step_stream_axis_on_brake();
        }
        break;
    case STEP_STREAM_AXIS_LOGIC_BRAKE_RETRY:
        ++step_stream_axis_brake_stats.retries;
        if (step_stream_axis_brake_attempt >= STEP_STREAM_AXIS_BRAKE_MAX_RETRIES) {
            step_stream_axis_brake_attempt = 0U;
            step_stream_axis_brake_stop_now();
        } else {
            ++step_stream_axis_brake_attempt;
            NVIC_SetPendingIRQ(DMA1_Stream0_IRQn);
        }
        break;
    case STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW:
        step_stream_axis_brake_stop_now();
        break;
    default:
        step_stream_axis_brake_requested = 0U;
        step_stream_axis_fail(STEP_STREAM_AXIS_FAULT_SEQUENCE);
        break;
    }
}

/* Returns 1 when the axis was stopped by a ring fault. The event's flag is already cleared and counted, so a flag
 * of the other half still pending is counted once more by the halt, and the step count stays exact. */
static uint8_t step_stream_axis_handle_half_event(uint8_t half_transfer) {
    const uint8_t braking = step_stream_axis_brake_active();
    const StepStreamAxisLogicAction_t action = step_stream_axis_logic_half_event(
        &step_stream_axis_ring,
        half_transfer,
        braking != 0U ? 0U : step_stream_axis_auto_refill
    );
    if (action == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED) {
        if (braking != 0U) {
            /* The played half is half 0 after HT and half 1 after TC. */
            const StepStreamAxisBrakeEvent_t event = step_stream_axis_logic_brake_half_released(
                &step_stream_axis_brake,
                &step_stream_axis_ring,
                step_stream_axis_dma_buffer,
                STEP_STREAM_AXIS_RECORD_COUNT,
                half_transfer != 0U ? 0U : 1U
            );
            if (event == STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_COMPLETE) {
                ++step_stream_axis_brake_stats.brakes_completed;
                step_stream_axis_halt(STEP_STREAM_AXIS_FAULT_NONE);
                return 1U;
            }
            if (event == STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT) {
                step_stream_axis_fail(STEP_STREAM_AXIS_FAULT_SEQUENCE);
                return 1U;
            }
            return 0U;
        }
        if (step_stream_axis_on_half_released != NULL) {
            step_stream_axis_on_half_released();
        }
        return 0U;
    }

    step_stream_axis_fail(
        action == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN ? STEP_STREAM_AXIS_FAULT_UNDERRUN
                                                               : STEP_STREAM_AXIS_FAULT_SEQUENCE
    );
    return 1U;
}

void DMA1_Stream0_IRQHandler(void) {
    if (step_stream_axis_state != STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_clear_dma_flags();
        step_stream_axis_brake_requested = 0U;
        return;
    }
    const uint32_t status = DMA1->LISR;
    if ((status & DMA_LISR_TEIF0) != 0U) {
        step_stream_axis_fail(STEP_STREAM_AXIS_FAULT_DMA);
        return;
    }
    if ((status & DMA_LISR_HTIF0) != 0U) {
        ++step_stream_axis_completed_half_buffers;
        DMA1->LIFCR = DMA_LIFCR_CHTIF0;
        if (step_stream_axis_handle_half_event(1U) != 0U) {
            return;
        }
    }
    if ((status & DMA_LISR_TCIF0) != 0U) {
        ++step_stream_axis_completed_half_buffers;
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
        if (step_stream_axis_handle_half_event(0U) != 0U) {
            return;
        }
    }
    /* After the half events: the ring state and the step count that the braking reads are then up to date. */
    if (step_stream_axis_brake_requested != 0U) {
        step_stream_axis_run_brake();
    }
}

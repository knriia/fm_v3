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

/* Flag bits of a DMA stream inside LISR/HISR (and LIFCR/HIFCR), relative to the stream's position in the register. */
#define STEP_STREAM_AXIS_DMA_FLAG_FEIF (1UL << 0U)
#define STEP_STREAM_AXIS_DMA_FLAG_DMEIF (1UL << 2U)
#define STEP_STREAM_AXIS_DMA_FLAG_TEIF (1UL << 3U)
#define STEP_STREAM_AXIS_DMA_FLAG_HTIF (1UL << 4U)
#define STEP_STREAM_AXIS_DMA_FLAG_TCIF (1UL << 5U)
#define STEP_STREAM_AXIS_DMA_FLAGS_ALL                                                                                 \
    (STEP_STREAM_AXIS_DMA_FLAG_FEIF | STEP_STREAM_AXIS_DMA_FLAG_DMEIF | STEP_STREAM_AXIS_DMA_FLAG_TEIF |               \
        STEP_STREAM_AXIS_DMA_FLAG_HTIF | STEP_STREAM_AXIS_DMA_FLAG_TCIF)
/* Position of stream n (0..7) inside its register: 0, 6, 16, 22 for the streams 0..3, the same again for 4..7. */
#define STEP_STREAM_AXIS_DMA_FLAG_SHIFT(stream) ((((stream) & 1U) != 0U ? 6U : 0U) + (((stream) & 2U) != 0U ? 16U : 0U))
#define STEP_STREAM_AXIS_DMA_FLAGS_IN_HIGH_REGISTER(stream) ((stream) >= 4U)

/* Everything that differs between the axes: the timer and its STEP pin, the DMA stream that feeds it. */
typedef struct {
    MotionAxis_t motion_axis;
    TIM_TypeDef *timer;
    volatile uint32_t *timer_rcc_enable_register;
    uint32_t timer_rcc_enable_bit;
    GPIO_TypeDef *step_port;
    uint32_t step_gpio_rcc_enable_bit;
    uint8_t step_pin;
    uint8_t step_alternate_function;
    DMA_TypeDef *dma;
    uint32_t dma_rcc_enable_bit;
    DMA_Stream_TypeDef *dma_stream;
    DMAMUX_Channel_TypeDef *dma_mux_channel;
    uint32_t dma_request;
    IRQn_Type dma_irq;
    uint8_t dma_flag_shift;
    uint8_t dma_flags_in_high_register;
} StepStreamAxisHardware_t;

struct StepStreamAxis {
    const StepStreamAxisHardware_t *hardware;
    uint16_t *dma_buffer;
    uint16_t *brake_storage;

    volatile StepStreamAxisState_t state;
    volatile uint32_t completed_half_buffers;
    volatile uint32_t last_steps_done;
    volatile uint32_t last_stale_steps;
    StepStreamRing_t ring;
    volatile StepStreamAxisFault_t fault;
    volatile uint8_t auto_refill;
    void (*volatile on_half_released)(void *context);
    void (*volatile on_fault)(void *context, StepStreamAxisFault_t fault);
    void (*volatile on_brake)(void *context);
    void *volatile listener_context;

    StepStreamBrake_t brake_table;
    volatile uint8_t brake_ready;
    /* Written by the DMA interrupt and, under a critical section, by the task. Read through
     * step_stream_axis_brake_active() wherever the other side may change it. */
    StepStreamAxisBrake_t brake;
    /* Set by step_stream_axis_request_brake from any interrupt. Cleared by the DMA interrupt, and by launch and
     * prepare while the axis is IDLE (no interrupt of the stream runs then). A lost update is harmless: a request is
     * either already handled or pends the DMA interrupt once more. */
    volatile uint8_t brake_requested;
    uint8_t brake_attempt;
    /* The EMPTY half that the task acquired and has not committed yet, or NO_HALF. */
    volatile uint8_t held_half;
    volatile StepStreamAxisBrakeStats_t brake_stats;
};

static const StepStreamAxisHardware_t step_stream_axis_x_hardware = {
    .motion_axis = MOTION_AXIS_X,
    .timer = TIM4,
    .timer_rcc_enable_register = &RCC->APB1LENR,
    .timer_rcc_enable_bit = RCC_APB1LENR_TIM4EN,
    .step_port = GPIOB,
    .step_gpio_rcc_enable_bit = RCC_AHB4ENR_GPIOBEN,
    .step_pin = 6U,
    .step_alternate_function = 2U,
    .dma = DMA1,
    .dma_rcc_enable_bit = RCC_AHB1ENR_DMA1EN,
    .dma_stream = DMA1_Stream0,
    .dma_mux_channel = DMAMUX1_Channel0,
    /* TIM4_UP. */
    .dma_request = 32U,
    .dma_irq = DMA1_Stream0_IRQn,
    .dma_flag_shift = STEP_STREAM_AXIS_DMA_FLAG_SHIFT(0U),
    .dma_flags_in_high_register = STEP_STREAM_AXIS_DMA_FLAGS_IN_HIGH_REGISTER(0U),
};

static uint16_t step_stream_axis_x_dma_buffer[STEP_STREAM_AXIS_BUFFER_WORD_COUNT]
    __attribute__((section(".stepper_dma_buffers"), aligned(64)));
static uint16_t step_stream_axis_x_brake_storage[STEP_STREAM_AXIS_BRAKE_CAPACITY];

/* Indexed by MotionAxis_t; an axis without a stream channel has no hardware. */
static StepStreamAxis_t step_stream_axis_instances[MOTION_AXIS_HARDWARE_COUNT] = {
    [MOTION_AXIS_X] =
        {
            .hardware = &step_stream_axis_x_hardware,
            .dma_buffer = step_stream_axis_x_dma_buffer,
            .brake_storage = step_stream_axis_x_brake_storage,
            .held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF,
        },
};

StepStreamAxis_t *step_stream_axis_get(MotionAxis_t axis) {
    if ((uint32_t)axis >= (uint32_t)MOTION_AXIS_HARDWARE_COUNT || step_stream_axis_instances[axis].hardware == NULL) {
        return NULL;
    }
    return &step_stream_axis_instances[axis];
}

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

/* Kernel clock of the APB1 timers (TIM4 today); keep this calculation identical to motion_axis_hardware.c. It does not
 * depend on the axis yet: a channel on another bus needs the bus in its hardware table first. */
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

static uint8_t step_stream_axis_brake_active(const StepStreamAxis_t *axis) {
    return *(const volatile uint8_t *)&axis->brake.active;
}

uint32_t step_stream_axis_enter_critical(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

void step_stream_axis_exit_critical(uint32_t primask) {
    __DMB();
    __set_PRIMASK(primask);
}

static uint32_t step_stream_axis_dma_status(const StepStreamAxis_t *axis) {
    const StepStreamAxisHardware_t *hardware = axis->hardware;
    const uint32_t status = hardware->dma_flags_in_high_register != 0U ? hardware->dma->HISR : hardware->dma->LISR;
    return (status >> hardware->dma_flag_shift) & STEP_STREAM_AXIS_DMA_FLAGS_ALL;
}

static void step_stream_axis_dma_clear(const StepStreamAxis_t *axis, uint32_t flags) {
    const StepStreamAxisHardware_t *hardware = axis->hardware;
    const uint32_t mask = flags << hardware->dma_flag_shift;
    if (hardware->dma_flags_in_high_register != 0U) {
        hardware->dma->HIFCR = mask;
    } else {
        hardware->dma->LIFCR = mask;
    }
}

static uint32_t step_stream_axis_pending_half_buffers(const void *context) {
    const uint32_t flags = step_stream_axis_dma_status(context);
    return ((flags & STEP_STREAM_AXIS_DMA_FLAG_HTIF) != 0U ? 1U : 0U) +
           ((flags & STEP_STREAM_AXIS_DMA_FLAG_TCIF) != 0U ? 1U : 0U);
}

static uint32_t step_stream_axis_dma_ndtr(const void *context) {
    const StepStreamAxis_t *axis = context;
    return axis->hardware->dma_stream->NDTR;
}

static uint8_t step_stream_axis_dma_disabled(const void *context) {
    const StepStreamAxis_t *axis = context;
    return (axis->hardware->dma_stream->CR & DMA_SxCR_EN) == 0U ? 1U : 0U;
}

static StepStreamAxisSnapshotSource_t step_stream_axis_snapshot_source(const StepStreamAxis_t *axis) {
    return (StepStreamAxisSnapshotSource_t){
        .pending_half_buffers = step_stream_axis_pending_half_buffers,
        .ndtr = step_stream_axis_dma_ndtr,
        .context = axis,
    };
}

static void step_stream_axis_clear_dma_flags(const StepStreamAxis_t *axis) {
    step_stream_axis_dma_clear(axis, STEP_STREAM_AXIS_DMA_FLAGS_ALL);
}

static void step_stream_axis_configure_gpio(const StepStreamAxis_t *axis) {
    const StepStreamAxisHardware_t *hardware = axis->hardware;
    const uint32_t pin = hardware->step_pin;
    GPIO_TypeDef *port = hardware->step_port;
    RCC->AHB4ENR |= hardware->step_gpio_rcc_enable_bit;
    (void)RCC->AHB4ENR;
    __DSB();

    port->MODER = (port->MODER & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << (pin * 2U))) | (3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));
    const uint32_t afr_index = pin / 8U;
    const uint32_t afr_shift = (pin % 8U) * 4U;
    port->AFR[afr_index] =
        (port->AFR[afr_index] & ~(0xFUL << afr_shift)) | ((uint32_t)hardware->step_alternate_function << afr_shift);
}

static void
step_stream_axis_configure_timer(const StepStreamAxis_t *axis, uint32_t period_ticks, uint32_t pulse_ticks) {
    TIM_TypeDef *timer = axis->hardware->timer;
    timer->CR1 = TIM_CR1_ARPE;
    timer->CR2 = 0U;
    timer->SMCR = 0U;
    timer->DIER = 0U;
    timer->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    timer->CCER = 0U;
    timer->PSC = 0U;
    timer->DCR = (STEP_STREAM_AXIS_TIMER_DBA_ARR << TIM_DCR_DBA_Pos) | (STEP_STREAM_AXIS_TIMER_DBL << TIM_DCR_DBL_Pos);
    timer->CNT = 0U;
    timer->ARR = (uint16_t)(period_ticks - 1U);
    timer->CCR1 = (uint16_t)pulse_ticks;
    timer->EGR = TIM_EGR_UG;
    timer->SR = 0U;
}

/* Returns 0 when the stream did not disable; the DMA registers are then left untouched. */
static uint8_t step_stream_axis_configure_dma(StepStreamAxis_t *axis) {
    const StepStreamAxisHardware_t *hardware = axis->hardware;
    DMA_Stream_TypeDef *stream = hardware->dma_stream;
    RCC->AHB1ENR |= hardware->dma_rcc_enable_bit;
    (void)RCC->AHB1ENR;
    __DSB();

    hardware->dma_mux_channel->CCR = hardware->dma_request;
    stream->CR &= ~DMA_SxCR_EN;
    if (step_stream_axis_logic_poll(step_stream_axis_dma_disabled, axis, STEP_STREAM_AXIS_DMA_DISABLE_MAX_POLLS) ==
        0U) {
        return 0U;
    }

    step_stream_axis_clear_dma_flags(axis);
    stream->PAR = (uint32_t)&hardware->timer->DMAR;
    stream->M0AR = (uint32_t)axis->dma_buffer;
    stream->NDTR = STEP_STREAM_AXIS_BUFFER_WORD_COUNT;
    stream->FCR = 0U;
    stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_DIR_0 |
                 DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
    NVIC_ClearPendingIRQ(hardware->dma_irq);
    NVIC_SetPriority(hardware->dma_irq, STEP_STREAM_AXIS_DMA_IRQ_PRIORITY);
    NVIC_EnableIRQ(hardware->dma_irq);
    stream->CR |= DMA_SxCR_EN;
    return 1U;
}

static uint32_t step_stream_axis_current_steps(const StepStreamAxis_t *axis) {
    const StepStreamAxisSnapshotSource_t source = step_stream_axis_snapshot_source(axis);
    const uint32_t primask = step_stream_axis_enter_critical();
    const uint32_t steps =
        step_stream_axis_logic_read_steps(&source, axis->completed_half_buffers, STEP_STREAM_AXIS_RECORD_COUNT);
    step_stream_axis_exit_critical(primask);
    return steps;
}

/* Enable the output only now: with CNT below CCR1 it would drive STEP high before the first period starts. */
static void step_stream_axis_run_timer(const StepStreamAxis_t *axis) {
    axis->hardware->timer->CCER = TIM_CCER_CC1E;
    axis->hardware->timer->CR1 |= TIM_CR1_CEN;
}

/* Common start: the ring is already prepared (stream) or prepared here (stand). The timer idles at period_ticks with
 * the first pulse at pulse_ticks (0 = none) until the DMA delivers the first record. */
static StepStreamAxisStatus_t
step_stream_axis_launch(StepStreamAxis_t *axis, uint32_t period_ticks, uint32_t pulse_ticks, uint8_t stand) {
    const StepStreamAxisHardware_t *hardware = axis->hardware;
    const uint32_t primask = step_stream_axis_enter_critical();
    if (axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }

    const MotionAxisHardwareStatus_t motion_status = motion_axis_hardware_get_status(hardware->motion_axis);
    if (motion_status == MOTION_AXIS_HARDWARE_STATUS_STARTING || motion_status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
        motion_status == MOTION_AXIS_HARDWARE_STATUS_RUNNING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_AXIS_BUSY;
    }

    /* Decide on the ring before any register is touched, so a refusal leaves the hardware as it was. */
    if (stand != 0U) {
        if (step_stream_axis_logic_prepare_stand_ring(&axis->ring) != 0U) {
            step_stream_axis_exit_critical(primask);
            return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
        }
    } else {
        const StepStreamRingStatus_t ring_status = step_stream_ring_start(&axis->ring);
        if (ring_status != STEP_STREAM_RING_STATUS_OK) {
            step_stream_axis_exit_critical(primask);
            return ring_status == STEP_STREAM_RING_STATUS_NOT_READY ? STEP_STREAM_AXIS_STATUS_NOT_READY
                                                                    : STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
        }
    }

    *hardware->timer_rcc_enable_register |= hardware->timer_rcc_enable_bit;
    (void)*hardware->timer_rcc_enable_register;
    __DSB();
    step_stream_axis_configure_gpio(axis);
    step_stream_axis_configure_timer(axis, period_ticks, pulse_ticks);
    axis->completed_half_buffers = 0U;
    axis->last_steps_done = 0U;
    axis->last_stale_steps = 0U;
    axis->fault = STEP_STREAM_AXIS_FAULT_NONE;
    axis->auto_refill = stand;
    step_stream_axis_logic_brake_reset(&axis->brake);
    axis->brake_requested = 0U;
    axis->brake_attempt = 0U;
    axis->held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    if (step_stream_axis_configure_dma(axis) == 0U) {
        /* The ring was started above; return it to the start so that the next attempt begins clean. */
        step_stream_ring_init(&axis->ring);
        axis->fault = STEP_STREAM_AXIS_FAULT_DMA;
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_DMA_ERROR;
    }
    hardware->timer->DIER = TIM_DIER_UDE;
    axis->state = STEP_STREAM_AXIS_STATE_STREAMING;
    __DMB();
    /* A stream is armed only: the timer waits for step_stream_axis_trigger. The stand runs at once. */
    if (stand != 0U) {
        step_stream_axis_run_timer(axis);
    }
    step_stream_axis_exit_critical(primask);
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_start(StepStreamAxis_t *axis, uint32_t frequency_hz, uint32_t pulse_width_ns) {
    if (axis == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    if (axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
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

    if (step_stream_format_fill(axis->dma_buffer, STEP_STREAM_AXIS_RECORD_COUNT, period_ticks, pulse_ticks) !=
        STEP_STREAM_FORMAT_STATUS_OK) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    return step_stream_axis_launch(axis, period_ticks, pulse_ticks, 1U);
}

uint32_t step_stream_axis_get_timer_clock_hz(const StepStreamAxis_t *axis) {
    return axis != NULL ? step_stream_axis_timer_clock_hz() : 0U;
}

StepStreamAxisStatus_t step_stream_axis_prepare_stream(StepStreamAxis_t *axis) {
    if (axis == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    const uint32_t primask = step_stream_axis_enter_critical();
    if (axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    /* The DMA is stopped and the interrupt ignores events while IDLE, so nobody else touches the ring. */
    step_stream_ring_init(&axis->ring);
    step_stream_axis_logic_brake_reset(&axis->brake);
    axis->brake_requested = 0U;
    axis->held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    axis->fault = STEP_STREAM_AXIS_FAULT_NONE;
    axis->last_steps_done = 0U;
    axis->last_stale_steps = 0U;
    step_stream_axis_exit_critical(primask);
    return STEP_STREAM_AXIS_STATUS_OK;
}

/* An IDLE axis whose ring still holds an earlier run must be prepared before the task touches the halves. */
static uint8_t step_stream_axis_ring_needs_prepare(const StepStreamAxis_t *axis) {
    return (axis->state == STEP_STREAM_AXIS_STATE_IDLE &&
               step_stream_ring_get_state(&axis->ring) != STEP_STREAM_RING_STATE_NOT_STARTED)
               ? 1U
               : 0U;
}

StepStreamAxisStatus_t step_stream_axis_acquire_half(StepStreamAxis_t *axis, uint8_t *half, uint16_t **records) {
    if (axis == NULL || half == NULL || records == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    if (step_stream_axis_ring_needs_prepare(axis) != 0U) {
        return STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
    }
    /* The braking must see the held half and the half itself in one piece, so it cannot start in between. */
    const uint32_t primask = step_stream_axis_enter_critical();
    uint8_t candidate = 0U;
    /* While braking, the interrupt refills every released half itself. */
    if (axis->brake.active != 0U ||
        step_stream_ring_next_fill_request(&axis->ring, &candidate) != STEP_STREAM_RING_STATUS_OK) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST;
    }
    axis->held_half = candidate;
    step_stream_axis_exit_critical(primask);
    *half = candidate;
    *records =
        &axis->dma_buffer[(size_t)candidate * STEP_STREAM_AXIS_HALF_RECORD_COUNT * STEP_STREAM_FORMAT_DMA_BURST_LENGTH];
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisStatus_t step_stream_axis_commit_half(StepStreamAxis_t *axis, uint8_t half) {
    if (axis == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    if (step_stream_axis_ring_needs_prepare(axis) != 0U) {
        return STEP_STREAM_AXIS_STATUS_NOT_PREPARED;
    }
    /* Release of the half and its announcement as FULL are one step: a braking that began in between would fill the
     * half itself and the announcement would then fail. */
    const uint32_t primask = step_stream_axis_enter_critical();
    axis->held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    StepStreamAxisStatus_t status = STEP_STREAM_AXIS_STATUS_OK;
    if (axis->brake.active != 0U) {
        /* The braking began while the task wrote this half: the ramp replaces its records, any other commit is void. */
        (void)step_stream_axis_logic_brake_commit(
            &axis->brake,
            &axis->ring,
            axis->dma_buffer,
            STEP_STREAM_AXIS_RECORD_COUNT,
            half
        );
    } else if (step_stream_ring_mark_filled(&axis->ring, half) != STEP_STREAM_RING_STATUS_OK) {
        status = STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    step_stream_axis_exit_critical(primask);
    return status;
}

StepStreamAxisStatus_t step_stream_axis_arm_stream(StepStreamAxis_t *axis, uint32_t startup_period_ns) {
    if (axis == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    if (axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
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
    return step_stream_axis_launch(axis, (uint32_t)ticks, 0U, 0U);
}

void step_stream_axis_trigger(StepStreamAxis_t *const *axes, uint32_t count) {
    if (axes == NULL) {
        return;
    }
    uint8_t started = 0U;
    const uint32_t primask = step_stream_axis_enter_critical();
    for (uint32_t index = 0U; index < count; ++index) {
        const StepStreamAxis_t *axis = axes[index];
        /* A stop between arm and trigger left the axis IDLE: there is nothing to start then. */
        if (axis != NULL && axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
            step_stream_axis_run_timer(axis);
            started = 1U;
        }
    }
    step_stream_axis_exit_critical(primask);
    if (started != 0U) {
        /* A switch edge that came between the direction output and this start was seen by nobody: look at the level. */
        motion_axis_hardware_poll_stream_limit();
    }
}

/* Caller holds the critical section (or is the DMA interrupt itself) and the axis is STREAMING. */
static void step_stream_axis_halt(StepStreamAxis_t *axis, StepStreamAxisFault_t fault) {
    const StepStreamAxisHardware_t *hardware = axis->hardware;
    hardware->timer->CR1 &= ~TIM_CR1_CEN;
    hardware->timer->DIER &= ~TIM_DIER_UDE;
    hardware->dma_stream->CR &= ~DMA_SxCR_EN;
    if (step_stream_axis_logic_poll(step_stream_axis_dma_disabled, axis, STEP_STREAM_AXIS_DMA_DISABLE_MAX_POLLS) ==
            0U &&
        fault == STEP_STREAM_AXIS_FAULT_NONE) {
        fault = STEP_STREAM_AXIS_FAULT_DMA;
    }

    __DSB();
    const StepStreamAxisSnapshotSource_t source = step_stream_axis_snapshot_source(axis);
    const uint32_t raw_steps =
        step_stream_axis_logic_read_steps(&source, axis->completed_half_buffers, STEP_STREAM_AXIS_RECORD_COUNT);
    uint32_t valid_steps = raw_steps;
    if (fault == STEP_STREAM_AXIS_FAULT_UNDERRUN) {
        /* The DMA had already entered a half that was never filled; only steps up to the half boundary count. */
        valid_steps =
            step_stream_axis_logic_boundary_steps(axis->completed_half_buffers, STEP_STREAM_AXIS_RECORD_COUNT);
    }
    axis->last_steps_done = valid_steps;
    axis->last_stale_steps = step_stream_axis_logic_stale_steps(raw_steps, valid_steps);
    axis->fault = fault;
    hardware->dma_stream->CR &= ~(DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE);
    step_stream_axis_clear_dma_flags(axis);
    NVIC_DisableIRQ(hardware->dma_irq);
    NVIC_ClearPendingIRQ(hardware->dma_irq);
    hardware->timer->CCER &= ~TIM_CCER_CC1E;
    hardware->timer->SR = 0U;
    axis->state = STEP_STREAM_AXIS_STATE_IDLE;
    /* A limit interrupt that preempted the halt may have asked for a braking meanwhile; nothing is left to brake. The
     * DMA interrupt it pended is disabled, and the next launch clears that pending bit before it enables it. */
    axis->brake_requested = 0U;
}

void step_stream_axis_stop(StepStreamAxis_t *axis) {
    if (axis == NULL) {
        return;
    }
    const uint32_t primask = step_stream_axis_enter_critical();
    if (axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_halt(axis, STEP_STREAM_AXIS_FAULT_NONE);
    }
    step_stream_axis_exit_critical(primask);
}

StepStreamAxisStatus_t step_stream_axis_set_listener(StepStreamAxis_t *axis, const StepStreamAxisListener_t *listener) {
    if (axis == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    const uint32_t primask = step_stream_axis_enter_critical();
    if (axis->state == STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_exit_critical(primask);
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    axis->on_half_released = listener != NULL ? listener->on_half_released : NULL;
    axis->on_fault = listener != NULL ? listener->on_fault : NULL;
    axis->on_brake = listener != NULL ? listener->on_brake : NULL;
    axis->listener_context = listener != NULL ? listener->context : NULL;
    step_stream_axis_exit_critical(primask);
    return STEP_STREAM_AXIS_STATUS_OK;
}

StepStreamAxisFault_t step_stream_axis_get_fault(const StepStreamAxis_t *axis) {
    return axis != NULL ? axis->fault : STEP_STREAM_AXIS_FAULT_NONE;
}

uint32_t step_stream_axis_stale_steps(const StepStreamAxis_t *axis) {
    return axis != NULL ? axis->last_stale_steps : 0U;
}

uint32_t step_stream_axis_steps_done(const StepStreamAxis_t *axis) {
    if (axis == NULL) {
        return 0U;
    }
    if (axis->state != STEP_STREAM_AXIS_STATE_STREAMING) {
        return axis->last_steps_done;
    }
    return step_stream_axis_current_steps(axis);
}

StepStreamAxisState_t step_stream_axis_get_state(const StepStreamAxis_t *axis) {
    return axis != NULL ? axis->state : STEP_STREAM_AXIS_STATE_IDLE;
}

/* Stops the axis from the interrupt and reports the fault. */
static void step_stream_axis_fail(StepStreamAxis_t *axis, StepStreamAxisFault_t fault) {
    step_stream_axis_halt(axis, fault);
    if (axis->on_fault != NULL) {
        axis->on_fault(axis->listener_context, fault);
    }
}

StepStreamAxisStatus_t step_stream_axis_configure_brake(StepStreamAxis_t *axis, const StepStreamBrakeConfig_t *config) {
    if (axis == NULL) {
        return STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
    }
    const uint32_t primask = step_stream_axis_enter_critical();
    const uint8_t streaming = axis->state == STEP_STREAM_AXIS_STATE_STREAMING ? 1U : 0U;
    /* Not ready from here on: a request that comes while the ramp is built stops the axis at once. */
    axis->brake_ready = 0U;
    step_stream_axis_exit_critical(primask);
    if (streaming != 0U) {
        return STEP_STREAM_AXIS_STATUS_BUSY;
    }
    /* Building the ramp takes milliseconds, so it runs with the interrupts on. Only the task that starts the streams
     * calls this, before the first start, so no stream begins meanwhile. */
    const StepStreamBrakeStatus_t status =
        step_stream_brake_init(&axis->brake_table, config, axis->brake_storage, STEP_STREAM_AXIS_BRAKE_CAPACITY);
    axis->brake_ready = status == STEP_STREAM_BRAKE_STATUS_OK ? 1U : 0U;
    return status == STEP_STREAM_BRAKE_STATUS_OK ? STEP_STREAM_AXIS_STATUS_OK
                                                 : STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT;
}

void step_stream_axis_request_brake(StepStreamAxis_t *axis) {
    if (axis == NULL || axis->state != STEP_STREAM_AXIS_STATE_STREAMING) {
        return;
    }
    axis->brake_requested = 1U;
    __DMB();
    NVIC_SetPendingIRQ(axis->hardware->dma_irq);
}

uint8_t step_stream_axis_is_braking(const StepStreamAxis_t *axis) {
    return (uint8_t)(axis != NULL && axis->state == STEP_STREAM_AXIS_STATE_STREAMING &&
                     step_stream_axis_brake_active(axis) != 0U);
}

void step_stream_axis_get_brake_stats(const StepStreamAxis_t *axis, StepStreamAxisBrakeStats_t *stats) {
    if (stats == NULL) {
        return;
    }
    if (axis == NULL) {
        *stats = (StepStreamAxisBrakeStats_t){0};
        return;
    }
    stats->brakes_started = axis->brake_stats.brakes_started;
    stats->brakes_completed = axis->brake_stats.brakes_completed;
    stats->stopped_at_once = axis->brake_stats.stopped_at_once;
    stats->retries = axis->brake_stats.retries;
}

/* A brake that cannot be placed stops the axis at once; the feeder is told first so that it stops feeding. */
static void step_stream_axis_brake_stop_now(StepStreamAxis_t *axis) {
    axis->brake_requested = 0U;
    ++axis->brake_stats.stopped_at_once;
    /* The stand has no feeder to tell. */
    if (axis->on_brake != NULL && axis->auto_refill == 0U) {
        axis->on_brake(axis->listener_context);
    }
    step_stream_axis_halt(axis, STEP_STREAM_AXIS_FAULT_NONE);
}

/* DMA interrupt, STREAMING, the pending half events already handled. */
static void step_stream_axis_run_brake(StepStreamAxis_t *axis) {
    if (step_stream_axis_brake_active(axis) != 0U) {
        axis->brake_requested = 0U;
        return;
    }
    /* The stand pattern is not a plan, and without a ramp there is nothing to brake with. */
    if (axis->auto_refill != 0U || axis->brake_ready == 0U) {
        step_stream_axis_brake_stop_now(axis);
        return;
    }

    const StepStreamAxisBrakeBegin_t result = step_stream_axis_logic_brake_begin(
        &axis->brake,
        &axis->brake_table,
        &axis->ring,
        axis->dma_buffer,
        STEP_STREAM_AXIS_RECORD_COUNT,
        step_stream_axis_current_steps(axis),
        STEP_STREAM_AXIS_BRAKE_MARGIN_RECORDS,
        axis->held_half
    );
    switch (result) {
    case STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED:
        axis->brake_requested = 0U;
        axis->brake_attempt = 0U;
        ++axis->brake_stats.brakes_started;
        if (axis->on_brake != NULL) {
            axis->on_brake(axis->listener_context);
        }
        break;
    case STEP_STREAM_AXIS_LOGIC_BRAKE_RETRY:
        ++axis->brake_stats.retries;
        if (axis->brake_attempt >= STEP_STREAM_AXIS_BRAKE_MAX_RETRIES) {
            axis->brake_attempt = 0U;
            step_stream_axis_brake_stop_now(axis);
        } else {
            ++axis->brake_attempt;
            NVIC_SetPendingIRQ(axis->hardware->dma_irq);
        }
        break;
    case STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW:
        step_stream_axis_brake_stop_now(axis);
        break;
    default:
        axis->brake_requested = 0U;
        step_stream_axis_fail(axis, STEP_STREAM_AXIS_FAULT_SEQUENCE);
        break;
    }
}

/* Returns 1 when the axis was stopped by a ring fault. The event's flag is already cleared and counted, so a flag
 * of the other half still pending is counted once more by the halt, and the step count stays exact. */
static uint8_t step_stream_axis_handle_half_event(StepStreamAxis_t *axis, uint8_t half_transfer) {
    const uint8_t braking = step_stream_axis_brake_active(axis);
    const StepStreamAxisLogicAction_t action =
        step_stream_axis_logic_half_event(&axis->ring, half_transfer, braking != 0U ? 0U : axis->auto_refill);
    if (action == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED) {
        if (braking != 0U) {
            /* The played half is half 0 after HT and half 1 after TC. */
            const StepStreamAxisBrakeEvent_t event = step_stream_axis_logic_brake_half_released(
                &axis->brake,
                &axis->ring,
                axis->dma_buffer,
                STEP_STREAM_AXIS_RECORD_COUNT,
                half_transfer != 0U ? 0U : 1U
            );
            if (event == STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_COMPLETE) {
                ++axis->brake_stats.brakes_completed;
                step_stream_axis_halt(axis, STEP_STREAM_AXIS_FAULT_NONE);
                return 1U;
            }
            if (event == STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT) {
                step_stream_axis_fail(axis, STEP_STREAM_AXIS_FAULT_SEQUENCE);
                return 1U;
            }
            return 0U;
        }
        if (axis->on_half_released != NULL) {
            axis->on_half_released(axis->listener_context);
        }
        return 0U;
    }

    step_stream_axis_fail(
        axis,
        action == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN ? STEP_STREAM_AXIS_FAULT_UNDERRUN
                                                               : STEP_STREAM_AXIS_FAULT_SEQUENCE
    );
    return 1U;
}

static void step_stream_axis_dma_irq(StepStreamAxis_t *axis) {
    if (axis->state != STEP_STREAM_AXIS_STATE_STREAMING) {
        step_stream_axis_clear_dma_flags(axis);
        axis->brake_requested = 0U;
        return;
    }
    const uint32_t status = step_stream_axis_dma_status(axis);
    if ((status & STEP_STREAM_AXIS_DMA_FLAG_TEIF) != 0U) {
        step_stream_axis_fail(axis, STEP_STREAM_AXIS_FAULT_DMA);
        return;
    }
    if ((status & STEP_STREAM_AXIS_DMA_FLAG_HTIF) != 0U) {
        ++axis->completed_half_buffers;
        step_stream_axis_dma_clear(axis, STEP_STREAM_AXIS_DMA_FLAG_HTIF);
        if (step_stream_axis_handle_half_event(axis, 1U) != 0U) {
            return;
        }
    }
    if ((status & STEP_STREAM_AXIS_DMA_FLAG_TCIF) != 0U) {
        ++axis->completed_half_buffers;
        step_stream_axis_dma_clear(axis, STEP_STREAM_AXIS_DMA_FLAG_TCIF);
        if (step_stream_axis_handle_half_event(axis, 0U) != 0U) {
            return;
        }
    }
    /* After the half events: the ring state and the step count that the braking reads are then up to date. */
    if (axis->brake_requested != 0U) {
        step_stream_axis_run_brake(axis);
    }
}

void DMA1_Stream0_IRQHandler(void) { step_stream_axis_dma_irq(&step_stream_axis_instances[MOTION_AXIS_X]); }

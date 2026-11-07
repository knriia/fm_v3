#include "motion_axis_hardware.h"

#include "motion/motion_event_source.h"

#include "stm32h723xx.h"
#include "system_stm32h7xx.h"

#include <math.h>
#include <stddef.h>

#define MOTION_AXIS_TIMER_IRQ_PRIORITY 5U
#define MOTION_AXIS_LIMIT_IRQ_PRIORITY 4U
#define MOTION_AXIS_NANOSECONDS_PER_SECOND 1000000000ULL
/* Fits 32 bits, so the step interrupt divides by a step rate with the hardware divider, not a library call. */
#define MOTION_AXIS_NANOSECONDS_PER_SECOND_U32 1000000000U
#define MOTION_AXIS_DWT_UNLOCK_KEY 0xC5ACCE55U
#define MOTION_AXIS_MAX_PERIOD_TICKS 65536ULL
#define MOTION_AXIS_ISR_GUARD_TICKS 256U
#define MOTION_AXIS_JOG_MAX_STEPS 1000000U
#define MOTION_AXIS_GPIO_MODE_OUTPUT 1U
#define MOTION_AXIS_GPIO_MODE_ALTERNATE 2U
#define MOTION_AXIS_GPIO_SPEED_VERY_HIGH 3U
#define MOTION_AXIS_TIMER_IRQ_MAX 2U

/* Board wiring of one axis. Each axis owns its timer, its output port and its switch port. */
typedef struct {
    TIM_TypeDef *timer;
    volatile uint32_t *timer_clock_enable;
    uint32_t timer_clock_enable_bit;
    /* TIM1 and TIM8 sit on APB2 and are advanced timers: their outputs also need BDTR.MOE. */
    uint8_t timer_on_apb2;
    uint8_t advanced_timer;
    /* Advanced timers raise update and compare on separate vectors; both call the same handler. */
    IRQn_Type timer_irq[MOTION_AXIS_TIMER_IRQ_MAX];
    uint8_t timer_irq_count;
    uint8_t channel; /* 1..4 */
    uint8_t step_alternate_function;
    GPIO_TypeDef *output_port;
    uint32_t output_port_clock_bit;
    uint8_t step_pin;
    uint8_t direction_pin;
    GPIO_TypeDef *enable_port;
    uint32_t enable_port_clock_bit;
    uint8_t enable_pin;
    /* Each switch has its own port; SYSCFG_EXTICR port code: A = 0, B = 1, ... */
    GPIO_TypeDef *start_limit_port;
    uint32_t start_limit_port_clock_bit;
    uint8_t start_limit_port_index;
    GPIO_TypeDef *end_limit_port;
    uint32_t end_limit_port_clock_bit;
    uint8_t end_limit_port_index;
    /* A switch pin number is also its EXTI line. */
    uint8_t start_limit_pin;
    uint8_t end_limit_pin;
    IRQn_Type start_limit_irq;
    IRQn_Type end_limit_irq;
} MotionAxisBoard_t;

static const MotionAxisBoard_t motion_axis_boards[MOTION_AXIS_HARDWARE_COUNT] = {
    [MOTION_AXIS_X] =
        {
            .timer = TIM4,
            .timer_clock_enable = &RCC->APB1LENR,
            .timer_clock_enable_bit = RCC_APB1LENR_TIM4EN,
            .timer_on_apb2 = 0U,
            .advanced_timer = 0U,
            .timer_irq = {TIM4_IRQn},
            .timer_irq_count = 1U,
            .channel = 1U,
            .step_alternate_function = 2U,
            .output_port = GPIOB,
            .output_port_clock_bit = RCC_AHB4ENR_GPIOBEN,
            .step_pin = 6U,
            .direction_pin = 7U,
            .enable_port = GPIOB,
            .enable_port_clock_bit = RCC_AHB4ENR_GPIOBEN,
            .enable_pin = 8U,
            .start_limit_port = GPIOF,
            .start_limit_port_clock_bit = RCC_AHB4ENR_GPIOFEN,
            .start_limit_port_index = 5U,
            .end_limit_port = GPIOF,
            .end_limit_port_clock_bit = RCC_AHB4ENR_GPIOFEN,
            .end_limit_port_index = 5U,
            .start_limit_pin = 10U,
            .end_limit_pin = 9U,
            .start_limit_irq = EXTI15_10_IRQn,
            .end_limit_irq = EXTI9_5_IRQn,
        },
    [MOTION_AXIS_Y] =
        {
            .timer = TIM1,
            .timer_clock_enable = &RCC->APB2ENR,
            .timer_clock_enable_bit = RCC_APB2ENR_TIM1EN,
            .timer_on_apb2 = 1U,
            .advanced_timer = 1U,
            .timer_irq = {TIM1_UP_IRQn, TIM1_CC_IRQn},
            .timer_irq_count = 2U,
            .channel = 2U,
            .step_alternate_function = 1U,
            .output_port = GPIOA,
            .output_port_clock_bit = RCC_AHB4ENR_GPIOAEN,
            .step_pin = 9U,
            .direction_pin = 10U,
            .enable_port = GPIOA,
            .enable_port_clock_bit = RCC_AHB4ENR_GPIOAEN,
            .enable_pin = 11U,
            .start_limit_port = GPIOE,
            .start_limit_port_clock_bit = RCC_AHB4ENR_GPIOEEN,
            .start_limit_port_index = 4U,
            .end_limit_port = GPIOE,
            .end_limit_port_clock_bit = RCC_AHB4ENR_GPIOEEN,
            .end_limit_port_index = 4U,
            .start_limit_pin = 5U,
            .end_limit_pin = 4U,
            .start_limit_irq = EXTI9_5_IRQn,
            .end_limit_irq = EXTI4_IRQn,
        },
    [MOTION_AXIS_Z] =
        {
            .timer = TIM3,
            .timer_clock_enable = &RCC->APB1LENR,
            .timer_clock_enable_bit = RCC_APB1LENR_TIM3EN,
            .timer_on_apb2 = 0U,
            .advanced_timer = 0U,
            .timer_irq = {TIM3_IRQn},
            .timer_irq_count = 1U,
            .channel = 3U,
            .step_alternate_function = 2U,
            .output_port = GPIOC,
            .output_port_clock_bit = RCC_AHB4ENR_GPIOCEN,
            .step_pin = 8U,
            .direction_pin = 9U,
            /* ENA is the only Z output on GPIOA. */
            .enable_port = GPIOA,
            .enable_port_clock_bit = RCC_AHB4ENR_GPIOAEN,
            .enable_pin = 8U,
            .start_limit_port = GPIOE,
            .start_limit_port_clock_bit = RCC_AHB4ENR_GPIOEEN,
            .start_limit_port_index = 4U,
            .end_limit_port = GPIOF,
            .end_limit_port_clock_bit = RCC_AHB4ENR_GPIOFEN,
            .end_limit_port_index = 5U,
            .start_limit_pin = 6U,
            .end_limit_pin = 7U,
            .start_limit_irq = EXTI9_5_IRQn,
            .end_limit_irq = EXTI9_5_IRQn,
        },
};

typedef enum {
    MOTION_AXIS_STARTUP_NONE = 0,
    MOTION_AXIS_STARTUP_ENABLE_LEAD,
    MOTION_AXIS_STARTUP_DIRECTION_SETUP,
    MOTION_AXIS_STARTUP_FIRST_INTERVAL
} MotionAxisHardwareStartupPhase_t;

typedef struct {
    const MotionAxisBoard_t *board;
    /* Register bits of the STEP channel, derived from the board at init. */
    volatile uint32_t *step_compare;
    uint32_t step_compare_flag;
    uint32_t step_compare_interrupt;
    uint32_t step_output_enable;
    uint32_t step_output_polarity;
    MotionAxisHardwareConfig_t config;
    MotionExecutionEvent_t current_event;
    /* A jog repeats this event in the timer ISR instead of queuing one event per step. */
    uint32_t jog_steps_remaining;
    uint32_t jog_total_steps;
    uint32_t jog_initial_step_rate_hz;
    uint32_t jog_target_step_rate_hz;
    uint32_t jog_acceleration_steps_per_s2;
    uint32_t jog_step_rate_hz;
    uint32_t jog_interval_remainder_ns;
    uint32_t jog_interval_remainder_accumulator;
    uint64_t jog_interval_base_ns;
    volatile MotionAxisHardwareStatus_t status;
    MotionAxisHardwareStartupPhase_t startup_phase;
    uint32_t timer_clock_hz;
    uint32_t prescaler_divider;
    uint32_t pulse_ticks;
    uint32_t low_time_ticks;
    uint32_t timer_ticks_per_ns_q32;
    uint32_t direction_setup_ticks;
    uint32_t enable_direction_lead_ticks;
    uint64_t first_interval_ticks;
    uint64_t max_interval_ns;
    volatile uint32_t conversion_remainder_q32;
    volatile uint64_t delay_remaining_ticks;
    volatile uint32_t last_jog_steps_completed;
    volatile uint8_t current_direction_positive;
    volatile uint8_t jog_mode;
    /* The last move pulled events from event_source, so only then may the stream be reset. Unlike jog_mode it
     * survives the stop, for the fault and homing paths that reset the stream afterwards. */
    volatile uint8_t uses_stream;
    /* Set while the axis is idle; survives init. Read by the step ISR through begin and pulse_complete. */
    MotionEventSource_t event_source;
    volatile uint8_t stop_requested;
    volatile MotionAxisHardwareStopReason_t pending_stop_reason;
    volatile MotionAxisHardwareStopReason_t stop_reason;
    volatile uint8_t start_limit_release_stop_armed;
    volatile MotionAxisHardwareStartLimitHook_t start_limit_hook;
    /* Switch point of the current move, recorded once by whichever interrupt sees the switch first. */
    volatile uint8_t limit_latched;
    volatile int32_t limit_trigger_position;
    /* Set by the switch interrupt; the timer interrupt, which owns the step count, turns it into a deceleration. */
    volatile MotionAxisHardwareStopReason_t limit_deceleration_requested;
    /* Timer interrupt only: the switch this move is braking for, or NONE. */
    MotionAxisHardwareStopReason_t limit_deceleration_reason;
    /* Step-timer health; written by the timer interrupt, read by diagnostics. */
    volatile uint32_t timing_faults;
    volatile uint32_t isr_max_cycles;
    /* In timer kernel clocks, so the interrupt only multiplies by the prescaler. */
    volatile uint32_t min_slack_timer_clocks;
} MotionAxisHardwareContext_t;

typedef MotionAxisHardwareContext_t Ctx_t;

static MotionAxisHardwareContext_t motion_axis_contexts[MOTION_AXIS_HARDWARE_COUNT] = {
    [MOTION_AXIS_X] = {.board = &motion_axis_boards[MOTION_AXIS_X],
        .min_slack_timer_clocks = MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE},
    [MOTION_AXIS_Y] = {.board = &motion_axis_boards[MOTION_AXIS_Y],
        .min_slack_timer_clocks = MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE},
    [MOTION_AXIS_Z] = {.board = &motion_axis_boards[MOTION_AXIS_Z],
        .min_slack_timer_clocks = MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE},
};
extern uint32_t SystemCoreClock;

static Ctx_t *motion_axis_context(MotionAxis_t axis) {
    return (uint32_t)axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT ? &motion_axis_contexts[axis] : NULL;
}

static uint32_t motion_axis_pin_mask(uint8_t pin) { return 1UL << pin; }

static uint32_t motion_axis_limit_lines(const MotionAxisBoard_t *board) {
    return motion_axis_pin_mask(board->start_limit_pin) | motion_axis_pin_mask(board->end_limit_pin);
}

static int motion_axis_is_boolean(uint8_t value) { return value <= 1U; }

static int motion_axis_status_is_fault(MotionAxisHardwareStatus_t status) {
    return status >= MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT && status <= MOTION_AXIS_HARDWARE_STATUS_FAULT_CLOCK;
}

static int motion_axis_status_is_active(MotionAxisHardwareStatus_t status) {
    return status == MOTION_AXIS_HARDWARE_STATUS_STARTING || status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
           status == MOTION_AXIS_HARDWARE_STATUS_RUNNING;
}

static int motion_axis_status_can_start(MotionAxisHardwareStatus_t status) {
    return status == MOTION_AXIS_HARDWARE_STATUS_READY || status == MOTION_AXIS_HARDWARE_STATUS_COMPLETE ||
           status == MOTION_AXIS_HARDWARE_STATUS_STOPPED;
}

static int motion_axis_config_is_valid(const MotionAxisHardwareConfig_t *config) {
    return config != NULL && motion_axis_is_boolean(config->step_active_high) &&
           motion_axis_is_boolean(config->enable_active_high) && config->step_active_high == 1U &&
           motion_axis_is_boolean(config->positive_direction_high) &&
           motion_axis_is_boolean(config->positive_moves_toward_end) &&
           motion_axis_is_boolean(config->start_limit_active_high) &&
           motion_axis_is_boolean(config->end_limit_active_high) &&
           config->limit_pull >= MOTION_AXIS_HARDWARE_LIMIT_PULL_NONE &&
           config->limit_pull <= MOTION_AXIS_HARDWARE_LIMIT_PULL_DOWN &&
           config->step_pulse_width_ns >= MOTION_AXIS_HARDWARE_DM556_MIN_STEP_PULSE_NS &&
           config->step_low_time_ns >= MOTION_AXIS_HARDWARE_DM556_MIN_STEP_LOW_NS &&
           config->direction_setup_ns >= MOTION_AXIS_HARDWARE_DM556_MIN_DIRECTION_SETUP_NS &&
           config->enable_setup_ns >= MOTION_AXIS_HARDWARE_DM556_MIN_ENABLE_LEAD_NS &&
           config->direction_hold_ns <= config->step_pulse_width_ns &&
           config->enable_hold_ns <= config->step_pulse_width_ns;
}

static uint32_t motion_axis_apb_divider(uint32_t prescaler) {
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

static uint32_t motion_axis_ahb_divider(void) {
    static const uint16_t ahb_dividers[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U, 2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
    const uint32_t prescaler = (RCC->D1CFGR & RCC_D1CFGR_HPRE_Msk) >> RCC_D1CFGR_HPRE_Pos;
    return ahb_dividers[prescaler];
}

/* Timer kernel clock of the APB1 or APB2 timers; RCC_CFGR.TIMPRE applies to both buses. */
static uint32_t motion_axis_timer_clock_hz(const MotionAxisBoard_t *board) {
    const uint32_t prescaler = board->timer_on_apb2 != 0U
                                   ? (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2_Msk) >> RCC_D2CFGR_D2PPRE2_Pos
                                   : (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_Msk) >> RCC_D2CFGR_D2PPRE1_Pos;
    const uint32_t ahb_divider = motion_axis_ahb_divider();
    const uint32_t apb_divider = motion_axis_apb_divider(prescaler);
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

static uint32_t motion_axis_enter_critical(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void motion_axis_exit_critical(uint32_t primask) {
    __DMB();
    __set_PRIMASK(primask);
}

static void motion_axis_write_pin(GPIO_TypeDef *port, uint8_t pin, uint8_t high) {
    const uint32_t mask = motion_axis_pin_mask(pin);
    port->BSRR = high != 0U ? mask : mask << 16U;
}

static void motion_axis_set_pin_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode) {
    const uint32_t shift = (uint32_t)pin * 2U;
    port->MODER = (port->MODER & ~(3UL << shift)) | (mode << shift);
}

static uint8_t motion_axis_limit_pin_is_active(GPIO_TypeDef *port, uint8_t pin, uint8_t active_high) {
    const uint8_t high = (port->IDR & motion_axis_pin_mask(pin)) != 0U;
    return high == (active_high != 0U);
}

static uint8_t motion_axis_start_limit_active(const Ctx_t *c) {
    return motion_axis_limit_pin_is_active(
        c->board->start_limit_port,
        c->board->start_limit_pin,
        c->config.start_limit_active_high
    );
}

static uint8_t motion_axis_end_limit_active(const Ctx_t *c) {
    return motion_axis_limit_pin_is_active(
        c->board->end_limit_port,
        c->board->end_limit_pin,
        c->config.end_limit_active_high
    );
}

static uint8_t motion_axis_step_mask(const Ctx_t *c) { return (uint8_t)(1U << (uint32_t)(c - motion_axis_contexts)); }

static uint8_t motion_axis_direction_is_positive(const Ctx_t *c, const MotionExecutionEvent_t *event) {
    return (event->positive_direction_mask & motion_axis_step_mask(c)) != 0U;
}

static uint8_t motion_axis_event_is_valid(const Ctx_t *c, const MotionExecutionEvent_t *event) {
    const uint8_t mask = motion_axis_step_mask(c);
    return event->step_mask == mask && (event->positive_direction_mask & (uint8_t)~mask) == 0U;
}

static uint8_t motion_axis_direction_toward_end(const Ctx_t *c, uint8_t positive_direction) {
    return positive_direction == c->config.positive_moves_toward_end;
}

static void motion_axis_set_direction(Ctx_t *c, uint8_t positive_direction) {
    const uint8_t high = positive_direction == c->config.positive_direction_high;
    c->current_direction_positive = positive_direction;
    motion_axis_write_pin(c->board->output_port, c->board->direction_pin, high);
}

static void motion_axis_set_enabled(const Ctx_t *c, uint8_t enabled) {
    const uint8_t high = enabled != 0U ? c->config.enable_active_high : (uint8_t)!c->config.enable_active_high;
    motion_axis_write_pin(c->board->enable_port, c->board->enable_pin, high);
}

static void motion_axis_set_step_idle_gpio(const Ctx_t *c) {
    const uint8_t idle_high = (uint8_t)!c->config.step_active_high;
    motion_axis_write_pin(c->board->output_port, c->board->step_pin, idle_high);
    motion_axis_set_pin_mode(c->board->output_port, c->board->step_pin, MOTION_AXIS_GPIO_MODE_OUTPUT);
}

static void motion_axis_set_step_alternate_function(const Ctx_t *c) {
    GPIO_TypeDef *port = c->board->output_port;
    const uint8_t pin = c->board->step_pin;
    const uint32_t af_register = pin / 8U;
    const uint32_t af_shift = (pin % 8U) * 4U;
    port->AFR[af_register] =
        (port->AFR[af_register] & ~(0xFUL << af_shift)) | ((uint32_t)c->board->step_alternate_function << af_shift);
    motion_axis_set_pin_mode(port, pin, MOTION_AXIS_GPIO_MODE_ALTERNATE);
}

/* Only a move that pulled events resets its source; a jog must not cut a stream another move still needs. */
static void motion_axis_reset_stream(const Ctx_t *c) {
    if (c->uses_stream != 0U) {
        c->event_source.reset(c->event_source.context);
    }
}

static int motion_axis_event_source_is_valid(const MotionEventSource_t *source) {
    return source != NULL && source->begin != NULL && source->pulse_complete != NULL && source->reset != NULL;
}

static void motion_axis_stop(Ctx_t *c, MotionAxisHardwareStatus_t final_status) {
    TIM_TypeDef *timer = c->board->timer;
    if (c->jog_mode != 0U && c->jog_total_steps >= c->jog_steps_remaining) {
        c->last_jog_steps_completed = c->jog_total_steps - c->jog_steps_remaining;
    }
    timer->CR1 &= ~TIM_CR1_CEN;
    timer->DIER = 0U;
    timer->CCER &= ~c->step_output_enable;
    motion_axis_set_step_idle_gpio(c);
    motion_axis_set_enabled(c, 1U);
    c->jog_mode = 0U;
    c->jog_steps_remaining = 0U;
    c->jog_total_steps = 0U;
    c->jog_initial_step_rate_hz = 0U;
    c->jog_target_step_rate_hz = 0U;
    c->jog_acceleration_steps_per_s2 = 0U;
    c->stop_requested = 0U;
    c->pending_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    c->start_limit_release_stop_armed = 0U;
    c->start_limit_hook = NULL;
    c->limit_deceleration_requested = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    c->limit_deceleration_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (motion_axis_status_is_fault(c->status) != 0) {
        final_status = c->status;
    } else if (final_status == MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING) {
        ++c->timing_faults;
    }
    c->status = final_status;
}

static uint64_t motion_axis_next_jog_interval_ns(Ctx_t *c) {
    uint64_t interval_ns = c->jog_interval_base_ns;
    if (c->jog_interval_remainder_ns != 0U) {
        c->jog_interval_remainder_accumulator += c->jog_interval_remainder_ns;
        if (c->jog_interval_remainder_accumulator >= c->jog_step_rate_hz) {
            c->jog_interval_remainder_accumulator -= c->jog_step_rate_hz;
            ++interval_ns;
        }
    }
    return interval_ns;
}

/* One VSQRT.F64 instead of a 64-bit bit-by-bit loop. A double holds the square exactly and its rounded root
 * truncates to the integer root for rates below 2^26 Hz, far above any step rate. */
static uint32_t motion_axis_rate_from_square(uint64_t rate_squared) { return (uint32_t)sqrt((double)rate_squared); }

/* Called on every step, so the constant-rate stretch of a move skips the square root. */
static uint32_t motion_axis_jog_profile_rate_hz(const Ctx_t *c) {
    const uint32_t initial_rate_hz = c->jog_initial_step_rate_hz;
    const uint32_t target_rate_hz = c->jog_target_step_rate_hz;
    const uint32_t acceleration_steps_per_s2 = c->jog_acceleration_steps_per_s2;
    if (acceleration_steps_per_s2 == 0U || initial_rate_hz == target_rate_hz) {
        return target_rate_hz;
    }

    const uint32_t completed_steps = c->jog_total_steps - c->jog_steps_remaining;
    const uint32_t remaining_after_next_step = c->jog_steps_remaining > 0U ? c->jog_steps_remaining - 1U : 0U;
    const uint64_t initial_rate_squared = (uint64_t)initial_rate_hz * initial_rate_hz;
    const uint64_t target_rate_squared = (uint64_t)target_rate_hz * target_rate_hz;
    uint64_t acceleration_rate_squared = initial_rate_squared + 2ULL * acceleration_steps_per_s2 * completed_steps;
    uint64_t deceleration_rate_squared =
        initial_rate_squared + 2ULL * acceleration_steps_per_s2 * remaining_after_next_step;
    if (acceleration_rate_squared >= target_rate_squared && deceleration_rate_squared >= target_rate_squared) {
        return target_rate_hz;
    }

    if (acceleration_rate_squared > target_rate_squared) {
        acceleration_rate_squared = target_rate_squared;
    }
    if (deceleration_rate_squared > target_rate_squared) {
        deceleration_rate_squared = target_rate_squared;
    }

    const uint64_t rate_squared =
        acceleration_rate_squared < deceleration_rate_squared ? acceleration_rate_squared : deceleration_rate_squared;
    uint32_t rate_hz = motion_axis_rate_from_square(rate_squared);
    if (rate_hz < initial_rate_hz) {
        rate_hz = initial_rate_hz;
    }
    return rate_hz;
}

static void motion_axis_update_jog_step_rate(Ctx_t *c) {
    const uint32_t rate_hz = motion_axis_jog_profile_rate_hz(c);
    if (rate_hz == 0U || rate_hz == c->jog_step_rate_hz) {
        return;
    }

    c->jog_step_rate_hz = rate_hz;
    c->jog_interval_base_ns = MOTION_AXIS_NANOSECONDS_PER_SECOND_U32 / rate_hz;
    c->jog_interval_remainder_ns = MOTION_AXIS_NANOSECONDS_PER_SECOND_U32 % rate_hz;
    c->jog_interval_remainder_accumulator = 0U;
}

static uint64_t motion_axis_next_profiled_jog_interval_ns(Ctx_t *c) {
    motion_axis_update_jog_step_rate(c);
    return motion_axis_next_jog_interval_ns(c);
}

static uint64_t motion_axis_ceil_div_u64(uint64_t numerator, uint64_t denominator) {
    return numerator / denominator + (numerator % denominator != 0U ? 1U : 0U);
}

static void motion_axis_schedule_delay(Ctx_t *c, uint64_t total_ticks) {
    TIM_TypeDef *timer = c->board->timer;
    const uint64_t chunk_ticks =
        total_ticks > MOTION_AXIS_MAX_PERIOD_TICKS ? MOTION_AXIS_MAX_PERIOD_TICKS : total_ticks;
    c->delay_remaining_ticks = total_ticks - chunk_ticks;
    timer->CNT = 0U;
    timer->ARR = (uint32_t)(chunk_ticks - 1U);
    timer->DIER = TIM_DIER_UIE;
}

static int motion_axis_interval_to_ticks(Ctx_t *c, uint64_t interval_ns, uint64_t *ticks) {
    if (ticks == NULL || interval_ns == 0U || interval_ns > c->max_interval_ns ||
        interval_ns < (uint64_t)c->config.step_pulse_width_ns + c->config.step_low_time_ns ||
        c->timer_ticks_per_ns_q32 == 0U) {
        return 0;
    }

    /* Q32 conversion keeps division out of the timer ISR and spreads fractional ticks across events. */
    const uint64_t numerator =
        (uint64_t)(uint32_t)interval_ns * c->timer_ticks_per_ns_q32 + c->conversion_remainder_q32;
    uint64_t converted_ticks = numerator >> 32U;
    uint32_t conversion_remainder_q32 = (uint32_t)numerator;
    const uint64_t minimum_ticks = (uint64_t)c->pulse_ticks + c->low_time_ticks;
    if (converted_ticks < minimum_ticks) {
        converted_ticks = minimum_ticks;
        conversion_remainder_q32 = 0U;
    }
    c->conversion_remainder_q32 = conversion_remainder_q32;
    if (converted_ticks == 0U) {
        return 0;
    }
    *ticks = converted_ticks;
    return 1;
}

static int motion_axis_select_timer_scale(Ctx_t *c, uint64_t first_interval_ns) {
    const uint64_t clock_hz = c->timer_clock_hz;
    if (clock_hz == 0U || first_interval_ns > UINT32_MAX || first_interval_ns > UINT64_MAX / clock_hz ||
        c->config.step_pulse_width_ns > UINT64_MAX / clock_hz || c->config.step_low_time_ns > UINT64_MAX / clock_hz) {
        return 0;
    }

    const uint64_t raw_interval_cycles =
        motion_axis_ceil_div_u64(first_interval_ns * clock_hz, MOTION_AXIS_NANOSECONDS_PER_SECOND);
    const uint64_t raw_pulse_cycles = motion_axis_ceil_div_u64(
        (uint64_t)c->config.step_pulse_width_ns * clock_hz,
        MOTION_AXIS_NANOSECONDS_PER_SECOND
    );
    const uint64_t raw_low_time_cycles =
        motion_axis_ceil_div_u64((uint64_t)c->config.step_low_time_ns * clock_hz, MOTION_AXIS_NANOSECONDS_PER_SECOND);
    if (raw_pulse_cycles < 2U || raw_low_time_cycles < 2U) {
        return 0;
    }

    uint64_t divider = motion_axis_ceil_div_u64(raw_interval_cycles, MOTION_AXIS_MAX_PERIOD_TICKS);
    uint64_t max_divider = raw_pulse_cycles / 2U;
    const uint64_t max_low_time_divider = raw_low_time_cycles / 2U;
    if (max_divider > max_low_time_divider) {
        max_divider = max_low_time_divider;
    }
    if (divider == 0U) {
        divider = 1U;
    }
    if (max_divider > 65536U) {
        max_divider = 65536U;
    }
    if (max_divider == 0U) {
        return 0;
    }
    if (divider > max_divider) {
        divider = max_divider;
    }

    uint64_t timer_tick_denominator = 0U;
    uint64_t pulse_ticks = 0U;
    uint64_t low_time_ticks = 0U;
    uint64_t first_interval_ticks = 0U;
    for (;;) {
        timer_tick_denominator = MOTION_AXIS_NANOSECONDS_PER_SECOND * divider;
        pulse_ticks =
            motion_axis_ceil_div_u64((uint64_t)c->config.step_pulse_width_ns * clock_hz, timer_tick_denominator);
        low_time_ticks =
            motion_axis_ceil_div_u64((uint64_t)c->config.step_low_time_ns * clock_hz, timer_tick_denominator);
        first_interval_ticks = motion_axis_ceil_div_u64(first_interval_ns * clock_hz, timer_tick_denominator);
        if (first_interval_ticks >= pulse_ticks + low_time_ticks || divider == max_divider) {
            break;
        }
        ++divider;
    }
    const uint64_t direction_setup_ticks =
        motion_axis_ceil_div_u64((uint64_t)c->config.direction_setup_ns * clock_hz, timer_tick_denominator);
    const uint64_t enable_direction_lead_ticks =
        motion_axis_ceil_div_u64((uint64_t)c->config.enable_setup_ns * clock_hz, timer_tick_denominator);
    const uint64_t q32_numerator = (clock_hz << 32U) + (timer_tick_denominator / 2U);
    const uint64_t timer_ticks_per_ns_q32 = q32_numerator / timer_tick_denominator;

    if (pulse_ticks == 0U || low_time_ticks == 0U || first_interval_ticks < pulse_ticks + low_time_ticks ||
        timer_ticks_per_ns_q32 == 0U || timer_ticks_per_ns_q32 > UINT32_MAX) {
        return 0;
    }

    c->prescaler_divider = (uint32_t)divider;
    c->pulse_ticks = (uint32_t)pulse_ticks;
    c->low_time_ticks = (uint32_t)low_time_ticks;
    c->timer_ticks_per_ns_q32 = (uint32_t)timer_ticks_per_ns_q32;
    c->direction_setup_ticks = (uint32_t)direction_setup_ticks;
    c->enable_direction_lead_ticks = (uint32_t)enable_direction_lead_ticks;
    c->max_interval_ns = UINT32_MAX;
    c->conversion_remainder_q32 = UINT32_MAX;
    return 1;
}

/* Returns the limit switch that blocks the event direction, or NONE when the move may continue. */
static MotionAxisHardwareStopReason_t
motion_axis_limit_blocks_direction(const Ctx_t *c, const MotionExecutionEvent_t *event) {
    const uint8_t toward_end = motion_axis_direction_toward_end(c, motion_axis_direction_is_positive(c, event));
    if (toward_end == 0U && motion_axis_start_limit_active(c) != 0U) {
        return MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    }
    if (toward_end != 0U && motion_axis_end_limit_active(c) != 0U) {
        return MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    }
    return MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
}

static uint8_t motion_axis_reason_is_limit(MotionAxisHardwareStopReason_t reason) {
    return reason == MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT ||
           reason == MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
}

/* Records the switch point once per move: the position first, then the armed start-switch hook, which may reset
 * it. */
static void motion_axis_latch_limit(Ctx_t *c, MotionAxisHardwareStopReason_t reason) {
    if (c->limit_latched == 0U) {
        c->limit_latched = 1U;
        c->limit_trigger_position = c->config.position_reader != NULL ? c->config.position_reader() : 0;
    }
    const MotionAxisHardwareStartLimitHook_t hook = c->start_limit_hook;
    c->start_limit_hook = NULL;
    if (reason == MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT && hook != NULL) {
        hook();
    }
}

/* Steps needed to brake the current profiled move down to its start rate; 0 means it can stop at once. */
static uint32_t motion_axis_limit_deceleration_steps(const Ctx_t *c) {
    const uint64_t deceleration = c->config.limit_deceleration_steps_per_s2;
    const uint64_t rate_hz = c->jog_step_rate_hz;
    const uint64_t initial_rate_hz = c->jog_initial_step_rate_hz;
    if (c->jog_mode == 0U || deceleration == 0U || rate_hz <= initial_rate_hz) {
        return 0U;
    }
    return (uint32_t)((rate_hz * rate_hz - initial_rate_hz * initial_rate_hz) / (2U * deceleration));
}

/* Timer interrupt only. Turns a switch hit into a deceleration; returns 0 when the move must stop at once. */
static uint8_t motion_axis_begin_limit_deceleration(Ctx_t *c, MotionAxisHardwareStopReason_t reason) {
    if (c->limit_deceleration_reason != MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        return 1U;
    }
    const uint32_t steps = motion_axis_limit_deceleration_steps(c);
    if (steps == 0U) {
        return 0U;
    }
    motion_axis_latch_limit(c, reason);
    c->limit_deceleration_reason = reason;
    /* The profile brakes over the remaining steps, so shortening them to the braking distance is the whole ramp. */
    c->jog_acceleration_steps_per_s2 = c->config.limit_deceleration_steps_per_s2;
    if (steps < c->jog_steps_remaining) {
        /* The move gets shorter, so the completed-step count stays right. */
        c->jog_total_steps -= c->jog_steps_remaining - steps;
        c->jog_steps_remaining = steps;
    }
    return 1U;
}

static void motion_axis_stop_with_reason(Ctx_t *c, MotionAxisHardwareStopReason_t reason) {
    if (motion_axis_reason_is_limit(reason) != 0U) {
        motion_axis_latch_limit(c, reason);
    }
    c->stop_reason = reason;
    motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_STOPPED);
    motion_axis_reset_stream(c);
}

static void motion_axis_configure_output_pin(GPIO_TypeDef *port, uint8_t pin, uint8_t high) {
    const uint32_t shift = (uint32_t)pin * 2U;
    motion_axis_write_pin(port, pin, high);
    port->OTYPER &= ~motion_axis_pin_mask(pin);
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << shift)) | (MOTION_AXIS_GPIO_SPEED_VERY_HIGH << shift);
    port->PUPDR &= ~(3UL << shift);
    motion_axis_set_pin_mode(port, pin, MOTION_AXIS_GPIO_MODE_OUTPUT);
}

static void motion_axis_configure_limit_pin(GPIO_TypeDef *port, uint8_t pin, MotionAxisHardwareLimitPull_t pull) {
    const uint32_t shift = (uint32_t)pin * 2U;
    motion_axis_set_pin_mode(port, pin, 0U);
    port->PUPDR = (port->PUPDR & ~(3UL << shift)) | ((uint32_t)pull << shift);
}

static void motion_axis_configure_gpio(const Ctx_t *c) {
    const MotionAxisBoard_t *board = c->board;
    motion_axis_configure_output_pin(board->output_port, board->step_pin, (uint8_t)!c->config.step_active_high);
    motion_axis_configure_output_pin(board->output_port, board->direction_pin, c->config.positive_direction_high);
    motion_axis_configure_output_pin(board->enable_port, board->enable_pin, (uint8_t)!c->config.enable_active_high);
    motion_axis_configure_limit_pin(board->start_limit_port, board->start_limit_pin, c->config.limit_pull);
    motion_axis_configure_limit_pin(board->end_limit_port, board->end_limit_pin, c->config.limit_pull);
}

static void motion_axis_route_exti_line(uint8_t line, uint8_t port_index) {
    const uint32_t shift = (line % 4U) * 4U;
    SYSCFG->EXTICR[line / 4U] = (SYSCFG->EXTICR[line / 4U] & ~(0xFUL << shift)) | ((uint32_t)port_index << shift);
}

static void motion_axis_enable_irq(IRQn_Type irq, uint32_t priority) {
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, priority);
    NVIC_EnableIRQ(irq);
}

static void motion_axis_configure_exti(const Ctx_t *c) {
    const MotionAxisBoard_t *board = c->board;
    const uint32_t start_line = motion_axis_pin_mask(board->start_limit_pin);
    const uint32_t end_line = motion_axis_pin_mask(board->end_limit_pin);
    const uint32_t lines = start_line | end_line;
    motion_axis_route_exti_line(board->start_limit_pin, board->start_limit_port_index);
    motion_axis_route_exti_line(board->end_limit_pin, board->end_limit_port_index);
    /* The start switch is watched on both edges: homing also needs the moment the switch is released. */
    EXTI->RTSR1 = (EXTI->RTSR1 & ~lines) | start_line | (c->config.end_limit_active_high != 0U ? end_line : 0U);
    EXTI->FTSR1 = (EXTI->FTSR1 & ~lines) | start_line | (c->config.end_limit_active_high != 0U ? 0U : end_line);
    EXTI->PR1 = lines;
    EXTI->IMR1 |= lines;
    motion_axis_enable_irq(board->start_limit_irq, MOTION_AXIS_LIMIT_IRQ_PRIORITY);
    motion_axis_enable_irq(board->end_limit_irq, MOTION_AXIS_LIMIT_IRQ_PRIORITY);
}

static void motion_axis_configure_timer(const Ctx_t *c) {
    const MotionAxisBoard_t *board = c->board;
    TIM_TypeDef *timer = board->timer;
    const uint32_t compare_shift = ((uint32_t)(board->channel - 1U) % 2U) * 8U;
    const uint32_t pwm_mode_1 = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) << compare_shift;
    timer->CR1 = 0U;
    timer->CR2 = 0U;
    timer->SMCR = 0U;
    timer->DIER = 0U;
    if (board->channel <= 2U) {
        timer->CCMR1 = pwm_mode_1;
    } else {
        timer->CCMR2 = pwm_mode_1;
    }
    timer->CCER = c->config.step_active_high != 0U ? 0U : c->step_output_polarity;
    timer->PSC = 0U;
    timer->ARR = 0xFFFFU;
    *c->step_compare = 1U;
    timer->CNT = 0U;
    if (board->advanced_timer != 0U) {
        /* No repetition, no dead time, no break: MOE only lets the channel drive its pin. */
        timer->RCR = 0U;
        timer->BDTR = TIM_BDTR_MOE;
    }
    timer->EGR = TIM_EGR_UG;
    timer->SR = 0U;
    for (uint32_t index = 0U; index < board->timer_irq_count; ++index) {
        motion_axis_enable_irq(board->timer_irq[index], MOTION_AXIS_TIMER_IRQ_PRIORITY);
    }
}

static volatile uint32_t *motion_axis_compare_register(TIM_TypeDef *timer, uint8_t channel) {
    switch (channel) {
    case 1U:
        return &timer->CCR1;
    case 2U:
        return &timer->CCR2;
    case 3U:
        return &timer->CCR3;
    default:
        return &timer->CCR4;
    }
}

/* The step interrupts time themselves with the DWT cycle counter. Cortex-M7 keeps DWT locked until LAR gets the key;
 * enabling it again for another axis changes nothing. */
static void motion_axis_enable_cycle_counter(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = MOTION_AXIS_DWT_UNLOCK_KEY;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void motion_axis_clear_timer_irqs(const Ctx_t *c) {
    for (uint32_t index = 0U; index < c->board->timer_irq_count; ++index) {
        NVIC_ClearPendingIRQ(c->board->timer_irq[index]);
    }
}

MotionAxisHardwareStatus_t motion_axis_hardware_init(MotionAxis_t axis, const MotionAxisHardwareConfig_t *config) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    }
    if (motion_axis_status_is_active(c->status) != 0) {
        return c->status;
    }
    if (!motion_axis_config_is_valid(config)) {
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG;
        return c->status;
    }

    const MotionAxisBoard_t *board = c->board;
    const uint32_t channel_shift = ((uint32_t)board->channel - 1U) * 4U;
    const MotionEventSource_t event_source = c->event_source;
    *c = (Ctx_t){0};
    c->board = board;
    c->event_source = event_source;
    c->min_slack_timer_clocks = MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE;
    c->step_compare = motion_axis_compare_register(board->timer, board->channel);
    c->step_compare_flag = TIM_SR_CC1IF << (board->channel - 1U);
    c->step_compare_interrupt = TIM_DIER_CC1IE << (board->channel - 1U);
    c->step_output_enable = TIM_CCER_CC1E << channel_shift;
    c->step_output_polarity = TIM_CCER_CC1P << channel_shift;
    c->config = *config;
    c->timer_clock_hz = motion_axis_timer_clock_hz(board);
    if (c->timer_clock_hz == 0U) {
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_CLOCK;
        return c->status;
    }

    RCC->AHB4ENR |= board->output_port_clock_bit | board->enable_port_clock_bit | board->start_limit_port_clock_bit |
                    board->end_limit_port_clock_bit;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    *board->timer_clock_enable |= board->timer_clock_enable_bit;
    (void)RCC->AHB4ENR;
    __DSB();

    motion_axis_enable_cycle_counter();
    motion_axis_configure_gpio(c);
    motion_axis_configure_exti(c);
    motion_axis_configure_timer(c);
    c->status = MOTION_AXIS_HARDWARE_STATUS_READY;
    return c->status;
}

static MotionAxisHardwareStatus_t motion_axis_start_event(Ctx_t *c, const MotionExecutionEvent_t *first_event) {
    TIM_TypeDef *timer = c->board->timer;
    if (motion_axis_status_can_start(c->status) == 0) {
        return c->status;
    }
    c->stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (first_event == NULL) {
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT;
        return c->status;
    }
    if (motion_axis_event_is_valid(c, first_event) == 0U) {
        motion_axis_reset_stream(c);
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT;
        return c->status;
    }
    const MotionAxisHardwareStopReason_t blocking_limit = motion_axis_limit_blocks_direction(c, first_event);
    if (blocking_limit != MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        /* Already on the switch when the move starts: this is the switch point. */
        c->limit_latched = 0U;
        motion_axis_latch_limit(c, blocking_limit);
        motion_axis_reset_stream(c);
        c->stop_reason = blocking_limit;
        c->status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
        return c->status;
    }
    if (motion_axis_select_timer_scale(c, first_event->interval_ns) == 0) {
        motion_axis_reset_stream(c);
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING;
        return c->status;
    }

    uint64_t first_interval_ticks = 0U;
    if (motion_axis_interval_to_ticks(c, first_event->interval_ns, &first_interval_ticks) == 0) {
        motion_axis_reset_stream(c);
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING;
        return c->status;
    }

    c->limit_latched = 0U;
    c->limit_deceleration_requested = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    c->limit_deceleration_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    c->current_event = *first_event;
    c->current_direction_positive = motion_axis_direction_is_positive(c, first_event);
    c->first_interval_ticks = first_interval_ticks;
    c->startup_phase = MOTION_AXIS_STARTUP_ENABLE_LEAD;
    motion_axis_set_enabled(c, 1U);

    timer->CR1 &= ~TIM_CR1_CEN;
    timer->PSC = c->prescaler_divider - 1U;
    *c->step_compare = c->pulse_ticks;
    timer->CCER &= ~c->step_output_enable;
    timer->DIER = 0U;
    timer->EGR = TIM_EGR_UG;
    timer->SR = 0U;
    motion_axis_clear_timer_irqs(c);
    motion_axis_schedule_delay(c, c->enable_direction_lead_ticks);
    c->status = MOTION_AXIS_HARDWARE_STATUS_STARTING;
    __DMB();
    timer->CR1 |= TIM_CR1_CEN;
    return c->status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_set_event_source(MotionAxis_t axis, const MotionEventSource_t *source) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    }
    if (motion_axis_status_is_active(c->status) != 0 || !motion_axis_event_source_is_valid(source)) {
        return c->status;
    }
    c->event_source = *source;
    return c->status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_start(MotionAxis_t axis) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    }
    if (motion_axis_status_can_start(c->status) == 0) {
        return c->status;
    }

    if (!motion_axis_event_source_is_valid(&c->event_source)) {
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT;
        return c->status;
    }

    c->jog_mode = 0U;
    c->uses_stream = 1U;
    c->event_source.reset(c->event_source.context);
    MotionExecutionEvent_t first_event = {0};
    const MotionEventSourceStatus_t source_status = c->event_source.begin(c->event_source.context, &first_event);
    if (source_status != MOTION_EVENT_SOURCE_STATUS_OK) {
        c->event_source.reset(c->event_source.context);
        c->status = source_status == MOTION_EVENT_SOURCE_STATUS_UNDERRUN ? MOTION_AXIS_HARDWARE_STATUS_FAULT_UNDERRUN
                                                                         : MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT;
        return c->status;
    }
    return motion_axis_start_event(c, &first_event);
}

static void motion_axis_clear_jog(Ctx_t *c) {
    c->jog_mode = 0U;
    c->jog_steps_remaining = 0U;
    c->jog_total_steps = 0U;
    c->jog_initial_step_rate_hz = 0U;
    c->jog_target_step_rate_hz = 0U;
    c->jog_acceleration_steps_per_s2 = 0U;
}

MotionAxisHardwareStatus_t motion_axis_hardware_start_jog_profile(
    MotionAxis_t axis,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2
) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    }
    if (motion_axis_status_can_start(c->status) == 0) {
        return c->status;
    }

    const uint64_t step_count = signed_steps < 0 ? (uint64_t)(-(int64_t)signed_steps) : (uint64_t)signed_steps;
    if (step_count == 0U || step_count > MOTION_AXIS_JOG_MAX_STEPS || initial_step_rate_hz == 0U ||
        target_step_rate_hz < initial_step_rate_hz ||
        (target_step_rate_hz > initial_step_rate_hz && acceleration_steps_per_s2 == 0U)) {
        c->status = MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT;
        return c->status;
    }

    /* A jog never touches the shared stream, so it may run while another axis runs the stream. */
    c->uses_stream = 0U;
    c->jog_mode = 1U;
    c->jog_steps_remaining = (uint32_t)step_count;
    c->jog_total_steps = (uint32_t)step_count;
    c->last_jog_steps_completed = 0U;
    c->jog_initial_step_rate_hz = initial_step_rate_hz;
    c->jog_target_step_rate_hz = target_step_rate_hz;
    c->jog_acceleration_steps_per_s2 = acceleration_steps_per_s2;
    c->jog_step_rate_hz = initial_step_rate_hz;
    c->jog_interval_base_ns = MOTION_AXIS_NANOSECONDS_PER_SECOND_U32 / initial_step_rate_hz;
    c->jog_interval_remainder_ns = MOTION_AXIS_NANOSECONDS_PER_SECOND_U32 % initial_step_rate_hz;
    c->jog_interval_remainder_accumulator = 0U;

    const uint8_t step_mask = motion_axis_step_mask(c);
    const MotionExecutionEvent_t first_event = {
        .interval_ns = motion_axis_next_profiled_jog_interval_ns(c),
        .sequence = 1U,
        .step_mask = step_mask,
        .positive_direction_mask = signed_steps > 0 ? step_mask : 0U,
        .last_in_block = 0U,
    };
    const MotionAxisHardwareStatus_t start_status = motion_axis_start_event(c, &first_event);
    if (start_status != MOTION_AXIS_HARDWARE_STATUS_STARTING) {
        motion_axis_clear_jog(c);
    }
    return start_status;
}

void motion_axis_hardware_request_stop(MotionAxis_t axis) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL || motion_axis_status_is_active(c->status) == 0) {
        return;
    }
    const uint32_t primask = motion_axis_enter_critical();
    if (c->stop_requested == 0U) {
        c->pending_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_REQUEST;
        c->stop_requested = 1U;
    }
    motion_axis_exit_critical(primask);
}

MotionAxisHardwareStopReason_t motion_axis_hardware_get_last_stop_reason(MotionAxis_t axis) {
    const Ctx_t *c = motion_axis_context(axis);
    return c != NULL ? c->stop_reason : MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
}

MotionAxisHardwareStatus_t motion_axis_hardware_clear_fault(MotionAxis_t axis) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    }
    const uint32_t primask = motion_axis_enter_critical();
    const MotionAxisHardwareStatus_t status = c->status;
    if (status == MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT || status == MOTION_AXIS_HARDWARE_STATUS_FAULT_UNDERRUN ||
        status == MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT || status == MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING) {
        /* A runtime fault already stopped the timer; force the idle outputs again before allowing a new start. */
        c->status = MOTION_AXIS_HARDWARE_STATUS_READY;
        motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_READY);
        c->startup_phase = MOTION_AXIS_STARTUP_NONE;
        c->delay_remaining_ticks = 0U;
        motion_axis_reset_stream(c);
    }
    const MotionAxisHardwareStatus_t result = c->status;
    motion_axis_exit_critical(primask);
    return result;
}

MotionAxisHardwareStatus_t motion_axis_hardware_ready_after_home(MotionAxis_t axis) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    }
    const uint32_t primask = motion_axis_enter_critical();
    if (c->status == MOTION_AXIS_HARDWARE_STATUS_STOPPED && motion_axis_start_limit_active(c) != 0U) {
        motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_READY);
        motion_axis_reset_stream(c);
    }
    const MotionAxisHardwareStatus_t result = c->status;
    motion_axis_exit_critical(primask);
    return result;
}

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) {
    const Ctx_t *c = motion_axis_context(axis);
    return c != NULL ? c->status : MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
}

uint8_t motion_axis_hardware_start_limit_is_active(MotionAxis_t axis) {
    const Ctx_t *c = motion_axis_context(axis);
    if (c == NULL || c->status == MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED) {
        return 0U;
    }
    return motion_axis_start_limit_active(c);
}

uint8_t motion_axis_hardware_end_limit_is_active(MotionAxis_t axis) {
    const Ctx_t *c = motion_axis_context(axis);
    if (c == NULL || c->status == MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED) {
        return 0U;
    }
    return motion_axis_end_limit_active(c);
}

uint32_t motion_axis_hardware_get_last_jog_steps_completed(MotionAxis_t axis) {
    const Ctx_t *c = motion_axis_context(axis);
    return c != NULL ? c->last_jog_steps_completed : 0U;
}

int32_t motion_axis_hardware_get_limit_trigger_position(MotionAxis_t axis) {
    const Ctx_t *c = motion_axis_context(axis);
    return c != NULL ? c->limit_trigger_position : 0;
}

static void motion_axis_set_release_stop(MotionAxis_t axis, uint8_t armed) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return;
    }
    const uint32_t primask = motion_axis_enter_critical();
    c->start_limit_release_stop_armed = armed;
    __DMB();
    motion_axis_exit_critical(primask);
}

void motion_axis_hardware_arm_start_limit_release_stop(MotionAxis_t axis) { motion_axis_set_release_stop(axis, 1U); }

void motion_axis_hardware_disarm_start_limit_release_stop(MotionAxis_t axis) { motion_axis_set_release_stop(axis, 0U); }

void motion_axis_hardware_arm_start_limit_hook(MotionAxis_t axis, MotionAxisHardwareStartLimitHook_t hook) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return;
    }
    const uint32_t primask = motion_axis_enter_critical();
    c->start_limit_hook = hook;
    __DMB();
    motion_axis_exit_critical(primask);
}

/* Update interrupt while STARTING or WAITING: walks the startup delays, then hands STEP to the timer channel. */
static void motion_axis_timer_update(Ctx_t *c) {
    TIM_TypeDef *timer = c->board->timer;
    timer->SR = ~(TIM_SR_UIF | c->step_compare_flag);
    if (c->delay_remaining_ticks != 0U) {
        timer->CR1 &= ~TIM_CR1_CEN;
        const uint32_t elapsed_ticks = timer->CNT;
        if (c->delay_remaining_ticks > elapsed_ticks) {
            const uint32_t primask = motion_axis_enter_critical();
            if (c->status != MOTION_AXIS_HARDWARE_STATUS_STARTING && c->status != MOTION_AXIS_HARDWARE_STATUS_WAITING) {
                motion_axis_exit_critical(primask);
                return;
            }
            c->delay_remaining_ticks -= elapsed_ticks;
            motion_axis_schedule_delay(c, c->delay_remaining_ticks);
            timer->CR1 |= TIM_CR1_CEN;
            motion_axis_exit_critical(primask);
            return;
        }
        c->delay_remaining_ticks = 0U;
    }

    const uint32_t primask = motion_axis_enter_critical();
    if (c->status != MOTION_AXIS_HARDWARE_STATUS_STARTING && c->status != MOTION_AXIS_HARDWARE_STATUS_WAITING) {
        motion_axis_exit_critical(primask);
        return;
    }
    const MotionAxisHardwareStopReason_t blocking_limit = motion_axis_limit_blocks_direction(c, &c->current_event);
    if (blocking_limit != MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        motion_axis_stop_with_reason(c, blocking_limit);
        motion_axis_exit_critical(primask);
        return;
    }

    if (c->status == MOTION_AXIS_HARDWARE_STATUS_STARTING) {
        switch (c->startup_phase) {
        case MOTION_AXIS_STARTUP_ENABLE_LEAD:
            motion_axis_set_direction(c, motion_axis_direction_is_positive(c, &c->current_event));
            c->startup_phase = MOTION_AXIS_STARTUP_DIRECTION_SETUP;
            motion_axis_schedule_delay(c, c->direction_setup_ticks);
            timer->CR1 |= TIM_CR1_CEN;
            motion_axis_exit_critical(primask);
            return;
        case MOTION_AXIS_STARTUP_DIRECTION_SETUP:
            c->startup_phase = MOTION_AXIS_STARTUP_FIRST_INTERVAL;
            motion_axis_schedule_delay(c, c->first_interval_ticks);
            timer->CR1 |= TIM_CR1_CEN;
            motion_axis_exit_critical(primask);
            return;
        case MOTION_AXIS_STARTUP_FIRST_INTERVAL:
            c->startup_phase = MOTION_AXIS_STARTUP_NONE;
            break;
        case MOTION_AXIS_STARTUP_NONE:
        default:
            motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT);
            motion_axis_reset_stream(c);
            motion_axis_exit_critical(primask);
            return;
        }
    }

    motion_axis_set_step_alternate_function(c);
    timer->CNT = 0U;
    *c->step_compare = c->pulse_ticks;
    timer->ARR = 0xFFFFU;
    timer->CCER = (timer->CCER & ~(c->step_output_enable | c->step_output_polarity)) |
                  (c->config.step_active_high != 0U ? 0U : c->step_output_polarity) | c->step_output_enable;
    timer->DIER = c->step_compare_interrupt;
    c->status = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    motion_axis_exit_critical(primask);
}

/* Time left until the next step is due, kept as the minimum since the last peak reset. */
static void motion_axis_record_slack(Ctx_t *c, uint64_t next_interval_ticks, uint32_t elapsed_ticks) {
    const uint64_t slack_ticks = next_interval_ticks > elapsed_ticks ? next_interval_ticks - elapsed_ticks : 0U;
    const uint64_t slack_clocks = slack_ticks * c->prescaler_divider;
    if (slack_clocks < c->min_slack_timer_clocks) {
        c->min_slack_timer_clocks = (uint32_t)slack_clocks;
    }
}

/* Compare interrupt while RUNNING: one STEP pulse has ended, so the next interval is loaded. */
static void motion_axis_timer_step(Ctx_t *c) {
    TIM_TypeDef *timer = c->board->timer;
    timer->SR = ~(c->step_compare_flag | TIM_SR_UIF);
    MotionExecutionEvent_t next_event = {0};
    MotionEventSourceStatus_t source_status;
    if (c->jog_mode != 0U) {
        if (c->jog_steps_remaining <= 1U) {
            c->jog_steps_remaining = 0U;
            source_status = MOTION_EVENT_SOURCE_STATUS_COMPLETE;
        } else {
            --c->jog_steps_remaining;
            next_event = c->current_event;
            next_event.interval_ns = motion_axis_next_profiled_jog_interval_ns(c);
            source_status = MOTION_EVENT_SOURCE_STATUS_OK;
        }
    } else {
        source_status = c->event_source.pulse_complete(c->event_source.context, &next_event);
    }
    if (source_status == MOTION_EVENT_SOURCE_STATUS_COMPLETE) {
        const uint32_t primask = motion_axis_enter_critical();
        if (c->status != MOTION_AXIS_HARDWARE_STATUS_RUNNING) {
            motion_axis_exit_critical(primask);
            return;
        }
        MotionAxisHardwareStopReason_t blocking_limit = c->limit_deceleration_reason;
        if (blocking_limit == MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
            blocking_limit = motion_axis_limit_blocks_direction(c, &c->current_event);
        }
        if (blocking_limit != MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
            motion_axis_stop_with_reason(c, blocking_limit);
            motion_axis_exit_critical(primask);
            return;
        }
        motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_COMPLETE);
        motion_axis_exit_critical(primask);
        return;
    }
    if (source_status != MOTION_EVENT_SOURCE_STATUS_OK || motion_axis_event_is_valid(c, &next_event) == 0U) {
        motion_axis_stop(
            c,
            source_status == MOTION_EVENT_SOURCE_STATUS_UNDERRUN ? MOTION_AXIS_HARDWARE_STATUS_FAULT_UNDERRUN
                                                                 : MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT
        );
        return;
    }

    uint64_t next_interval_ticks = 0U;
    if (motion_axis_interval_to_ticks(c, next_event.interval_ns, &next_interval_ticks) == 0) {
        motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
        return;
    }
    if (next_interval_ticks < (uint64_t)c->pulse_ticks + c->low_time_ticks) {
        motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
        return;
    }
    const uint32_t primask = motion_axis_enter_critical();
    if (c->status != MOTION_AXIS_HARDWARE_STATUS_RUNNING) {
        motion_axis_exit_critical(primask);
        return;
    }
    MotionAxisHardwareStopReason_t blocking_limit = motion_axis_limit_blocks_direction(c, &next_event);
    if (blocking_limit == MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        /* The switch may already be off again; its interrupt still counts. */
        blocking_limit = c->limit_deceleration_requested;
    }
    if (blocking_limit != MOTION_AXIS_HARDWARE_STOP_REASON_NONE &&
        motion_axis_begin_limit_deceleration(c, blocking_limit) == 0U) {
        motion_axis_stop_with_reason(c, blocking_limit);
        motion_axis_exit_critical(primask);
        return;
    }
    const uint8_t next_direction_positive = motion_axis_direction_is_positive(c, &next_event);
    const uint8_t direction_changed = next_direction_positive != c->current_direction_positive;
    if (direction_changed != 0U) {
        motion_axis_set_direction(c, next_direction_positive);
    }

    const uint32_t elapsed_ticks = timer->CNT;
    motion_axis_record_slack(c, next_interval_ticks, elapsed_ticks);
    if (next_interval_ticks <= (uint64_t)elapsed_ticks + MOTION_AXIS_ISR_GUARD_TICKS +
                                   (direction_changed != 0U ? c->direction_setup_ticks : 0U)) {
        motion_axis_stop(c, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
        motion_axis_exit_critical(primask);
        return;
    }

    c->current_event = next_event;
    if (next_interval_ticks <= MOTION_AXIS_MAX_PERIOD_TICKS) {
        timer->ARR = (uint32_t)next_interval_ticks - 1U;
    } else {
        timer->CCER &= ~c->step_output_enable;
        motion_axis_set_step_idle_gpio(c);
        timer->ARR = 0xFFFFU;
        c->delay_remaining_ticks = next_interval_ticks - MOTION_AXIS_MAX_PERIOD_TICKS;
        c->status = MOTION_AXIS_HARDWARE_STATUS_WAITING;
        timer->DIER = TIM_DIER_UIE;
    }
    motion_axis_exit_critical(primask);
}

static void motion_axis_timer_irq_dispatch(Ctx_t *c) {
    const uint32_t status = c->board->timer->SR;
    if (c->stop_requested != 0U && motion_axis_status_is_active(c->status) != 0) {
        if (c->jog_mode != 0U && c->status == MOTION_AXIS_HARDWARE_STATUS_RUNNING &&
            (status & c->step_compare_flag) != 0U && c->jog_steps_remaining != 0U) {
            --c->jog_steps_remaining;
        }
        motion_axis_stop_with_reason(c, c->pending_stop_reason);
        return;
    }
    if ((status & TIM_SR_UIF) != 0U &&
        (c->status == MOTION_AXIS_HARDWARE_STATUS_STARTING || c->status == MOTION_AXIS_HARDWARE_STATUS_WAITING)) {
        motion_axis_timer_update(c);
        return;
    }
    if ((status & c->step_compare_flag) != 0U && c->status == MOTION_AXIS_HARDWARE_STATUS_RUNNING) {
        motion_axis_timer_step(c);
    }
}

void motion_axis_hardware_timer_irq(MotionAxis_t axis) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return;
    }
    /* DWT counts core cycles; the difference stays right across a counter wrap. */
    const uint32_t start_cycles = DWT->CYCCNT;
    motion_axis_timer_irq_dispatch(c);
    const uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > c->isr_max_cycles) {
        c->isr_max_cycles = cycles;
    }
}

static uint32_t motion_axis_scale_to_ns(uint32_t count, uint32_t clock_hz) {
    if (clock_hz == 0U) {
        return 0U;
    }
    const uint64_t ns = (uint64_t)count * MOTION_AXIS_NANOSECONDS_PER_SECOND / clock_hz;
    return ns > UINT32_MAX ? UINT32_MAX : (uint32_t)ns;
}

void motion_axis_hardware_get_timing(MotionAxis_t axis, MotionAxisHardwareTiming_t *timing) {
    const Ctx_t *c = motion_axis_context(axis);
    if (c == NULL || timing == NULL) {
        return;
    }
    const uint32_t min_slack_clocks = c->min_slack_timer_clocks;
    *timing = (MotionAxisHardwareTiming_t){
        .status = (uint32_t)c->status,
        .timing_faults = c->timing_faults,
        .isr_max_ns = motion_axis_scale_to_ns(c->isr_max_cycles, SystemCoreClock),
        .min_slack_ns = min_slack_clocks == MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE
                            ? MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE
                            : motion_axis_scale_to_ns(min_slack_clocks, c->timer_clock_hz),
    };
}

void motion_axis_hardware_reset_timing_peaks(MotionAxis_t axis) {
    Ctx_t *c = motion_axis_context(axis);
    if (c == NULL) {
        return;
    }
    const uint32_t primask = motion_axis_enter_critical();
    c->isr_max_cycles = 0U;
    c->min_slack_timer_clocks = MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE;
    motion_axis_exit_critical(primask);
}

static void motion_axis_limit_irq_for(Ctx_t *c, uint32_t pending_lines) {
    if (motion_axis_status_is_active(c->status) == 0) {
        return;
    }

    const uint8_t toward_end = motion_axis_direction_toward_end(c, c->current_direction_positive);
    const uint8_t start_pending = (pending_lines & motion_axis_pin_mask(c->board->start_limit_pin)) != 0U;
    const uint8_t start_active = start_pending != 0U && motion_axis_start_limit_active(c) != 0U;
    if (start_pending != 0U && start_active == 0U && c->start_limit_release_stop_armed != 0U && toward_end != 0U &&
        c->stop_requested == 0U) {
        c->pending_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE;
        c->stop_requested = 1U;
        __DMB();
    }
    const uint8_t end_active =
        (pending_lines & motion_axis_pin_mask(c->board->end_limit_pin)) != 0U && motion_axis_end_limit_active(c) != 0U;

    MotionAxisHardwareStopReason_t reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    if (start_active != 0U && toward_end == 0U) {
        reason = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    } else if (end_active != 0U && toward_end != 0U) {
        reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    }
    if (reason == MOTION_AXIS_HARDWARE_STOP_REASON_NONE) {
        return;
    }
    /* A fast profiled move brakes: the switch point is recorded here and the timer interrupt starts the ramp on its
     * next step, at most one step later. Everything else stops at once. */
    if (c->status == MOTION_AXIS_HARDWARE_STATUS_RUNNING && motion_axis_limit_deceleration_steps(c) != 0U) {
        motion_axis_latch_limit(c, reason);
        c->limit_deceleration_requested = reason;
        __DMB();
        return;
    }
    motion_axis_stop_with_reason(c, reason);
}

void motion_axis_hardware_limit_irq(uint32_t pending_lines) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        Ctx_t *c = &motion_axis_contexts[axis];
        const uint32_t lines = pending_lines & motion_axis_limit_lines(c->board);
        if (lines != 0U) {
            motion_axis_limit_irq_for(c, lines);
        }
    }
}

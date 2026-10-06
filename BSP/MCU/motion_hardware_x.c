#include "motion_hardware_x.h"

#include "motion/execution_buffer.h"
#include "motion/step_executor.h"

#include "stm32h723xx.h"
#include "system_stm32h7xx.h"

#include <stddef.h>

#define MOTION_X_STEP_PIN (1UL << 6U)
#define MOTION_X_DIRECTION_PIN (1UL << 7U)
#define MOTION_X_ENABLE_PIN (1UL << 8U)
#define MOTION_X_START_LIMIT_PIN (1UL << 10U)
#define MOTION_X_END_LIMIT_PIN (1UL << 9U)
#define MOTION_X_LIMIT_MASK (MOTION_X_START_LIMIT_PIN | MOTION_X_END_LIMIT_PIN)
#define MOTION_X_STEP_AXIS_MASK 0x01U
#define MOTION_X_TIMER_IRQ_PRIORITY 5U
#define MOTION_X_LIMIT_IRQ_PRIORITY 4U
#define MOTION_X_NANOSECONDS_PER_SECOND 1000000000ULL
#define MOTION_X_MAX_PERIOD_TICKS 65536ULL
#define MOTION_X_ISR_GUARD_TICKS 256U
#define MOTION_X_JOG_MAX_STEPS 1000000U

typedef enum {
    MOTION_X_STARTUP_NONE = 0,
    MOTION_X_STARTUP_ENABLE_LEAD,
    MOTION_X_STARTUP_DIRECTION_SETUP,
    MOTION_X_STARTUP_FIRST_INTERVAL
} MotionHardwareXStartupPhase_t;

typedef struct {
    MotionHardwareXConfig_t config;
    MotionExecutionEvent_t current_event;
    /* X_JOG repeats this event in the timer ISR instead of queuing one event per step. */
    uint32_t jog_steps_remaining;
    uint32_t jog_total_steps;
    uint32_t jog_initial_step_rate_hz;
    uint32_t jog_target_step_rate_hz;
    uint32_t jog_acceleration_steps_per_s2;
    uint32_t jog_step_rate_hz;
    uint32_t jog_interval_remainder_ns;
    uint32_t jog_interval_remainder_accumulator;
    uint64_t jog_interval_base_ns;
    volatile MotionHardwareXStatus_t status;
    MotionHardwareXStartupPhase_t startup_phase;
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
    volatile uint8_t stop_requested;
    volatile MotionHardwareXStopReason_t pending_stop_reason;
    volatile MotionHardwareXStopReason_t stop_reason;
    volatile uint8_t start_limit_release_event_pending;
    volatile uint8_t start_limit_release_stop_armed;
} MotionHardwareXContext_t;

static MotionHardwareXContext_t motion_hardware_x_context;
extern uint32_t SystemCoreClock;

static int motion_hardware_x_is_boolean(uint8_t value) { return value <= 1U; }

static int motion_hardware_x_status_is_fault(MotionHardwareXStatus_t status) {
    return status >= MOTION_HARDWARE_X_STATUS_FAULT_LIMIT && status <= MOTION_HARDWARE_X_STATUS_FAULT_CLOCK;
}

static int motion_hardware_x_config_is_valid(const MotionHardwareXConfig_t *config) {
    return config != NULL && motion_hardware_x_is_boolean(config->step_active_high) &&
           motion_hardware_x_is_boolean(config->enable_active_high) && config->step_active_high == 1U &&
           motion_hardware_x_is_boolean(config->positive_direction_high) &&
           motion_hardware_x_is_boolean(config->positive_moves_toward_end) &&
           motion_hardware_x_is_boolean(config->start_limit_active_high) &&
           motion_hardware_x_is_boolean(config->end_limit_active_high) &&
           config->limit_pull >= MOTION_HARDWARE_X_LIMIT_PULL_NONE &&
           config->limit_pull <= MOTION_HARDWARE_X_LIMIT_PULL_DOWN &&
           config->step_pulse_width_ns >= MOTION_HARDWARE_X_DM556_MIN_STEP_PULSE_NS &&
           config->step_low_time_ns >= MOTION_HARDWARE_X_DM556_MIN_STEP_LOW_NS &&
           config->direction_setup_ns >= MOTION_HARDWARE_X_DM556_MIN_DIRECTION_SETUP_NS &&
           config->enable_setup_ns >= MOTION_HARDWARE_X_DM556_MIN_ENABLE_LEAD_NS &&
           config->direction_hold_ns <= config->step_pulse_width_ns &&
           config->enable_hold_ns <= config->step_pulse_width_ns;
}

static uint32_t motion_hardware_x_apb1_divider(void) {
    const uint32_t prescaler = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_Msk) >> RCC_D2CFGR_D2PPRE1_Pos;
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

static uint32_t motion_hardware_x_ahb_divider(void) {
    static const uint16_t ahb_dividers[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U, 2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
    const uint32_t prescaler = (RCC->D1CFGR & RCC_D1CFGR_HPRE_Msk) >> RCC_D1CFGR_HPRE_Pos;
    return ahb_dividers[prescaler];
}

static uint32_t motion_hardware_x_timer_clock_hz(void) {
    const uint32_t ahb_divider = motion_hardware_x_ahb_divider();
    const uint32_t apb1_divider = motion_hardware_x_apb1_divider();
    if (SystemCoreClock == 0U || ahb_divider == 0U || apb1_divider == 0U) {
        return 0U;
    }
    const uint32_t hclk = SystemCoreClock / ahb_divider;
    const uint32_t pclk1 = hclk / apb1_divider;
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U) {
        return apb1_divider <= 4U ? hclk : pclk1 * 4U;
    }
    return apb1_divider == 1U ? pclk1 : pclk1 * 2U;
}

static uint32_t motion_hardware_x_enter_critical(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void motion_hardware_x_exit_critical(uint32_t primask) {
    __DMB();
    __set_PRIMASK(primask);
}

static void motion_hardware_x_write_pin(GPIO_TypeDef *port, uint32_t pin, uint8_t high) {
    port->BSRR = high != 0U ? pin : pin << 16U;
}

static uint8_t motion_hardware_x_limit_is_active(uint32_t pin, uint8_t active_high) {
    const uint8_t high = (GPIOF->IDR & pin) != 0U;
    return high == (active_high != 0U);
}

static uint8_t motion_hardware_x_direction_is_positive(const MotionExecutionEvent_t *event) {
    return (event->positive_direction_mask & MOTION_X_STEP_AXIS_MASK) != 0U;
}

static uint8_t motion_hardware_x_direction_toward_end(uint8_t positive_direction) {
    return positive_direction == motion_hardware_x_context.config.positive_moves_toward_end;
}

static void motion_hardware_x_set_direction(uint8_t positive_direction) {
    const uint8_t high = positive_direction == motion_hardware_x_context.config.positive_direction_high;
    motion_hardware_x_context.current_direction_positive = positive_direction;
    motion_hardware_x_write_pin(GPIOB, MOTION_X_DIRECTION_PIN, high);
}

static void motion_hardware_x_set_enabled(uint8_t enabled) {
    const uint8_t high = enabled != 0U ? motion_hardware_x_context.config.enable_active_high
                                       : (uint8_t)!motion_hardware_x_context.config.enable_active_high;
    motion_hardware_x_write_pin(GPIOB, MOTION_X_ENABLE_PIN, high);
}

static void motion_hardware_x_set_step_idle_gpio(void) {
    const uint8_t idle_high = (uint8_t)!motion_hardware_x_context.config.step_active_high;
    motion_hardware_x_write_pin(GPIOB, MOTION_X_STEP_PIN, idle_high);
    GPIOB->MODER = (GPIOB->MODER & ~(3UL << (6U * 2U))) | (1UL << (6U * 2U));
}

static void motion_hardware_x_set_step_alternate_function(void) {
    const uint32_t af_shift = 6U * 4U;
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFUL << af_shift)) | (2UL << af_shift);
    GPIOB->MODER = (GPIOB->MODER & ~(3UL << 12U)) | (2UL << 12U);
}

static void motion_hardware_x_stop(MotionHardwareXStatus_t final_status) {
    if (motion_hardware_x_context.jog_mode != 0U &&
        motion_hardware_x_context.jog_total_steps >= motion_hardware_x_context.jog_steps_remaining) {
        motion_hardware_x_context.last_jog_steps_completed =
            motion_hardware_x_context.jog_total_steps - motion_hardware_x_context.jog_steps_remaining;
    }
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->DIER = 0U;
    TIM4->CCER &= ~TIM_CCER_CC1E;
    motion_hardware_x_set_step_idle_gpio();
    motion_hardware_x_set_enabled(1U);
    motion_hardware_x_context.jog_mode = 0U;
    motion_hardware_x_context.jog_steps_remaining = 0U;
    motion_hardware_x_context.jog_total_steps = 0U;
    motion_hardware_x_context.jog_initial_step_rate_hz = 0U;
    motion_hardware_x_context.jog_target_step_rate_hz = 0U;
    motion_hardware_x_context.jog_acceleration_steps_per_s2 = 0U;
    motion_hardware_x_context.stop_requested = 0U;
    motion_hardware_x_context.pending_stop_reason = MOTION_HARDWARE_X_STOP_REASON_NONE;
    motion_hardware_x_context.start_limit_release_stop_armed = 0U;
    if (motion_hardware_x_status_is_fault(motion_hardware_x_context.status) != 0) {
        final_status = motion_hardware_x_context.status;
    }
    motion_hardware_x_context.status = final_status;
}

static uint64_t motion_hardware_x_next_jog_interval_ns(void) {
    uint64_t interval_ns = motion_hardware_x_context.jog_interval_base_ns;
    if (motion_hardware_x_context.jog_interval_remainder_ns != 0U) {
        motion_hardware_x_context.jog_interval_remainder_accumulator +=
            motion_hardware_x_context.jog_interval_remainder_ns;
        if (motion_hardware_x_context.jog_interval_remainder_accumulator >=
            motion_hardware_x_context.jog_step_rate_hz) {
            motion_hardware_x_context.jog_interval_remainder_accumulator -= motion_hardware_x_context.jog_step_rate_hz;
            ++interval_ns;
        }
    }
    return interval_ns;
}

static uint64_t motion_hardware_x_integer_sqrt(uint64_t value) {
    uint64_t result = 0U;
    uint64_t bit = 1ULL << 62U;

    while (bit > value) {
        bit >>= 2U;
    }
    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1U) + bit;
        } else {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return result;
}

static uint32_t motion_hardware_x_jog_profile_rate_hz(void) {
    const uint32_t initial_rate_hz = motion_hardware_x_context.jog_initial_step_rate_hz;
    const uint32_t target_rate_hz = motion_hardware_x_context.jog_target_step_rate_hz;
    const uint32_t acceleration_steps_per_s2 = motion_hardware_x_context.jog_acceleration_steps_per_s2;
    if (acceleration_steps_per_s2 == 0U || initial_rate_hz == target_rate_hz) {
        return target_rate_hz;
    }

    const uint32_t completed_steps =
        motion_hardware_x_context.jog_total_steps - motion_hardware_x_context.jog_steps_remaining;
    const uint32_t remaining_after_next_step =
        motion_hardware_x_context.jog_steps_remaining > 0U ? motion_hardware_x_context.jog_steps_remaining - 1U : 0U;
    const uint64_t initial_rate_squared = (uint64_t)initial_rate_hz * initial_rate_hz;
    const uint64_t target_rate_squared = (uint64_t)target_rate_hz * target_rate_hz;
    uint64_t acceleration_rate_squared = initial_rate_squared + 2ULL * acceleration_steps_per_s2 * completed_steps;
    uint64_t deceleration_rate_squared =
        initial_rate_squared + 2ULL * acceleration_steps_per_s2 * remaining_after_next_step;

    if (acceleration_rate_squared > target_rate_squared) {
        acceleration_rate_squared = target_rate_squared;
    }
    if (deceleration_rate_squared > target_rate_squared) {
        deceleration_rate_squared = target_rate_squared;
    }

    const uint64_t rate_squared =
        acceleration_rate_squared < deceleration_rate_squared ? acceleration_rate_squared : deceleration_rate_squared;
    uint32_t rate_hz = (uint32_t)motion_hardware_x_integer_sqrt(rate_squared);
    if (rate_hz < initial_rate_hz) {
        rate_hz = initial_rate_hz;
    }
    return rate_hz;
}

static void motion_hardware_x_update_jog_step_rate(void) {
    const uint32_t rate_hz = motion_hardware_x_jog_profile_rate_hz();
    if (rate_hz == 0U || rate_hz == motion_hardware_x_context.jog_step_rate_hz) {
        return;
    }

    motion_hardware_x_context.jog_step_rate_hz = rate_hz;
    motion_hardware_x_context.jog_interval_base_ns = MOTION_X_NANOSECONDS_PER_SECOND / rate_hz;
    motion_hardware_x_context.jog_interval_remainder_ns = (uint32_t)(MOTION_X_NANOSECONDS_PER_SECOND % rate_hz);
    motion_hardware_x_context.jog_interval_remainder_accumulator = 0U;
}

static uint64_t motion_hardware_x_next_profiled_jog_interval_ns(void) {
    motion_hardware_x_update_jog_step_rate();
    return motion_hardware_x_next_jog_interval_ns();
}

static uint64_t motion_hardware_x_ceil_div_u64(uint64_t numerator, uint64_t denominator) {
    return numerator / denominator + (numerator % denominator != 0U ? 1U : 0U);
}

static void motion_hardware_x_schedule_delay(uint64_t total_ticks) {
    const uint64_t chunk_ticks = total_ticks > MOTION_X_MAX_PERIOD_TICKS ? MOTION_X_MAX_PERIOD_TICKS : total_ticks;
    motion_hardware_x_context.delay_remaining_ticks = total_ticks - chunk_ticks;
    TIM4->CNT = 0U;
    TIM4->ARR = (uint32_t)(chunk_ticks - 1U);
    TIM4->DIER = TIM_DIER_UIE;
}

static int motion_hardware_x_interval_to_ticks(uint64_t interval_ns, uint64_t *ticks) {
    if (ticks == NULL || interval_ns == 0U || interval_ns > motion_hardware_x_context.max_interval_ns ||
        interval_ns < (uint64_t)motion_hardware_x_context.config.step_pulse_width_ns +
                          motion_hardware_x_context.config.step_low_time_ns ||
        motion_hardware_x_context.timer_ticks_per_ns_q32 == 0U) {
        return 0;
    }

    /* Q32 conversion keeps division out of the timer ISR and spreads fractional ticks across events. */
    const uint64_t numerator = (uint64_t)(uint32_t)interval_ns * motion_hardware_x_context.timer_ticks_per_ns_q32 +
                               motion_hardware_x_context.conversion_remainder_q32;
    uint64_t converted_ticks = numerator >> 32U;
    uint32_t conversion_remainder_q32 = (uint32_t)numerator;
    const uint64_t minimum_ticks =
        (uint64_t)motion_hardware_x_context.pulse_ticks + motion_hardware_x_context.low_time_ticks;
    if (converted_ticks < minimum_ticks) {
        converted_ticks = minimum_ticks;
        conversion_remainder_q32 = 0U;
    }
    motion_hardware_x_context.conversion_remainder_q32 = conversion_remainder_q32;
    if (converted_ticks == 0U) {
        return 0;
    }
    *ticks = converted_ticks;
    return 1;
}

static int motion_hardware_x_select_timer_scale(uint64_t first_interval_ns) {
    if (motion_hardware_x_context.timer_clock_hz == 0U || first_interval_ns > UINT32_MAX ||
        first_interval_ns > UINT64_MAX / motion_hardware_x_context.timer_clock_hz ||
        motion_hardware_x_context.config.step_pulse_width_ns > UINT64_MAX / motion_hardware_x_context.timer_clock_hz ||
        motion_hardware_x_context.config.step_low_time_ns > UINT64_MAX / motion_hardware_x_context.timer_clock_hz) {
        return 0;
    }

    const uint64_t raw_interval_cycles = motion_hardware_x_ceil_div_u64(
        first_interval_ns * motion_hardware_x_context.timer_clock_hz,
        MOTION_X_NANOSECONDS_PER_SECOND
    );
    const uint64_t raw_pulse_cycles = motion_hardware_x_ceil_div_u64(
        (uint64_t)motion_hardware_x_context.config.step_pulse_width_ns * motion_hardware_x_context.timer_clock_hz,
        MOTION_X_NANOSECONDS_PER_SECOND
    );
    const uint64_t raw_low_time_cycles = motion_hardware_x_ceil_div_u64(
        (uint64_t)motion_hardware_x_context.config.step_low_time_ns * motion_hardware_x_context.timer_clock_hz,
        MOTION_X_NANOSECONDS_PER_SECOND
    );
    if (raw_pulse_cycles < 2U || raw_low_time_cycles < 2U) {
        return 0;
    }

    uint64_t divider = motion_hardware_x_ceil_div_u64(raw_interval_cycles, MOTION_X_MAX_PERIOD_TICKS);
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
        timer_tick_denominator = MOTION_X_NANOSECONDS_PER_SECOND * divider;
        pulse_ticks = motion_hardware_x_ceil_div_u64(
            (uint64_t)motion_hardware_x_context.config.step_pulse_width_ns * motion_hardware_x_context.timer_clock_hz,
            timer_tick_denominator
        );
        low_time_ticks = motion_hardware_x_ceil_div_u64(
            (uint64_t)motion_hardware_x_context.config.step_low_time_ns * motion_hardware_x_context.timer_clock_hz,
            timer_tick_denominator
        );
        first_interval_ticks = motion_hardware_x_ceil_div_u64(
            first_interval_ns * motion_hardware_x_context.timer_clock_hz,
            timer_tick_denominator
        );
        if (first_interval_ticks >= pulse_ticks + low_time_ticks || divider == max_divider) {
            break;
        }
        ++divider;
    }
    const uint64_t direction_setup_ticks = motion_hardware_x_ceil_div_u64(
        (uint64_t)motion_hardware_x_context.config.direction_setup_ns * motion_hardware_x_context.timer_clock_hz,
        timer_tick_denominator
    );
    const uint64_t enable_direction_lead_ticks = motion_hardware_x_ceil_div_u64(
        (uint64_t)motion_hardware_x_context.config.enable_setup_ns * motion_hardware_x_context.timer_clock_hz,
        timer_tick_denominator
    );
    const uint64_t q32_numerator =
        ((uint64_t)motion_hardware_x_context.timer_clock_hz << 32U) + (timer_tick_denominator / 2U);
    const uint64_t timer_ticks_per_ns_q32 = q32_numerator / timer_tick_denominator;

    if (pulse_ticks == 0U || low_time_ticks == 0U || first_interval_ticks < pulse_ticks + low_time_ticks ||
        timer_ticks_per_ns_q32 == 0U || timer_ticks_per_ns_q32 > UINT32_MAX) {
        return 0;
    }

    motion_hardware_x_context.prescaler_divider = (uint32_t)divider;
    motion_hardware_x_context.pulse_ticks = (uint32_t)pulse_ticks;
    motion_hardware_x_context.low_time_ticks = (uint32_t)low_time_ticks;
    motion_hardware_x_context.timer_ticks_per_ns_q32 = (uint32_t)timer_ticks_per_ns_q32;
    motion_hardware_x_context.direction_setup_ticks = (uint32_t)direction_setup_ticks;
    motion_hardware_x_context.enable_direction_lead_ticks = (uint32_t)enable_direction_lead_ticks;
    motion_hardware_x_context.max_interval_ns = UINT32_MAX;
    motion_hardware_x_context.conversion_remainder_q32 = UINT32_MAX;
    return 1;
}

/* Returns the limit switch that blocks the event direction, or NONE when the move may continue. */
static MotionHardwareXStopReason_t motion_hardware_x_limit_blocks_direction(const MotionExecutionEvent_t *event) {
    const uint8_t toward_end = motion_hardware_x_direction_toward_end(motion_hardware_x_direction_is_positive(event));
    if (toward_end == 0U && motion_hardware_x_limit_is_active(
                                MOTION_X_START_LIMIT_PIN,
                                motion_hardware_x_context.config.start_limit_active_high
                            )) {
        return MOTION_HARDWARE_X_STOP_REASON_START_LIMIT;
    }
    if (toward_end != 0U && motion_hardware_x_limit_is_active(
                                MOTION_X_END_LIMIT_PIN,
                                motion_hardware_x_context.config.end_limit_active_high
                            )) {
        return MOTION_HARDWARE_X_STOP_REASON_END_LIMIT;
    }
    return MOTION_HARDWARE_X_STOP_REASON_NONE;
}

static void motion_hardware_x_stop_with_reason(MotionHardwareXStopReason_t reason) {
    motion_hardware_x_context.stop_reason = reason;
    motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_STOPPED);
    step_executor_reset();
}

static void motion_hardware_x_configure_gpio(const MotionHardwareXConfig_t *config) {
    const uint32_t step_idle_high = (uint32_t)!config->step_active_high;
    const uint32_t direction_high = config->positive_direction_high;
    const uint32_t enable_disabled_high = (uint32_t)!config->enable_active_high;
    GPIOB->BSRR = (step_idle_high ? MOTION_X_STEP_PIN : 0U) | (direction_high ? MOTION_X_DIRECTION_PIN : 0U) |
                  (enable_disabled_high ? MOTION_X_ENABLE_PIN : 0U) |
                  ((step_idle_high ? 0U : MOTION_X_STEP_PIN) << 16U) |
                  ((direction_high ? 0U : MOTION_X_DIRECTION_PIN) << 16U) |
                  ((enable_disabled_high ? 0U : MOTION_X_ENABLE_PIN) << 16U);
    GPIOB->OTYPER &= ~(MOTION_X_STEP_PIN | MOTION_X_DIRECTION_PIN | MOTION_X_ENABLE_PIN);
    GPIOB->OSPEEDR =
        (GPIOB->OSPEEDR & ~((3UL << 12U) | (3UL << 14U) | (3UL << 16U))) | (3UL << 12U) | (3UL << 14U) | (3UL << 16U);
    GPIOB->PUPDR &= ~((3UL << 12U) | (3UL << 14U) | (3UL << 16U));
    GPIOB->MODER =
        (GPIOB->MODER & ~((3UL << 12U) | (3UL << 14U) | (3UL << 16U))) | (1UL << 12U) | (1UL << 14U) | (1UL << 16U);

    GPIOF->MODER &= ~((3UL << 18U) | (3UL << 20U));
    GPIOF->PUPDR = (GPIOF->PUPDR & ~((3UL << 18U) | (3UL << 20U))) | ((uint32_t)config->limit_pull << 18U) |
                   ((uint32_t)config->limit_pull << 20U);
}

static void motion_hardware_x_configure_exti(const MotionHardwareXConfig_t *config) {
    const uint32_t start_edge = config->start_limit_active_high != 0U ? MOTION_X_START_LIMIT_PIN : 0U;
    const uint32_t end_edge = config->end_limit_active_high != 0U ? MOTION_X_END_LIMIT_PIN : 0U;
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(SYSCFG_EXTICR3_EXTI9_Msk | SYSCFG_EXTICR3_EXTI10_Msk)) |
                        SYSCFG_EXTICR3_EXTI9_PF | SYSCFG_EXTICR3_EXTI10_PF;
    EXTI->RTSR1 = (EXTI->RTSR1 & ~MOTION_X_LIMIT_MASK) | MOTION_X_START_LIMIT_PIN | end_edge;
    EXTI->FTSR1 = (EXTI->FTSR1 & ~MOTION_X_LIMIT_MASK) | MOTION_X_START_LIMIT_PIN |
                  (config->end_limit_active_high != 0U ? 0U : MOTION_X_END_LIMIT_PIN);
    EXTI->PR1 = MOTION_X_LIMIT_MASK;
    EXTI->IMR1 |= MOTION_X_LIMIT_MASK;
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, MOTION_X_LIMIT_IRQ_PRIORITY);
    NVIC_SetPriority(EXTI15_10_IRQn, MOTION_X_LIMIT_IRQ_PRIORITY);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void motion_hardware_x_configure_timer(void) {
    TIM4->CR1 = 0U;
    TIM4->CR2 = 0U;
    TIM4->SMCR = 0U;
    TIM4->DIER = 0U;
    TIM4->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM4->CCER = motion_hardware_x_context.config.step_active_high != 0U ? 0U : TIM_CCER_CC1P;
    TIM4->PSC = 0U;
    TIM4->ARR = 0xFFFFU;
    TIM4->CCR1 = 1U;
    TIM4->CNT = 0U;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
    NVIC_ClearPendingIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, MOTION_X_TIMER_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM4_IRQn);
}

MotionHardwareXStatus_t motion_hardware_x_init(const MotionHardwareXConfig_t *config) {
    if (motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_STARTING ||
        motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_WAITING ||
        motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_RUNNING) {
        return motion_hardware_x_context.status;
    }
    if (!motion_hardware_x_config_is_valid(config)) {
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_CONFIG;
        return motion_hardware_x_context.status;
    }

    motion_hardware_x_context = (MotionHardwareXContext_t){0};
    motion_hardware_x_context.config = *config;
    motion_hardware_x_context.timer_clock_hz = motion_hardware_x_timer_clock_hz();
    if (motion_hardware_x_context.timer_clock_hz == 0U) {
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_CLOCK;
        return motion_hardware_x_context.status;
    }

    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOFEN;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
    (void)RCC->AHB4ENR;
    __DSB();

    motion_hardware_x_configure_gpio(config);
    motion_hardware_x_configure_exti(config);
    motion_hardware_x_configure_timer();
    step_executor_reset();
    motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_READY;
    return motion_hardware_x_context.status;
}

static MotionHardwareXStatus_t motion_hardware_x_start_event(const MotionExecutionEvent_t *first_event) {
    if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_READY &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_COMPLETE &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_STOPPED) {
        return motion_hardware_x_context.status;
    }
    motion_hardware_x_context.stop_reason = MOTION_HARDWARE_X_STOP_REASON_NONE;
    if (first_event == NULL) {
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_EVENT;
        return motion_hardware_x_context.status;
    }
    if (first_event->step_mask != MOTION_X_STEP_AXIS_MASK ||
        (first_event->positive_direction_mask & (uint8_t)~MOTION_X_STEP_AXIS_MASK) != 0U) {
        step_executor_reset();
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_EVENT;
        return motion_hardware_x_context.status;
    }
    const MotionHardwareXStopReason_t blocking_limit = motion_hardware_x_limit_blocks_direction(first_event);
    if (blocking_limit != MOTION_HARDWARE_X_STOP_REASON_NONE) {
        step_executor_reset();
        motion_hardware_x_context.stop_reason = blocking_limit;
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_STOPPED;
        return motion_hardware_x_context.status;
    }
    if (motion_hardware_x_select_timer_scale(first_event->interval_ns) == 0) {
        step_executor_reset();
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_TIMING;
        return motion_hardware_x_context.status;
    }

    uint64_t first_interval_ticks = 0U;
    if (motion_hardware_x_interval_to_ticks(first_event->interval_ns, &first_interval_ticks) == 0) {
        step_executor_reset();
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_TIMING;
        return motion_hardware_x_context.status;
    }

    motion_hardware_x_context.current_event = *first_event;
    motion_hardware_x_context.current_direction_positive = motion_hardware_x_direction_is_positive(first_event);
    motion_hardware_x_context.first_interval_ticks = first_interval_ticks;
    motion_hardware_x_context.startup_phase = MOTION_X_STARTUP_ENABLE_LEAD;
    motion_hardware_x_set_enabled(1U);

    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->PSC = motion_hardware_x_context.prescaler_divider - 1U;
    TIM4->CCR1 = motion_hardware_x_context.pulse_ticks;
    TIM4->CCER &= ~TIM_CCER_CC1E;
    TIM4->DIER = 0U;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
    NVIC_ClearPendingIRQ(TIM4_IRQn);
    motion_hardware_x_schedule_delay(motion_hardware_x_context.enable_direction_lead_ticks);
    motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_STARTING;
    __DMB();
    TIM4->CR1 |= TIM_CR1_CEN;
    return motion_hardware_x_context.status;
}

MotionHardwareXStatus_t motion_hardware_x_start(void) {
    if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_READY &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_COMPLETE &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_STOPPED) {
        return motion_hardware_x_context.status;
    }

    motion_hardware_x_context.jog_mode = 0U;
    step_executor_reset();
    MotionExecutionEvent_t first_event = {0};
    const StepExecutorStatus_t executor_status = step_executor_begin(&first_event);
    if (executor_status != STEP_EXECUTOR_STATUS_OK) {
        step_executor_reset();
        motion_hardware_x_context.status = executor_status == STEP_EXECUTOR_STATUS_UNDERRUN
                                               ? MOTION_HARDWARE_X_STATUS_FAULT_UNDERRUN
                                               : MOTION_HARDWARE_X_STATUS_FAULT_EVENT;
        return motion_hardware_x_context.status;
    }
    return motion_hardware_x_start_event(&first_event);
}

MotionHardwareXStatus_t motion_hardware_x_start_jog_profile(
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2
) {
    if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_READY &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_COMPLETE &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_STOPPED) {
        return motion_hardware_x_context.status;
    }

    const uint64_t step_count = signed_steps < 0 ? (uint64_t)(-(int64_t)signed_steps) : (uint64_t)signed_steps;
    if (step_count == 0U || step_count > MOTION_X_JOG_MAX_STEPS || initial_step_rate_hz == 0U ||
        target_step_rate_hz < initial_step_rate_hz ||
        (target_step_rate_hz > initial_step_rate_hz && acceleration_steps_per_s2 == 0U)) {
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_FAULT_EVENT;
        return motion_hardware_x_context.status;
    }

    motion_hardware_x_context.jog_mode = 1U;
    motion_hardware_x_context.jog_steps_remaining = (uint32_t)step_count;
    motion_hardware_x_context.jog_total_steps = (uint32_t)step_count;
    motion_hardware_x_context.last_jog_steps_completed = 0U;
    motion_hardware_x_context.start_limit_release_event_pending = 0U;
    motion_hardware_x_context.jog_initial_step_rate_hz = initial_step_rate_hz;
    motion_hardware_x_context.jog_target_step_rate_hz = target_step_rate_hz;
    motion_hardware_x_context.jog_acceleration_steps_per_s2 = acceleration_steps_per_s2;
    motion_hardware_x_context.jog_step_rate_hz = initial_step_rate_hz;
    motion_hardware_x_context.jog_interval_base_ns = MOTION_X_NANOSECONDS_PER_SECOND / initial_step_rate_hz;
    motion_hardware_x_context.jog_interval_remainder_ns =
        (uint32_t)(MOTION_X_NANOSECONDS_PER_SECOND % initial_step_rate_hz);
    motion_hardware_x_context.jog_interval_remainder_accumulator = 0U;
    step_executor_reset();

    const MotionExecutionEvent_t first_event = {
        .interval_ns = motion_hardware_x_next_profiled_jog_interval_ns(),
        .sequence = 1U,
        .step_mask = MOTION_X_STEP_AXIS_MASK,
        .positive_direction_mask = signed_steps > 0 ? MOTION_X_STEP_AXIS_MASK : 0U,
        .last_in_block = 0U,
    };
    const MotionHardwareXStatus_t start_status = motion_hardware_x_start_event(&first_event);
    if (start_status != MOTION_HARDWARE_X_STATUS_STARTING) {
        motion_hardware_x_context.jog_mode = 0U;
        motion_hardware_x_context.jog_steps_remaining = 0U;
        motion_hardware_x_context.jog_total_steps = 0U;
        motion_hardware_x_context.jog_initial_step_rate_hz = 0U;
        motion_hardware_x_context.jog_target_step_rate_hz = 0U;
        motion_hardware_x_context.jog_acceleration_steps_per_s2 = 0U;
    }
    return start_status;
}

MotionHardwareXStatus_t motion_hardware_x_start_jog(int32_t signed_steps, uint32_t step_rate_hz) {
    return motion_hardware_x_start_jog_profile(signed_steps, step_rate_hz, step_rate_hz, 0U);
}

void motion_hardware_x_request_stop(void) {
    const MotionHardwareXStatus_t status = motion_hardware_x_context.status;
    if (status == MOTION_HARDWARE_X_STATUS_STARTING || status == MOTION_HARDWARE_X_STATUS_WAITING ||
        status == MOTION_HARDWARE_X_STATUS_RUNNING) {
        const uint32_t primask = motion_hardware_x_enter_critical();
        if (motion_hardware_x_context.stop_requested == 0U) {
            motion_hardware_x_context.pending_stop_reason = MOTION_HARDWARE_X_STOP_REASON_REQUEST;
            motion_hardware_x_context.stop_requested = 1U;
        }
        motion_hardware_x_exit_critical(primask);
    }
}

MotionHardwareXStopReason_t motion_hardware_x_get_last_stop_reason(void) {
    return motion_hardware_x_context.stop_reason;
}

MotionHardwareXStatus_t motion_hardware_x_clear_fault(void) {
    const uint32_t primask = motion_hardware_x_enter_critical();
    const MotionHardwareXStatus_t status = motion_hardware_x_context.status;
    if (status == MOTION_HARDWARE_X_STATUS_FAULT_LIMIT || status == MOTION_HARDWARE_X_STATUS_FAULT_UNDERRUN ||
        status == MOTION_HARDWARE_X_STATUS_FAULT_EVENT || status == MOTION_HARDWARE_X_STATUS_FAULT_TIMING) {
        /* A runtime fault already stopped TIM4; force the idle outputs again before allowing a new start. */
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_READY;
        motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_READY);
        motion_hardware_x_context.startup_phase = MOTION_X_STARTUP_NONE;
        motion_hardware_x_context.delay_remaining_ticks = 0U;
        step_executor_reset();
    }
    const MotionHardwareXStatus_t result = motion_hardware_x_context.status;
    motion_hardware_x_exit_critical(primask);
    return result;
}

MotionHardwareXStatus_t motion_hardware_x_get_status(void) { return motion_hardware_x_context.status; }

uint8_t motion_hardware_x_start_limit_is_active(void) {
    if (motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_UNINITIALIZED) {
        return 0U;
    }
    return motion_hardware_x_limit_is_active(
        MOTION_X_START_LIMIT_PIN,
        motion_hardware_x_context.config.start_limit_active_high
    );
}

uint8_t motion_hardware_x_take_start_limit_release_event(void) {
    const uint32_t primask = motion_hardware_x_enter_critical();
    const uint8_t pending = motion_hardware_x_context.start_limit_release_event_pending;
    motion_hardware_x_context.start_limit_release_event_pending = 0U;
    motion_hardware_x_exit_critical(primask);
    return pending;
}

uint32_t motion_hardware_x_get_last_jog_steps_completed(void) {
    return motion_hardware_x_context.last_jog_steps_completed;
}

void motion_hardware_x_arm_start_limit_release_stop(void) {
    const uint32_t primask = motion_hardware_x_enter_critical();
    motion_hardware_x_context.start_limit_release_event_pending = 0U;
    motion_hardware_x_context.start_limit_release_stop_armed = 1U;
    __DMB();
    motion_hardware_x_exit_critical(primask);
}

void motion_hardware_x_disarm_start_limit_release_stop(void) {
    const uint32_t primask = motion_hardware_x_enter_critical();
    motion_hardware_x_context.start_limit_release_stop_armed = 0U;
    __DMB();
    motion_hardware_x_exit_critical(primask);
}

void motion_hardware_x_timer_irq(void) {
    const uint32_t status = TIM4->SR;
    if (motion_hardware_x_context.stop_requested != 0U &&
        (motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_STARTING ||
            motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_WAITING ||
            motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_RUNNING)) {
        if (motion_hardware_x_context.jog_mode != 0U &&
            motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_RUNNING && (status & TIM_SR_CC1IF) != 0U &&
            motion_hardware_x_context.jog_steps_remaining != 0U) {
            --motion_hardware_x_context.jog_steps_remaining;
        }
        motion_hardware_x_stop_with_reason(motion_hardware_x_context.pending_stop_reason);
        return;
    }
    if ((status & TIM_SR_UIF) != 0U && (motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_STARTING ||
                                           motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_WAITING)) {
        TIM4->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
        if (motion_hardware_x_context.delay_remaining_ticks != 0U) {
            TIM4->CR1 &= ~TIM_CR1_CEN;
            const uint32_t elapsed_ticks = TIM4->CNT;
            if (motion_hardware_x_context.delay_remaining_ticks > elapsed_ticks) {
                const uint32_t primask = motion_hardware_x_enter_critical();
                if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_STARTING &&
                    motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_WAITING) {
                    motion_hardware_x_exit_critical(primask);
                    return;
                }
                motion_hardware_x_context.delay_remaining_ticks -= elapsed_ticks;
                motion_hardware_x_schedule_delay(motion_hardware_x_context.delay_remaining_ticks);
                TIM4->CR1 |= TIM_CR1_CEN;
                motion_hardware_x_exit_critical(primask);
                return;
            }
            motion_hardware_x_context.delay_remaining_ticks = 0U;
        }

        const uint32_t primask = motion_hardware_x_enter_critical();
        if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_STARTING &&
            motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_WAITING) {
            motion_hardware_x_exit_critical(primask);
            return;
        }
        const MotionHardwareXStopReason_t blocking_limit =
            motion_hardware_x_limit_blocks_direction(&motion_hardware_x_context.current_event);
        if (blocking_limit != MOTION_HARDWARE_X_STOP_REASON_NONE) {
            motion_hardware_x_stop_with_reason(blocking_limit);
            motion_hardware_x_exit_critical(primask);
            return;
        }

        if (motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_STARTING) {
            switch (motion_hardware_x_context.startup_phase) {
            case MOTION_X_STARTUP_ENABLE_LEAD:
                motion_hardware_x_set_direction(
                    motion_hardware_x_direction_is_positive(&motion_hardware_x_context.current_event)
                );
                motion_hardware_x_context.startup_phase = MOTION_X_STARTUP_DIRECTION_SETUP;
                motion_hardware_x_schedule_delay(motion_hardware_x_context.direction_setup_ticks);
                TIM4->CR1 |= TIM_CR1_CEN;
                motion_hardware_x_exit_critical(primask);
                return;
            case MOTION_X_STARTUP_DIRECTION_SETUP:
                motion_hardware_x_context.startup_phase = MOTION_X_STARTUP_FIRST_INTERVAL;
                motion_hardware_x_schedule_delay(motion_hardware_x_context.first_interval_ticks);
                TIM4->CR1 |= TIM_CR1_CEN;
                motion_hardware_x_exit_critical(primask);
                return;
            case MOTION_X_STARTUP_FIRST_INTERVAL:
                motion_hardware_x_context.startup_phase = MOTION_X_STARTUP_NONE;
                break;
            case MOTION_X_STARTUP_NONE:
            default:
                motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_FAULT_EVENT);
                step_executor_reset();
                motion_hardware_x_exit_critical(primask);
                return;
            }
        }

        motion_hardware_x_set_step_alternate_function();
        TIM4->CNT = 0U;
        TIM4->CCR1 = motion_hardware_x_context.pulse_ticks;
        TIM4->ARR = 0xFFFFU;
        TIM4->CCER = (TIM4->CCER & ~TIM_CCER_CC1E) |
                     (motion_hardware_x_context.config.step_active_high != 0U ? 0U : TIM_CCER_CC1P) | TIM_CCER_CC1E;
        TIM4->DIER = TIM_DIER_CC1IE;
        motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_RUNNING;
        motion_hardware_x_exit_critical(primask);
        return;
    }

    if ((status & TIM_SR_CC1IF) != 0U && motion_hardware_x_context.status == MOTION_HARDWARE_X_STATUS_RUNNING) {
        TIM4->SR = ~(TIM_SR_CC1IF | TIM_SR_UIF);
        MotionExecutionEvent_t next_event = {0};
        StepExecutorStatus_t executor_status;
        if (motion_hardware_x_context.jog_mode != 0U) {
            if (motion_hardware_x_context.jog_steps_remaining <= 1U) {
                motion_hardware_x_context.jog_steps_remaining = 0U;
                executor_status = STEP_EXECUTOR_STATUS_COMPLETE;
            } else {
                --motion_hardware_x_context.jog_steps_remaining;
                next_event = motion_hardware_x_context.current_event;
                next_event.interval_ns = motion_hardware_x_next_profiled_jog_interval_ns();
                executor_status = STEP_EXECUTOR_STATUS_OK;
            }
        } else {
            executor_status = step_executor_pulse_complete(&next_event);
        }
        if (executor_status == STEP_EXECUTOR_STATUS_COMPLETE) {
            const uint32_t primask = motion_hardware_x_enter_critical();
            if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_RUNNING) {
                motion_hardware_x_exit_critical(primask);
                return;
            }
            const MotionHardwareXStopReason_t blocking_limit =
                motion_hardware_x_limit_blocks_direction(&motion_hardware_x_context.current_event);
            if (blocking_limit != MOTION_HARDWARE_X_STOP_REASON_NONE) {
                motion_hardware_x_stop_with_reason(blocking_limit);
                motion_hardware_x_exit_critical(primask);
                return;
            }
            motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_COMPLETE);
            motion_hardware_x_exit_critical(primask);
            return;
        }
        if (executor_status != STEP_EXECUTOR_STATUS_OK || next_event.step_mask != MOTION_X_STEP_AXIS_MASK ||
            (next_event.positive_direction_mask & (uint8_t)~MOTION_X_STEP_AXIS_MASK) != 0U) {
            motion_hardware_x_stop(
                executor_status == STEP_EXECUTOR_STATUS_UNDERRUN ? MOTION_HARDWARE_X_STATUS_FAULT_UNDERRUN
                                                                 : MOTION_HARDWARE_X_STATUS_FAULT_EVENT
            );
            return;
        }

        uint64_t next_interval_ticks = 0U;
        if (motion_hardware_x_interval_to_ticks(next_event.interval_ns, &next_interval_ticks) == 0) {
            motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_FAULT_TIMING);
            return;
        }
        if (next_interval_ticks <
            (uint64_t)motion_hardware_x_context.pulse_ticks + motion_hardware_x_context.low_time_ticks) {
            motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_FAULT_TIMING);
            return;
        }
        const uint32_t primask = motion_hardware_x_enter_critical();
        if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_RUNNING) {
            motion_hardware_x_exit_critical(primask);
            return;
        }
        const MotionHardwareXStopReason_t blocking_limit = motion_hardware_x_limit_blocks_direction(&next_event);
        if (blocking_limit != MOTION_HARDWARE_X_STOP_REASON_NONE) {
            motion_hardware_x_stop_with_reason(blocking_limit);
            motion_hardware_x_exit_critical(primask);
            return;
        }
        const uint8_t next_direction_positive = motion_hardware_x_direction_is_positive(&next_event);
        const uint8_t direction_changed =
            next_direction_positive != motion_hardware_x_context.current_direction_positive;
        if (direction_changed != 0U) {
            motion_hardware_x_set_direction(next_direction_positive);
        }

        if (next_interval_ticks <=
            (uint64_t)TIM4->CNT + MOTION_X_ISR_GUARD_TICKS +
                (direction_changed != 0U ? motion_hardware_x_context.direction_setup_ticks : 0U)) {
            motion_hardware_x_stop(MOTION_HARDWARE_X_STATUS_FAULT_TIMING);
            motion_hardware_x_exit_critical(primask);
            return;
        }

        motion_hardware_x_context.current_event = next_event;
        if (next_interval_ticks <= MOTION_X_MAX_PERIOD_TICKS) {
            TIM4->ARR = (uint32_t)next_interval_ticks - 1U;
        } else {
            TIM4->CCER &= ~TIM_CCER_CC1E;
            motion_hardware_x_set_step_idle_gpio();
            TIM4->ARR = 0xFFFFU;
            motion_hardware_x_context.delay_remaining_ticks = next_interval_ticks - MOTION_X_MAX_PERIOD_TICKS;
            motion_hardware_x_context.status = MOTION_HARDWARE_X_STATUS_WAITING;
            TIM4->DIER = TIM_DIER_UIE;
        }
        motion_hardware_x_exit_critical(primask);
    }
}

void motion_hardware_x_limit_irq(uint32_t pending_lines) {
    if (motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_STARTING &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_WAITING &&
        motion_hardware_x_context.status != MOTION_HARDWARE_X_STATUS_RUNNING) {
        return;
    }

    const uint8_t toward_end =
        motion_hardware_x_direction_toward_end(motion_hardware_x_context.current_direction_positive);
    const uint8_t start_pending = (pending_lines & MOTION_X_START_LIMIT_PIN) != 0U;
    const uint8_t start_active = start_pending != 0U && motion_hardware_x_limit_is_active(
                                                            MOTION_X_START_LIMIT_PIN,
                                                            motion_hardware_x_context.config.start_limit_active_high
                                                        ) != 0U;
    if (start_pending != 0U && start_active == 0U) {
        motion_hardware_x_context.start_limit_release_event_pending = 1U;
        if (motion_hardware_x_context.start_limit_release_stop_armed != 0U && toward_end != 0U &&
            motion_hardware_x_context.stop_requested == 0U) {
            motion_hardware_x_context.pending_stop_reason = MOTION_HARDWARE_X_STOP_REASON_START_LIMIT_RELEASE;
            motion_hardware_x_context.stop_requested = 1U;
        }
        __DMB();
    }
    const uint8_t end_active =
        (pending_lines & MOTION_X_END_LIMIT_PIN) != 0U && motion_hardware_x_limit_is_active(
                                                              MOTION_X_END_LIMIT_PIN,
                                                              motion_hardware_x_context.config.end_limit_active_high
                                                          ) != 0U;

    if (start_active != 0U && toward_end == 0U) {
        motion_hardware_x_stop_with_reason(MOTION_HARDWARE_X_STOP_REASON_START_LIMIT);
    } else if (end_active != 0U && toward_end != 0U) {
        motion_hardware_x_stop_with_reason(MOTION_HARDWARE_X_STOP_REASON_END_LIMIT);
    }
}

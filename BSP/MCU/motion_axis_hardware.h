#ifndef FM_V3_MOTION_AXIS_HARDWARE_H
#define FM_V3_MOTION_AXIS_HARDWARE_H

#include "motion/motion_event_source.h"

#include <stdint.h>

#define MOTION_AXIS_HARDWARE_DM556_MIN_STEP_PULSE_NS 2500U
#define MOTION_AXIS_HARDWARE_DM556_MIN_STEP_LOW_NS 2500U
#define MOTION_AXIS_HARDWARE_DM556_MIN_DIRECTION_SETUP_NS 5000U
#define MOTION_AXIS_HARDWARE_DM556_MIN_ENABLE_LEAD_NS 5000U

/* One STEP/DIR/ENA output with its own timer, two switches and a scale. The value is the planner axis index.
 *   X: TIM4 CH1 STEP PB6, DIR PB7, ENA PB8; start switch PF10, end switch PF9.
 *   Y: TIM1 CH2 STEP PA9, DIR PA10, ENA PA11; start switch PE5, end switch PE4.
 *   Z: TIM3 CH3 STEP PC8, DIR PC9, ENA PA8; start switch PE6 (min), end switch PF7 (max). */
typedef enum { MOTION_AXIS_X = 0, MOTION_AXIS_Y, MOTION_AXIS_Z, MOTION_AXIS_HARDWARE_COUNT } MotionAxis_t;

typedef enum {
    MOTION_AXIS_HARDWARE_LIMIT_PULL_NONE = 0,
    MOTION_AXIS_HARDWARE_LIMIT_PULL_UP,
    MOTION_AXIS_HARDWARE_LIMIT_PULL_DOWN
} MotionAxisHardwareLimitPull_t;

/* Reads the axis position in um; called from the switch interrupt, so it must be short and must not block. */
typedef int32_t (*MotionAxisHardwarePositionReader_t)(void);

typedef struct {
    uint8_t step_active_high;                 /* Must be 1: the STEP output is active-high. */
    uint8_t enable_active_high;               /* ENA active level. */
    uint8_t positive_direction_high;          /* DIR level for positive motion. */
    uint8_t positive_moves_toward_end;        /* Positive motion heads toward the end switch when set. */
    uint8_t start_limit_active_high;          /* Start switch active level. */
    uint8_t end_limit_active_high;            /* End switch active level. */
    MotionAxisHardwareLimitPull_t limit_pull; /* External board pull resistors may make NONE correct. */
    uint32_t step_pulse_width_ns;             /* Must satisfy the motor-driver data sheet. */
    uint32_t step_low_time_ns;                /* Minimum inactive STEP interval from the data sheet. */
    uint32_t direction_setup_ns;
    uint32_t direction_hold_ns; /* This backend requires hold <= STEP high/active width. */
    uint32_t enable_setup_ns;   /* Minimum ENA-to-DIR lead time. */
    uint32_t enable_hold_ns;    /* This backend requires hold <= STEP high/active width. */
    /* A switch that a profiled move (start_jog_profile) runs into brakes it at this rate instead of stopping it
     * dead; 0 keeps the hard stop. Planned moves and STOP always stop at once. */
    uint32_t limit_deceleration_steps_per_s2;
    /* Read when a switch fires, so the switch point is known even though the axis brakes past it; may be NULL. */
    MotionAxisHardwarePositionReader_t position_reader;
} MotionAxisHardwareConfig_t;

typedef enum {
    MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED = 0,
    MOTION_AXIS_HARDWARE_STATUS_READY,
    MOTION_AXIS_HARDWARE_STATUS_STARTING,
    MOTION_AXIS_HARDWARE_STATUS_WAITING,
    MOTION_AXIS_HARDWARE_STATUS_RUNNING,
    MOTION_AXIS_HARDWARE_STATUS_COMPLETE,
    MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT,
    MOTION_AXIS_HARDWARE_STATUS_FAULT_UNDERRUN,
    MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT,
    MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING,
    MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG,
    MOTION_AXIS_HARDWARE_STATUS_FAULT_CLOCK,
    MOTION_AXIS_HARDWARE_STATUS_STOPPED
} MotionAxisHardwareStatus_t;

/* Why the last move ended with MOTION_AXIS_HARDWARE_STATUS_STOPPED; reset to NONE on every start. */
typedef enum {
    MOTION_AXIS_HARDWARE_STOP_REASON_NONE = 0,
    MOTION_AXIS_HARDWARE_STOP_REASON_REQUEST,
    MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
    MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
    MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE
} MotionAxisHardwareStopReason_t;

/* Runs in interrupt context: must be short and must not block. */
typedef void (*MotionAxisHardwareStartLimitHook_t)(void);

#define MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE UINT32_MAX

/* Step-timer health of one axis, sent in the diagnostic frame. */
typedef struct {
    uint32_t status; /* MotionAxisHardwareStatus_t */
    /* Moves stopped with MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING since power-up, mostly missed step deadlines. */
    uint32_t timing_faults;
    /* Longest step-timer interrupt since the last peak reset. */
    uint32_t isr_max_ns;
    /* Least time left before a step deadline when the interrupt loaded it, since the last peak reset;
     * MOTION_AXIS_HARDWARE_NO_SLACK_SAMPLE until a step was loaded. Near 0 means the axis is close to a fault. */
    uint32_t min_slack_ns;
} MotionAxisHardwareTiming_t;

/* Every call takes the axis; an axis outside MotionAxis_t is ignored (status UNINITIALIZED, values 0).
 * Electrical levels and driver timing are mandatory; no board defaults are assumed. */
MotionAxisHardwareStatus_t motion_axis_hardware_init(MotionAxis_t axis, const MotionAxisHardwareConfig_t *config);
/* Where the axis pulls its planned step events from. Needed before start; survives init. Ignored (status returned
 * unchanged) while the axis is moving or when `source` is incomplete. */
MotionAxisHardwareStatus_t motion_axis_hardware_set_event_source(MotionAxis_t axis, const MotionEventSource_t *source);
/* Runs the planned step stream of the axis event source. Without a valid source the axis ends in
 * MOTION_AXIS_HARDWARE_STATUS_FAULT_EVENT. */
MotionAxisHardwareStatus_t motion_axis_hardware_start(MotionAxis_t axis);
/* Planned DMA stream: enables the driver, waits its enable lead and sets DIR. Only while the axis is READY, COMPLETE
 * or STOPPED; any other status is returned unchanged and nothing is written. The caller still has to wait the DIR
 * setup time before the first STEP. */
MotionAxisHardwareStatus_t motion_axis_hardware_set_stream_outputs(MotionAxis_t axis, uint8_t positive_direction);
MotionAxisHardwareStatus_t motion_axis_hardware_start_jog_profile(
    MotionAxis_t axis,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2
);
/* Stops active motion at the next timer interrupt. */
void motion_axis_hardware_request_stop(MotionAxis_t axis);
/* Returns a runtime fault (limit, underrun, event, timing) to READY. Config and clock faults need init. */
MotionAxisHardwareStatus_t motion_axis_hardware_clear_fault(MotionAxis_t axis);
MotionAxisHardwareStatus_t motion_axis_hardware_ready_after_home(MotionAxis_t axis);
MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis);
MotionAxisHardwareStopReason_t motion_axis_hardware_get_last_stop_reason(MotionAxis_t axis);
/* Forgets the last stop reason. The planned stream of X records its stop here without clearing the old one, so
 * MotionTask clears it, while the stream is not running, before it waits for a new stop. */
void motion_axis_hardware_clear_stop_reason(MotionAxis_t axis);
uint8_t motion_axis_hardware_start_limit_is_active(MotionAxis_t axis);
uint8_t motion_axis_hardware_end_limit_is_active(MotionAxis_t axis);
uint32_t motion_axis_hardware_get_last_jog_steps_completed(MotionAxis_t axis);
/* Position reader value at the moment a switch fired during the last move; 0 when no reader is configured. */
int32_t motion_axis_hardware_get_limit_trigger_position(MotionAxis_t axis);
/* While armed, the start switch release during a move toward the end switch stops that move. */
void motion_axis_hardware_arm_start_limit_release_stop(MotionAxis_t axis);
void motion_axis_hardware_disarm_start_limit_release_stop(MotionAxis_t axis);
/* One shot: `hook` runs in the same interrupt that stops the next move on the start switch, so it sees the exact
 * switch point. Any stop clears it; NULL disarms it. */
void motion_axis_hardware_arm_start_limit_hook(MotionAxis_t axis, MotionAxisHardwareStartLimitHook_t hook);
/* An unknown axis leaves `timing` untouched. */
void motion_axis_hardware_get_timing(MotionAxis_t axis, MotionAxisHardwareTiming_t *timing);
/* Starts a new measurement of isr_max_ns and min_slack_ns; timing_faults keeps counting. */
void motion_axis_hardware_reset_timing_peaks(MotionAxis_t axis);

/* Called only by the CMSIS interrupt handlers: the axis timer vectors, and the EXTI vectors with the pending lines
 * they own (already cleared); lines of every axis may be passed together. */
void motion_axis_hardware_timer_irq(MotionAxis_t axis);
void motion_axis_hardware_limit_irq(uint32_t pending_lines);
/* Looks at the level of the X limit switches and brakes the running planned stream if it moves toward a pressed one.
 * The stream start calls it, because an edge that came before the start raised no interrupt that anybody acted on. */
void motion_axis_hardware_poll_stream_limit(void);

#endif /* FM_V3_MOTION_AXIS_HARDWARE_H */

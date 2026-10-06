#ifndef FM_V3_MOTION_HARDWARE_X_H
#define FM_V3_MOTION_HARDWARE_X_H

#include <stdint.h>

#define MOTION_HARDWARE_X_DM556_MIN_STEP_PULSE_NS 2500U
#define MOTION_HARDWARE_X_DM556_MIN_STEP_LOW_NS 2500U
#define MOTION_HARDWARE_X_DM556_MIN_DIRECTION_SETUP_NS 5000U
#define MOTION_HARDWARE_X_DM556_MIN_ENABLE_LEAD_NS 5000U

typedef enum {
    MOTION_HARDWARE_X_LIMIT_PULL_NONE = 0,
    MOTION_HARDWARE_X_LIMIT_PULL_UP,
    MOTION_HARDWARE_X_LIMIT_PULL_DOWN
} MotionHardwareXLimitPull_t;

typedef struct {
    uint8_t step_active_high;              /* Must be 1 for the active-high PB6 STEP output. */
    uint8_t enable_active_high;            /* ENA active level on PB8 EB output. */
    uint8_t positive_direction_high;       /* PB7 level for positive X motion. */
    uint8_t positive_moves_toward_end;     /* Positive motion heads toward PF9 when set. */
    uint8_t start_limit_active_high;       /* PF10 active level. */
    uint8_t end_limit_active_high;         /* PF9 active level. */
    MotionHardwareXLimitPull_t limit_pull; /* External board pull resistors may make NONE correct. */
    uint32_t step_pulse_width_ns;          /* Must satisfy the motor-driver data sheet. */
    uint32_t step_low_time_ns;             /* Minimum inactive STEP interval from the data sheet. */
    uint32_t direction_setup_ns;
    uint32_t direction_hold_ns; /* This backend requires hold <= STEP high/active width. */
    uint32_t enable_setup_ns;   /* Minimum ENA-to-DIR lead time when PB8 is connected to ENA. */
    uint32_t enable_hold_ns;    /* This backend requires hold <= STEP high/active width. */
} MotionHardwareXConfig_t;

typedef enum {
    MOTION_HARDWARE_X_STATUS_UNINITIALIZED = 0,
    MOTION_HARDWARE_X_STATUS_READY,
    MOTION_HARDWARE_X_STATUS_STARTING,
    MOTION_HARDWARE_X_STATUS_WAITING,
    MOTION_HARDWARE_X_STATUS_RUNNING,
    MOTION_HARDWARE_X_STATUS_COMPLETE,
    MOTION_HARDWARE_X_STATUS_FAULT_LIMIT,
    MOTION_HARDWARE_X_STATUS_FAULT_UNDERRUN,
    MOTION_HARDWARE_X_STATUS_FAULT_EVENT,
    MOTION_HARDWARE_X_STATUS_FAULT_TIMING,
    MOTION_HARDWARE_X_STATUS_FAULT_CONFIG,
    MOTION_HARDWARE_X_STATUS_FAULT_CLOCK,
    MOTION_HARDWARE_X_STATUS_STOPPED
} MotionHardwareXStatus_t;

/* Why the last move ended with MOTION_HARDWARE_X_STATUS_STOPPED; reset to NONE on every start. */
typedef enum {
    MOTION_HARDWARE_X_STOP_REASON_NONE = 0,
    MOTION_HARDWARE_X_STOP_REASON_REQUEST,
    MOTION_HARDWARE_X_STOP_REASON_START_LIMIT,
    MOTION_HARDWARE_X_STOP_REASON_END_LIMIT,
    MOTION_HARDWARE_X_STOP_REASON_START_LIMIT_RELEASE
} MotionHardwareXStopReason_t;

/* Electrical levels and driver timing are mandatory; no board defaults are assumed. */
MotionHardwareXStatus_t motion_hardware_x_init(const MotionHardwareXConfig_t *config);
MotionHardwareXStatus_t motion_hardware_x_start(void);
MotionHardwareXStatus_t motion_hardware_x_start_jog(int32_t signed_steps, uint32_t step_rate_hz);
MotionHardwareXStatus_t motion_hardware_x_start_jog_profile(
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2
);
/* Stops active motion at the next timer interrupt. */
void motion_hardware_x_request_stop(void);
/* Returns a runtime fault (limit, underrun, event, timing) to READY. Config and clock faults need init. */
MotionHardwareXStatus_t motion_hardware_x_clear_fault(void);
MotionHardwareXStatus_t motion_hardware_x_get_status(void);
MotionHardwareXStopReason_t motion_hardware_x_get_last_stop_reason(void);
uint8_t motion_hardware_x_start_limit_is_active(void);
uint8_t motion_hardware_x_take_start_limit_release_event(void);
uint32_t motion_hardware_x_get_last_jog_steps_completed(void);
void motion_hardware_x_arm_start_limit_release_stop(void);
void motion_hardware_x_disarm_start_limit_release_stop(void);

/* Called only by the matching CMSIS interrupt handlers. */
void motion_hardware_x_timer_irq(void);
void motion_hardware_x_limit_irq(uint32_t pending_lines);

#endif /* FM_V3_MOTION_HARDWARE_X_H */

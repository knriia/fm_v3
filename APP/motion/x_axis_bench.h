#ifndef FM_V3_X_AXIS_BENCH_H
#define FM_V3_X_AXIS_BENCH_H

#include "motion_hardware_x.h"

#include <stdint.h>

#define X_AXIS_BENCH_MAX_STEPS 1000000U

typedef enum {
    X_AXIS_BENCH_STATUS_STARTED = 0,
    X_AXIS_BENCH_STATUS_INVALID_ARGUMENT,
    X_AXIS_BENCH_STATUS_BUSY,
    X_AXIS_BENCH_STATUS_BUFFER_ERROR,
    X_AXIS_BENCH_STATUS_HARDWARE_ERROR
} XAxisBenchStatus_t;

/* Starts one finite, constant-rate X move expressed as signed motor steps. */
XAxisBenchStatus_t x_axis_bench_start(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t step_rate_hz,
    MotionHardwareXStatus_t *hardware_status
);

/* Starts a finite X move with a symmetric acceleration/deceleration profile. */
XAxisBenchStatus_t x_axis_bench_start_profile(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionHardwareXStatus_t *hardware_status
);

#endif /* FM_V3_X_AXIS_BENCH_H */

#ifndef FM_V3_AXIS_BENCH_H
#define FM_V3_AXIS_BENCH_H

#include "motion_axis_hardware.h"

#include <stdint.h>

#define AXIS_BENCH_MAX_STEPS 1000000U

typedef enum {
    AXIS_BENCH_STATUS_STARTED = 0,
    AXIS_BENCH_STATUS_INVALID_ARGUMENT,
    AXIS_BENCH_STATUS_BUSY,
    AXIS_BENCH_STATUS_BUFFER_ERROR,
    AXIS_BENCH_STATUS_HARDWARE_ERROR
} AxisBenchStatus_t;

/* Start rate, top rate and acceleration of one move; equal rates with no acceleration give a constant rate. */
typedef struct {
    uint32_t initial_step_rate_hz;
    uint32_t target_step_rate_hz;
    uint32_t acceleration_steps_per_s2;
} AxisBenchProfile_t;

typedef enum { AXIS_BENCH_TOWARD_START = 0, AXIS_BENCH_TOWARD_END } AxisBenchDirection_t;

/* Starts a finite move of one axis toward one of its switches with a symmetric acceleration/deceleration profile;
 * hardware_config->positive_moves_toward_end turns the direction into the motor direction. */
AxisBenchStatus_t axis_bench_start_profile(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionAxisHardwareStatus_t *hardware_status
);

#endif /* FM_V3_AXIS_BENCH_H */

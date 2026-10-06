#include "motion/x_axis_bench.h"

#include <stddef.h>
#include <stdint.h>

#define X_AXIS_BENCH_NANOSECONDS_PER_SECOND 1000000000ULL

/* The STEP period cannot be shorter than the driver's pulse width plus its minimum low time. */
static uint32_t x_axis_bench_max_step_rate_hz(const MotionHardwareXConfig_t *hardware_config) {
    const uint64_t min_period_ns = (uint64_t)hardware_config->step_pulse_width_ns + hardware_config->step_low_time_ns;
    if (min_period_ns == 0U) {
        return 0U;
    }
    return (uint32_t)(X_AXIS_BENCH_NANOSECONDS_PER_SECOND / min_period_ns);
}

static XAxisBenchStatus_t x_axis_bench_start_internal(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionHardwareXStatus_t *hardware_status
) {
    if (hardware_config == NULL || signed_steps == 0 || initial_step_rate_hz == 0U ||
        target_step_rate_hz < initial_step_rate_hz ||
        (target_step_rate_hz > initial_step_rate_hz && acceleration_steps_per_s2 == 0U) || hardware_status == NULL) {
        return X_AXIS_BENCH_STATUS_INVALID_ARGUMENT;
    }

    const uint64_t step_count = signed_steps < 0 ? (uint64_t)(-(int64_t)signed_steps) : (uint64_t)signed_steps;
    if (step_count > X_AXIS_BENCH_MAX_STEPS || target_step_rate_hz > x_axis_bench_max_step_rate_hz(hardware_config)) {
        return X_AXIS_BENCH_STATUS_INVALID_ARGUMENT;
    }

    MotionHardwareXStatus_t init_status = motion_hardware_x_get_status();
    if (init_status == MOTION_HARDWARE_X_STATUS_UNINITIALIZED) {
        init_status = motion_hardware_x_init(hardware_config);
    }
    *hardware_status = init_status;
    if (init_status == MOTION_HARDWARE_X_STATUS_STARTING || init_status == MOTION_HARDWARE_X_STATUS_WAITING ||
        init_status == MOTION_HARDWARE_X_STATUS_RUNNING) {
        return X_AXIS_BENCH_STATUS_BUSY;
    }
    if (init_status != MOTION_HARDWARE_X_STATUS_READY && init_status != MOTION_HARDWARE_X_STATUS_COMPLETE &&
        init_status != MOTION_HARDWARE_X_STATUS_STOPPED) {
        return X_AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }

    *hardware_status = motion_hardware_x_start_jog_profile(
        signed_steps,
        initial_step_rate_hz,
        target_step_rate_hz,
        acceleration_steps_per_s2
    );
    if (*hardware_status != MOTION_HARDWARE_X_STATUS_STARTING) {
        return X_AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }
    return X_AXIS_BENCH_STATUS_STARTED;
}

XAxisBenchStatus_t x_axis_bench_start(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t step_rate_hz,
    MotionHardwareXStatus_t *hardware_status
) {
    return x_axis_bench_start_internal(hardware_config, signed_steps, step_rate_hz, step_rate_hz, 0U, hardware_status);
}

XAxisBenchStatus_t x_axis_bench_start_profile(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionHardwareXStatus_t *hardware_status
) {
    return x_axis_bench_start_internal(
        hardware_config,
        signed_steps,
        initial_step_rate_hz,
        target_step_rate_hz,
        acceleration_steps_per_s2,
        hardware_status
    );
}

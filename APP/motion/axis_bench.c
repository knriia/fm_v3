#include "motion/axis_bench.h"

#include "cmsis_os.h"

#include <stddef.h>
#include <stdint.h>

#define AXIS_BENCH_NANOSECONDS_PER_SECOND 1000000000ULL

/* The STEP period cannot be shorter than the driver's pulse width plus its minimum low time. */
static uint32_t axis_bench_max_step_rate_hz(const MotionAxisHardwareConfig_t *hardware_config) {
    const uint64_t min_period_ns = (uint64_t)hardware_config->step_pulse_width_ns + hardware_config->step_low_time_ns;
    if (min_period_ns == 0U) {
        return 0U;
    }
    return (uint32_t)(AXIS_BENCH_NANOSECONDS_PER_SECOND / min_period_ns);
}

static AxisBenchStatus_t axis_bench_start_idle_axis(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionAxisHardwareStatus_t *hardware_status
) {
    MotionAxisHardwareStatus_t init_status = motion_axis_hardware_get_status(axis);
    if (init_status == MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED) {
        init_status = motion_axis_hardware_init(axis, hardware_config);
    }
    *hardware_status = init_status;
    if (init_status == MOTION_AXIS_HARDWARE_STATUS_STARTING || init_status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
        init_status == MOTION_AXIS_HARDWARE_STATUS_RUNNING) {
        return AXIS_BENCH_STATUS_BUSY;
    }
    if (init_status != MOTION_AXIS_HARDWARE_STATUS_READY && init_status != MOTION_AXIS_HARDWARE_STATUS_COMPLETE &&
        init_status != MOTION_AXIS_HARDWARE_STATUS_STOPPED) {
        return AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }

    *hardware_status = motion_axis_hardware_start_jog_profile(
        axis,
        signed_steps,
        initial_step_rate_hz,
        target_step_rate_hz,
        acceleration_steps_per_s2
    );
    if (*hardware_status != MOTION_AXIS_HARDWARE_STATUS_STARTING) {
        return AXIS_BENCH_STATUS_HARDWARE_ERROR;
    }
    return AXIS_BENCH_STATUS_STARTED;
}

AxisBenchStatus_t axis_bench_start_profile(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionAxisHardwareStatus_t *hardware_status
) {
    if (hardware_config == NULL || steps == 0U || steps > AXIS_BENCH_MAX_STEPS || initial_step_rate_hz == 0U ||
        target_step_rate_hz < initial_step_rate_hz ||
        (target_step_rate_hz > initial_step_rate_hz && acceleration_steps_per_s2 == 0U) || hardware_status == NULL ||
        target_step_rate_hz > axis_bench_max_step_rate_hz(hardware_config)) {
        return AXIS_BENCH_STATUS_INVALID_ARGUMENT;
    }

    const uint8_t toward_end = direction == AXIS_BENCH_TOWARD_END ? 1U : 0U;
    const int32_t signed_steps =
        toward_end == (hardware_config->positive_moves_toward_end != 0U) ? (int32_t)steps : -(int32_t)steps;

    /* StepGeneratorTask also starts X; with task switching paused the status check and the start cannot interleave
     * with its start. Interrupts keep running. */
    (void)osKernelLock();
    const AxisBenchStatus_t status = axis_bench_start_idle_axis(
        axis,
        hardware_config,
        signed_steps,
        initial_step_rate_hz,
        target_step_rate_hz,
        acceleration_steps_per_s2,
        hardware_status
    );
    (void)osKernelUnlock();
    return status;
}

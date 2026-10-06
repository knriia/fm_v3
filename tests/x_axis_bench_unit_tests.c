#include "motion/x_axis_bench.h"

#include <assert.h>
#include <stdio.h>

static MotionHardwareXStatus_t fake_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;
static MotionHardwareXStatus_t fake_init_status = MOTION_HARDWARE_X_STATUS_READY;
static MotionHardwareXStatus_t fake_start_status = MOTION_HARDWARE_X_STATUS_STARTING;
static uint32_t fake_init_calls;
static uint32_t fake_start_calls;
static int32_t fake_last_signed_steps;
static uint32_t fake_last_initial_rate_hz;
static uint32_t fake_last_target_rate_hz;
static uint32_t fake_last_acceleration;

MotionHardwareXStatus_t motion_hardware_x_init(const MotionHardwareXConfig_t *config) {
    assert(config != NULL);
    ++fake_init_calls;
    fake_status = fake_init_status;
    return fake_status;
}

MotionHardwareXStatus_t motion_hardware_x_get_status(void) { return fake_status; }

MotionHardwareXStatus_t motion_hardware_x_start_jog_profile(
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2
) {
    ++fake_start_calls;
    fake_last_signed_steps = signed_steps;
    fake_last_initial_rate_hz = initial_step_rate_hz;
    fake_last_target_rate_hz = target_step_rate_hz;
    fake_last_acceleration = acceleration_steps_per_s2;
    fake_status = fake_start_status;
    return fake_status;
}

static void reset_fakes(MotionHardwareXStatus_t status) {
    fake_status = status;
    fake_init_status = MOTION_HARDWARE_X_STATUS_READY;
    fake_start_status = MOTION_HARDWARE_X_STATUS_STARTING;
    fake_init_calls = 0U;
    fake_start_calls = 0U;
    fake_last_signed_steps = 0;
    fake_last_initial_rate_hz = 0U;
    fake_last_target_rate_hz = 0U;
    fake_last_acceleration = 0U;
}

static MotionHardwareXConfig_t make_config(void) {
    return (MotionHardwareXConfig_t){
        .step_active_high = 1U,
        .enable_active_high = 1U,
        .positive_direction_high = 1U,
        .positive_moves_toward_end = 1U,
        .start_limit_active_high = 0U,
        .end_limit_active_high = 0U,
        .limit_pull = MOTION_HARDWARE_X_LIMIT_PULL_UP,
        .step_pulse_width_ns = 5000U,
        .step_low_time_ns = 2500U,
        .direction_setup_ns = 5000U,
        .direction_hold_ns = 0U,
        .enable_setup_ns = 5000U,
        .enable_hold_ns = 0U,
    };
}

static void test_constant_rate_move_initializes_once_and_starts_jog(void) {
    reset_fakes(MOTION_HARDWARE_X_STATUS_UNINITIALIZED);
    const MotionHardwareXConfig_t config = make_config();
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;

    assert(x_axis_bench_start(&config, -3, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_STARTED);
    assert(hardware_status == MOTION_HARDWARE_X_STATUS_STARTING);
    assert(fake_init_calls == 1U);
    assert(fake_start_calls == 1U);
    assert(fake_last_signed_steps == -3);
    assert(fake_last_initial_rate_hz == 100U);
    assert(fake_last_target_rate_hz == 100U);
    assert(fake_last_acceleration == 0U);

    fake_status = MOTION_HARDWARE_X_STATUS_COMPLETE;
    assert(x_axis_bench_start(&config, 2, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_STARTED);
    assert(fake_init_calls == 1U);
}

static void test_profile_move_passes_profile(void) {
    reset_fakes(MOTION_HARDWARE_X_STATUS_READY);
    const MotionHardwareXConfig_t config = make_config();
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;

    assert(
        x_axis_bench_start_profile(&config, 500, 10U, 1000U, 2000U, &hardware_status) ==
        X_AXIS_BENCH_STATUS_STARTED
    );
    assert(fake_last_initial_rate_hz == 10U);
    assert(fake_last_target_rate_hz == 1000U);
    assert(fake_last_acceleration == 2000U);
}

static void test_invalid_arguments_do_not_touch_hardware(void) {
    reset_fakes(MOTION_HARDWARE_X_STATUS_READY);
    const MotionHardwareXConfig_t config = make_config();
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;

    assert(x_axis_bench_start(NULL, 1, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(x_axis_bench_start(&config, 1, 100U, NULL) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(x_axis_bench_start(&config, 0, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(x_axis_bench_start(&config, 1, 0U, &hardware_status) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(
        x_axis_bench_start(&config, (int32_t)X_AXIS_BENCH_MAX_STEPS + 1, 100U, &hardware_status) ==
        X_AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(x_axis_bench_start(&config, INT32_MIN, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(
        x_axis_bench_start_profile(&config, 1, 100U, 10U, 1000U, &hardware_status) ==
        X_AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(
        x_axis_bench_start_profile(&config, 1, 10U, 100U, 0U, &hardware_status) ==
        X_AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(fake_init_calls == 0U);
    assert(fake_start_calls == 0U);
}

static void test_rate_is_limited_by_driver_timing(void) {
    reset_fakes(MOTION_HARDWARE_X_STATUS_READY);
    const MotionHardwareXConfig_t config = make_config();
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;

    /* 5000 ns pulse + 2500 ns low time => at most 133333 Hz. */
    assert(x_axis_bench_start(&config, 1, 133333U, &hardware_status) == X_AXIS_BENCH_STATUS_STARTED);
    assert(x_axis_bench_start(&config, 1, 133334U, &hardware_status) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(
        x_axis_bench_start_profile(&config, 1, 10U, 133334U, 1000U, &hardware_status) ==
        X_AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(fake_start_calls == 1U);
}

static void test_zero_driver_timing_rejects_any_rate(void) {
    reset_fakes(MOTION_HARDWARE_X_STATUS_READY);
    MotionHardwareXConfig_t config = make_config();
    config.step_pulse_width_ns = 0U;
    config.step_low_time_ns = 0U;
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;

    assert(x_axis_bench_start(&config, 1, 1U, &hardware_status) == X_AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(fake_start_calls == 0U);
}

static void test_busy_fault_and_start_error(void) {
    const MotionHardwareXConfig_t config = make_config();
    MotionHardwareXStatus_t hardware_status = MOTION_HARDWARE_X_STATUS_UNINITIALIZED;

    const MotionHardwareXStatus_t busy_statuses[] = {
        MOTION_HARDWARE_X_STATUS_STARTING,
        MOTION_HARDWARE_X_STATUS_WAITING,
        MOTION_HARDWARE_X_STATUS_RUNNING,
    };
    for (size_t index = 0U; index < sizeof(busy_statuses) / sizeof(busy_statuses[0]); ++index) {
        reset_fakes(busy_statuses[index]);
        assert(x_axis_bench_start(&config, 1, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_BUSY);
        assert(hardware_status == busy_statuses[index]);
        assert(fake_start_calls == 0U);
    }

    reset_fakes(MOTION_HARDWARE_X_STATUS_FAULT_TIMING);
    assert(x_axis_bench_start(&config, 1, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_HARDWARE_ERROR);
    assert(hardware_status == MOTION_HARDWARE_X_STATUS_FAULT_TIMING);

    reset_fakes(MOTION_HARDWARE_X_STATUS_STOPPED);
    fake_start_status = MOTION_HARDWARE_X_STATUS_FAULT_TIMING;
    assert(x_axis_bench_start(&config, 1, 100U, &hardware_status) == X_AXIS_BENCH_STATUS_HARDWARE_ERROR);
    assert(hardware_status == MOTION_HARDWARE_X_STATUS_FAULT_TIMING);
    assert(fake_start_calls == 1U);
}

int main(void) {
    test_constant_rate_move_initializes_once_and_starts_jog();
    test_profile_move_passes_profile();
    test_invalid_arguments_do_not_touch_hardware();
    test_rate_is_limited_by_driver_timing();
    test_zero_driver_timing_rejects_any_rate();
    test_busy_fault_and_start_error();
    puts("X-axis bench unit tests passed");
    return 0;
}

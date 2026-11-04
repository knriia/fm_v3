#include "motion/axis_bench.h"

#include "cmsis_os.h"

#include <assert.h>
#include <stdio.h>

static MotionAxisHardwareStatus_t fake_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
static MotionAxisHardwareStatus_t fake_init_status = MOTION_AXIS_HARDWARE_STATUS_READY;
static MotionAxisHardwareStatus_t fake_start_status = MOTION_AXIS_HARDWARE_STATUS_STARTING;
static uint32_t fake_init_calls;
static uint32_t fake_start_calls;
static int32_t fake_last_signed_steps;
static uint32_t fake_last_initial_rate_hz;
static uint32_t fake_last_target_rate_hz;
static uint32_t fake_last_acceleration;
static uint32_t fake_kernel_lock_depth;
static uint32_t fake_kernel_lock_calls;
static uint32_t fake_started_unlocked;

int32_t osKernelLock(void) {
    ++fake_kernel_lock_calls;
    ++fake_kernel_lock_depth;
    return 0;
}

int32_t osKernelUnlock(void) {
    --fake_kernel_lock_depth;
    return 1;
}

/* Every hardware call must name the axis the bench was asked to move. */
static MotionAxis_t fake_expected_axis = MOTION_AXIS_X;

MotionAxisHardwareStatus_t motion_axis_hardware_init(MotionAxis_t axis, const MotionAxisHardwareConfig_t *config) {
    assert(axis == fake_expected_axis);
    assert(config != NULL);
    ++fake_init_calls;
    fake_status = fake_init_status;
    return fake_status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) {
    assert(axis == fake_expected_axis);
    return fake_status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_start_jog_profile(
    MotionAxis_t axis,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2
) {
    assert(axis == fake_expected_axis);
    ++fake_start_calls;
    if (fake_kernel_lock_depth == 0U) {
        ++fake_started_unlocked;
    }
    fake_last_signed_steps = signed_steps;
    fake_last_initial_rate_hz = initial_step_rate_hz;
    fake_last_target_rate_hz = target_step_rate_hz;
    fake_last_acceleration = acceleration_steps_per_s2;
    fake_status = fake_start_status;
    return fake_status;
}

static void reset_fakes(MotionAxisHardwareStatus_t status) {
    fake_expected_axis = MOTION_AXIS_X;
    fake_status = status;
    fake_init_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    fake_start_status = MOTION_AXIS_HARDWARE_STATUS_STARTING;
    fake_init_calls = 0U;
    fake_start_calls = 0U;
    fake_last_signed_steps = 0;
    fake_last_initial_rate_hz = 0U;
    fake_last_target_rate_hz = 0U;
    fake_last_acceleration = 0U;
}

static MotionAxisHardwareConfig_t make_config(void) {
    return (MotionAxisHardwareConfig_t){
        .step_active_high = 1U,
        .enable_active_high = 1U,
        .positive_direction_high = 1U,
        .positive_moves_toward_end = 1U,
        .start_limit_active_high = 0U,
        .end_limit_active_high = 0U,
        .limit_pull = MOTION_AXIS_HARDWARE_LIMIT_PULL_UP,
        .step_pulse_width_ns = 5000U,
        .step_low_time_ns = 2500U,
        .direction_setup_ns = 5000U,
        .direction_hold_ns = 0U,
        .enable_setup_ns = 5000U,
        .enable_hold_ns = 0U,
    };
}

static void test_constant_rate_move_initializes_once_and_starts_jog(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED);
    const MotionAxisHardwareConfig_t config = make_config();
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 3U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_STARTED);
    assert(hardware_status == MOTION_AXIS_HARDWARE_STATUS_STARTING);
    assert(fake_init_calls == 1U);
    assert(fake_start_calls == 1U);
    assert(fake_last_signed_steps == -3);
    assert(fake_last_initial_rate_hz == 100U);
    assert(fake_last_target_rate_hz == 100U);
    assert(fake_last_acceleration == 0U);

    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 2U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_STARTED);
    assert(fake_init_calls == 1U);
}

static void test_profile_move_passes_profile(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_READY);
    const MotionAxisHardwareConfig_t config = make_config();
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    assert(
        axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 500U, 10U, 1000U, 2000U, &hardware_status) ==
        AXIS_BENCH_STATUS_STARTED
    );
    assert(fake_last_initial_rate_hz == 10U);
    assert(fake_last_target_rate_hz == 1000U);
    assert(fake_last_acceleration == 2000U);
}

static void test_invalid_arguments_do_not_touch_hardware(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_READY);
    const MotionAxisHardwareConfig_t config = make_config();
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    assert(axis_bench_start_profile(MOTION_AXIS_X, NULL, AXIS_BENCH_TOWARD_END, 1U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 100U, 100U, 0U, NULL) == AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 0U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 0U, 0U, 0U, &hardware_status) == AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(
        axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, AXIS_BENCH_MAX_STEPS + 1U, 100U, 100U, 0U, &hardware_status) ==
        AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(
        axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 100U, 10U, 1000U, &hardware_status) ==
        AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(
        axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 10U, 100U, 0U, &hardware_status) ==
        AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(fake_init_calls == 0U);
    assert(fake_start_calls == 0U);
}

static void test_rate_is_limited_by_driver_timing(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_READY);
    const MotionAxisHardwareConfig_t config = make_config();
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    /* 5000 ns pulse + 2500 ns low time => at most 133333 Hz. */
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 133333U, 133333U, 0U, &hardware_status) == AXIS_BENCH_STATUS_STARTED);
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 133334U, 133334U, 0U, &hardware_status) == AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(
        axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 10U, 133334U, 1000U, &hardware_status) ==
        AXIS_BENCH_STATUS_INVALID_ARGUMENT
    );
    assert(fake_start_calls == 1U);
}

static void test_zero_driver_timing_rejects_any_rate(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_READY);
    MotionAxisHardwareConfig_t config = make_config();
    config.step_pulse_width_ns = 0U;
    config.step_low_time_ns = 0U;
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 1U, 1U, 0U, &hardware_status) == AXIS_BENCH_STATUS_INVALID_ARGUMENT);
    assert(fake_start_calls == 0U);
}

static void test_busy_fault_and_start_error(void) {
    const MotionAxisHardwareConfig_t config = make_config();
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    const MotionAxisHardwareStatus_t busy_statuses[] = {
        MOTION_AXIS_HARDWARE_STATUS_STARTING,
        MOTION_AXIS_HARDWARE_STATUS_WAITING,
        MOTION_AXIS_HARDWARE_STATUS_RUNNING,
    };
    for (size_t index = 0U; index < sizeof(busy_statuses) / sizeof(busy_statuses[0]); ++index) {
        reset_fakes(busy_statuses[index]);
        assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_BUSY);
        assert(hardware_status == busy_statuses[index]);
        assert(fake_start_calls == 0U);
    }

    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_HARDWARE_ERROR);
    assert(hardware_status == MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);

    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_STOPPED);
    fake_start_status = MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING;
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 1U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_HARDWARE_ERROR);
    assert(hardware_status == MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
    assert(fake_start_calls == 1U);
}

static void test_other_axis_is_passed_through(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED);
    fake_expected_axis = MOTION_AXIS_Y;
    const MotionAxisHardwareConfig_t config = make_config();
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    assert(axis_bench_start_profile(MOTION_AXIS_Y, &config, AXIS_BENCH_TOWARD_END, 7U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_STARTED);
    assert(fake_init_calls == 1U);
    assert(fake_last_signed_steps == 7);
}

/* With positive motion toward the start switch the signs flip. */
static void test_direction_follows_the_config(void) {
    reset_fakes(MOTION_AXIS_HARDWARE_STATUS_READY);
    MotionAxisHardwareConfig_t config = make_config();
    config.positive_moves_toward_end = 0U;
    MotionAxisHardwareStatus_t hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;

    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_END, 5U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_STARTED);
    assert(fake_last_signed_steps == -5);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    assert(axis_bench_start_profile(MOTION_AXIS_X, &config, AXIS_BENCH_TOWARD_START, 6U, 100U, 100U, 0U, &hardware_status) == AXIS_BENCH_STATUS_STARTED);
    assert(fake_last_signed_steps == 6);
}

int main(void) {
    test_constant_rate_move_initializes_once_and_starts_jog();
    test_profile_move_passes_profile();
    test_invalid_arguments_do_not_touch_hardware();
    test_rate_is_limited_by_driver_timing();
    test_zero_driver_timing_rejects_any_rate();
    test_busy_fault_and_start_error();
    test_other_axis_is_passed_through();
    test_direction_follows_the_config();
    assert(fake_kernel_lock_calls > 0U);
    assert(fake_kernel_lock_depth == 0U);
    assert(fake_started_unlocked == 0U);
    puts("Axis bench unit tests passed");
    return 0;
}

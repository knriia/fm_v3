#include "motion/x_axis_bench.h"
#include "motion/x_axis_calibration.h"

#include <stdint.h>
#include <stdio.h>

static uint32_t test_failures;
static MotionHardwareXStatus_t fake_status;
static MotionHardwareXStopReason_t fake_stop_reason;
static uint32_t fake_jog_steps_completed;
static int32_t fake_encoder_um;
static uint32_t fake_stop_requests;
static uint32_t fake_profile_starts;
static uint32_t fake_constant_starts;
static int32_t fake_last_steps;
static uint32_t fake_last_rate_hz;
static XAxisBenchStatus_t fake_profile_result;
static XAxisBenchStatus_t fake_constant_result;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

MotionHardwareXStatus_t motion_hardware_x_get_status(void) { return fake_status; }

MotionHardwareXStopReason_t motion_hardware_x_get_last_stop_reason(void) { return fake_stop_reason; }

uint32_t motion_hardware_x_get_last_jog_steps_completed(void) { return fake_jog_steps_completed; }

void motion_hardware_x_request_stop(void) { ++fake_stop_requests; }

int32_t encoder_ox_get_coordinate(void) { return fake_encoder_um; }

XAxisBenchStatus_t x_axis_bench_start_profile(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionHardwareXStatus_t *hardware_status
) {
    (void)hardware_config;
    (void)initial_step_rate_hz;
    (void)acceleration_steps_per_s2;
    ++fake_profile_starts;
    fake_last_steps = signed_steps;
    fake_last_rate_hz = target_step_rate_hz;
    *hardware_status = fake_status;
    if (fake_profile_result == X_AXIS_BENCH_STATUS_STARTED) {
        fake_status = MOTION_HARDWARE_X_STATUS_RUNNING;
    }
    return fake_profile_result;
}

XAxisBenchStatus_t x_axis_bench_start(
    const MotionHardwareXConfig_t *hardware_config,
    int32_t signed_steps,
    uint32_t step_rate_hz,
    MotionHardwareXStatus_t *hardware_status
) {
    (void)hardware_config;
    ++fake_constant_starts;
    fake_last_steps = signed_steps;
    fake_last_rate_hz = step_rate_hz;
    *hardware_status = fake_status;
    if (fake_constant_result == X_AXIS_BENCH_STATUS_STARTED) {
        fake_status = MOTION_HARDWARE_X_STATUS_RUNNING;
    }
    return fake_constant_result;
}

static MotionHardwareXConfig_t config_toward_end(uint8_t positive_moves_toward_end) {
    return (MotionHardwareXConfig_t){.positive_moves_toward_end = positive_moves_toward_end};
}

static void reset_fakes(void) {
    fake_status = MOTION_HARDWARE_X_STATUS_READY;
    fake_stop_reason = MOTION_HARDWARE_X_STOP_REASON_NONE;
    fake_jog_steps_completed = 0U;
    fake_encoder_um = 0;
    fake_stop_requests = 0U;
    fake_profile_starts = 0U;
    fake_constant_starts = 0U;
    fake_last_steps = 0;
    fake_last_rate_hz = 0U;
    fake_profile_result = X_AXIS_BENCH_STATUS_STARTED;
    fake_constant_result = X_AXIS_BENCH_STATUS_STARTED;
}

/* Drives the procedure up to the measuring move with the scale at `end_um` on the end switch. */
static void reach_measure(XAxisCalibration_t *calibration, const MotionHardwareXConfig_t *config, int32_t end_um) {
    AxisCalibration_t result = {0};
    expect_u32(x_axis_calibration_begin(calibration, config), X_AXIS_CALIBRATION_RESULT_RUNNING, "seek starts");
    fake_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    fake_stop_reason = MOTION_HARDWARE_X_STOP_REASON_END_LIMIT;
    fake_encoder_um = end_um;
    expect_u32(
        x_axis_calibration_poll(calibration, config, &result),
        X_AXIS_CALIBRATION_RESULT_RUNNING,
        "end switch starts the measuring move"
    );
}

static void test_successful_calibration(void) {
    reset_fakes();
    const MotionHardwareXConfig_t config = config_toward_end(1U);
    XAxisCalibration_t calibration = {0};
    AxisCalibration_t result = {0};

    expect_u32(x_axis_calibration_is_active(&calibration), 0U, "idle procedure is not active");
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_NONE, "idle poll");
    expect_u32(x_axis_calibration_begin(&calibration, &config), X_AXIS_CALIBRATION_RESULT_RUNNING, "seek starts");
    expect_u32(x_axis_calibration_is_active(&calibration), 1U, "seek makes the procedure active");
    expect_u32((uint32_t)fake_last_steps, X_AXIS_BENCH_MAX_STEPS, "seek drives toward the end switch");
    expect_u32(fake_last_rate_hz, X_AXIS_CALIBRATION_SEEK_STEP_RATE_HZ, "seek uses the seek rate");
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_RUNNING, "seek runs");
    fake_status = MOTION_HARDWARE_X_STATUS_STARTING;
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_RUNNING, "seek starting");
    fake_status = MOTION_HARDWARE_X_STATUS_WAITING;
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_RUNNING, "seek waiting");

    fake_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    fake_stop_reason = MOTION_HARDWARE_X_STOP_REASON_END_LIMIT;
    fake_encoder_um = 301000;
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_RUNNING, "measuring");
    expect_u32(calibration.travel_um, 301000U, "travel is the scale reading at the end switch");
    expect_u32((uint32_t)fake_last_steps, (uint32_t)-(int32_t)X_AXIS_CALIBRATION_MEASURE_STEPS, "measure heads back");
    expect_u32(fake_last_rate_hz, X_AXIS_CALIBRATION_MEASURE_STEP_RATE_HZ, "measure uses the slow rate");
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_RUNNING, "measure runs");

    fake_status = MOTION_HARDWARE_X_STATUS_COMPLETE;
    fake_jog_steps_completed = X_AXIS_CALIBRATION_MEASURE_STEPS;
    fake_encoder_um = 300988;
    expect_u32(x_axis_calibration_poll(&calibration, &config, &result), X_AXIS_CALIBRATION_RESULT_OK, "calibration ends");
    expect_u32(result.steps, X_AXIS_CALIBRATION_MEASURE_STEPS, "result holds the measured steps");
    expect_u32(result.distance_um, 12U, "result holds the measured distance");
    expect_u32(result.travel_um, 301000U, "result holds the travel");
    expect_u32(x_axis_calibration_is_active(&calibration), 0U, "finished procedure is idle");

    reset_fakes();
    const MotionHardwareXConfig_t reversed = config_toward_end(0U);
    calibration = (XAxisCalibration_t){0};
    reach_measure(&calibration, &reversed, -50000);
    expect_u32((uint32_t)fake_last_steps, X_AXIS_CALIBRATION_MEASURE_STEPS, "reversed motor measures with positive steps");
    expect_u32(calibration.travel_um, 50000U, "negative scale reading gives the travel");
    fake_status = MOTION_HARDWARE_X_STATUS_COMPLETE;
    fake_jog_steps_completed = X_AXIS_CALIBRATION_MEASURE_STEPS;
    fake_encoder_um = -49990;
    expect_u32(x_axis_calibration_poll(&calibration, &reversed, &result), X_AXIS_CALIBRATION_RESULT_OK, "reversed ends");
    expect_u32(result.distance_um, 10U, "distance is measured in either direction");
}

static void test_failures_during_seek(void) {
    const MotionHardwareXConfig_t config = config_toward_end(1U);
    XAxisCalibration_t calibration = {0};
    AxisCalibration_t result = {0};

    reset_fakes();
    fake_profile_result = X_AXIS_BENCH_STATUS_BUSY;
    expect_u32(x_axis_calibration_begin(&calibration, &config), X_AXIS_CALIBRATION_RESULT_MOVE_FAILED, "seek start fails");
    expect_u32(x_axis_calibration_is_active(&calibration), 0U, "failed start leaves it idle");

    reset_fakes();
    (void)x_axis_calibration_begin(&calibration, &config);
    fake_status = MOTION_HARDWARE_X_STATUS_COMPLETE;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_END_NOT_REACHED,
        "seek that runs out of steps fails"
    );

    reset_fakes();
    (void)x_axis_calibration_begin(&calibration, &config);
    fake_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    fake_stop_reason = MOTION_HARDWARE_X_STOP_REASON_START_LIMIT;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_END_NOT_REACHED,
        "seek stopped by the start switch fails"
    );

    reset_fakes();
    (void)x_axis_calibration_begin(&calibration, &config);
    fake_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    fake_stop_reason = MOTION_HARDWARE_X_STOP_REASON_END_LIMIT;
    fake_encoder_um = 0;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_NO_DISTANCE,
        "end switch at the scale zero fails"
    );

    reset_fakes();
    (void)x_axis_calibration_begin(&calibration, &config);
    fake_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    fake_stop_reason = MOTION_HARDWARE_X_STOP_REASON_END_LIMIT;
    fake_encoder_um = 1000;
    fake_constant_result = X_AXIS_BENCH_STATUS_HARDWARE_ERROR;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_MOVE_FAILED,
        "measuring move that cannot start fails"
    );
}

static void test_failures_during_measure(void) {
    const MotionHardwareXConfig_t config = config_toward_end(1U);
    XAxisCalibration_t calibration = {0};
    AxisCalibration_t result = {0};

    reset_fakes();
    reach_measure(&calibration, &config, 1000);
    fake_status = MOTION_HARDWARE_X_STATUS_STOPPED;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_MOVE_FAILED,
        "interrupted measuring move fails"
    );

    reset_fakes();
    reach_measure(&calibration, &config, 1000);
    fake_status = MOTION_HARDWARE_X_STATUS_COMPLETE;
    fake_jog_steps_completed = X_AXIS_CALIBRATION_MEASURE_STEPS - 1U;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_MOVE_FAILED,
        "measuring move with lost steps fails"
    );

    reset_fakes();
    reach_measure(&calibration, &config, 1000);
    fake_status = MOTION_HARDWARE_X_STATUS_COMPLETE;
    fake_jog_steps_completed = X_AXIS_CALIBRATION_MEASURE_STEPS;
    expect_u32(
        x_axis_calibration_poll(&calibration, &config, &result),
        X_AXIS_CALIBRATION_RESULT_NO_DISTANCE,
        "measuring move the scale did not see fails"
    );
}

static void test_abort(void) {
    const MotionHardwareXConfig_t config = config_toward_end(1U);
    XAxisCalibration_t calibration = {0};

    reset_fakes();
    x_axis_calibration_abort(&calibration);
    expect_u32(fake_stop_requests, 0U, "idle abort does not stop the axis");

    (void)x_axis_calibration_begin(&calibration, &config);
    x_axis_calibration_abort(&calibration);
    expect_u32(fake_stop_requests, 1U, "active abort stops the axis");
    expect_u32(x_axis_calibration_is_active(&calibration), 0U, "aborted procedure is idle");
}

int main(void) {
    test_successful_calibration();
    test_failures_during_seek();
    test_failures_during_measure();
    test_abort();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "X axis calibration unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("X axis calibration unit tests passed.\n");
    return 0;
}

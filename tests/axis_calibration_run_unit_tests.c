#include "motion/axis_bench.h"
#include "motion/axis_calibration_run.h"

#include <stdint.h>
#include <stdio.h>

static uint32_t test_failures;
static MotionAxisHardwareStatus_t fake_status;
static MotionAxisHardwareStopReason_t fake_stop_reason;
static uint32_t fake_jog_steps_completed;
static int32_t fake_encoder_um;
static uint32_t fake_stop_requests;
static uint32_t fake_profile_starts;
static int32_t fake_last_steps;
static uint32_t fake_last_initial_rate_hz;
static uint32_t fake_last_rate_hz;
static uint32_t fake_last_acceleration;
static AxisBenchStatus_t fake_profile_result;

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

/* Every hardware call must name the axis the procedure was started for. */
static MotionAxis_t fake_expected_axis;
static uint32_t fake_wrong_axis_calls;

static void check_axis(MotionAxis_t axis) {
    if (axis != fake_expected_axis) {
        ++fake_wrong_axis_calls;
    }
}

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) {
    check_axis(axis);
    return fake_status;
}

MotionAxisHardwareStopReason_t motion_axis_hardware_get_last_stop_reason(MotionAxis_t axis) {
    check_axis(axis);
    return fake_stop_reason;
}

uint32_t motion_axis_hardware_get_last_jog_steps_completed(MotionAxis_t axis) {
    check_axis(axis);
    return fake_jog_steps_completed;
}

void motion_axis_hardware_request_stop(MotionAxis_t axis) {
    check_axis(axis);
    ++fake_stop_requests;
}

static int32_t fake_scale_reader(void) { return fake_encoder_um; }

/* The switch point latched by the hardware; the axis then brakes on to fake_encoder_um. */
static int32_t fake_limit_trigger_um;

int32_t motion_axis_hardware_get_limit_trigger_position(MotionAxis_t axis) {
    check_axis(axis);
    return fake_limit_trigger_um;
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
    check_axis(axis);
    ++fake_profile_starts;
    const uint8_t toward_end = direction == AXIS_BENCH_TOWARD_END ? 1U : 0U;
    fake_last_steps =
        toward_end == (hardware_config->positive_moves_toward_end != 0U) ? (int32_t)steps : -(int32_t)steps;
    fake_last_initial_rate_hz = initial_step_rate_hz;
    fake_last_rate_hz = target_step_rate_hz;
    fake_last_acceleration = acceleration_steps_per_s2;
    *hardware_status = fake_status;
    if (fake_profile_result == AXIS_BENCH_STATUS_STARTED) {
        fake_status = MOTION_AXIS_HARDWARE_STATUS_RUNNING;
    }
    return fake_profile_result;
}

static MotionAxisHardwareConfig_t config_toward_end(uint8_t positive_moves_toward_end) {
    return (MotionAxisHardwareConfig_t){
        .positive_moves_toward_end = positive_moves_toward_end,
        .position_reader = fake_scale_reader,
    };
}

static void reset_fakes(void) {
    fake_expected_axis = MOTION_AXIS_X;
    fake_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    fake_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    fake_jog_steps_completed = 0U;
    fake_encoder_um = 0;
    fake_limit_trigger_um = 0;
    fake_stop_requests = 0U;
    fake_profile_starts = 0U;
    fake_last_steps = 0;
    fake_last_initial_rate_hz = 0U;
    fake_last_rate_hz = 0U;
    fake_last_acceleration = 0U;
    fake_profile_result = AXIS_BENCH_STATUS_STARTED;
}

/* Drives the procedure up to the take-up move with the scale at `end_um` on the end switch. */
static void reach_take_up(AxisCalibrationRun_t *calibration, const MotionAxisHardwareConfig_t *config, int32_t end_um) {
    AxisCalibration_t result = {0};
    expect_u32(axis_calibration_run_begin(calibration, fake_expected_axis, config), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "seek starts");
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    fake_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    fake_limit_trigger_um = end_um;
    fake_encoder_um = end_um;
    expect_u32(
        axis_calibration_run_poll(calibration, config, &result),
        AXIS_CALIBRATION_RUN_RESULT_RUNNING,
        "end switch starts the take-up move"
    );
}

/* Continues from the take-up: it completes with the scale at `start_um`, which starts the measuring move. */
static void reach_measure(
    AxisCalibrationRun_t *calibration,
    const MotionAxisHardwareConfig_t *config,
    int32_t end_um,
    int32_t start_um
) {
    AxisCalibration_t result = {0};
    reach_take_up(calibration, config, end_um);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_TAKE_UP_STEPS;
    fake_encoder_um = start_um;
    expect_u32(
        axis_calibration_run_poll(calibration, config, &result),
        AXIS_CALIBRATION_RUN_RESULT_RUNNING,
        "finished take-up starts the measuring move"
    );
}

static void test_successful_calibration(void) {
    reset_fakes();
    const MotionAxisHardwareConfig_t config = config_toward_end(1U);
    AxisCalibrationRun_t calibration = {0};
    AxisCalibration_t result = {0};

    expect_u32(axis_calibration_run_is_active(&calibration), 0U, "idle procedure is not active");
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_NONE, "idle poll");
    expect_u32(axis_calibration_run_begin(&calibration, fake_expected_axis, &config), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "seek starts");
    expect_u32(axis_calibration_run_is_active(&calibration), 1U, "seek makes the procedure active");
    expect_u32((uint32_t)fake_last_steps, AXIS_BENCH_MAX_STEPS, "seek drives toward the end switch");
    expect_u32(fake_last_rate_hz, AXIS_CALIBRATION_RUN_SEEK_STEP_RATE_HZ, "seek uses the seek rate");
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "seek runs");
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STARTING;
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "seek starting");
    fake_status = MOTION_AXIS_HARDWARE_STATUS_WAITING;
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "seek waiting");

    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    fake_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    fake_limit_trigger_um = 179500;
    /* The seek brakes about 1 mm past the switch; the travel still ends where the switch fired. */
    fake_encoder_um = 180400;
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "take-up");
    expect_u32(calibration.travel_um, 179500U, "travel is the scale reading latched when the end switch fired");
    expect_u32(calibration.phase, AXIS_CALIBRATION_RUN_PHASE_TAKE_UP, "the slack is taken up first");
    expect_u32((uint32_t)fake_last_steps, (uint32_t)-(int32_t)AXIS_CALIBRATION_RUN_TAKE_UP_STEPS, "take-up heads back");
    expect_u32(fake_last_initial_rate_hz, AXIS_CALIBRATION_RUN_MEASURE_INITIAL_STEP_RATE_HZ, "take-up ramps up");
    expect_u32(fake_last_rate_hz, AXIS_CALIBRATION_RUN_MEASURE_STEP_RATE_HZ, "take-up uses the measuring rate");
    expect_u32(
        fake_last_acceleration,
        AXIS_CALIBRATION_RUN_MEASURE_ACCELERATION_STEPS_PER_S2,
        "take-up uses the measuring acceleration"
    );
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "take-up runs");

    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_TAKE_UP_STEPS;
    fake_encoder_um = 178480;
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "measuring");
    expect_u32(calibration.phase, AXIS_CALIBRATION_RUN_PHASE_MEASURE, "the measurement follows the take-up");
    expect_u32((uint32_t)calibration.measure_start_um, 178480U, "the measurement starts after the slack");
    expect_u32((uint32_t)fake_last_steps, (uint32_t)-(int32_t)AXIS_CALIBRATION_RUN_MEASURE_STEPS, "measure heads back");
    expect_u32(fake_last_rate_hz, AXIS_CALIBRATION_RUN_MEASURE_STEP_RATE_HZ, "measure uses the measuring rate");
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_RUNNING, "measure runs");

    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_MEASURE_STEPS;
    fake_encoder_um = 153230;
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_OK, "calibration ends");
    expect_u32(result.steps, AXIS_CALIBRATION_RUN_MEASURE_STEPS, "result holds the measured steps");
    expect_u32(result.distance_um, 25250U, "result holds the distance after the take-up");
    expect_u32(result.travel_um, 179500U, "result holds the travel");
    expect_u32(axis_calibration_run_is_active(&calibration), 0U, "finished procedure is idle");

    reset_fakes();
    const MotionAxisHardwareConfig_t reversed = config_toward_end(0U);
    calibration = (AxisCalibrationRun_t){0};
    reach_take_up(&calibration, &reversed, -50000);
    expect_u32((uint32_t)fake_last_steps, AXIS_CALIBRATION_RUN_TAKE_UP_STEPS, "reversed motor takes up with positive steps");
    expect_u32(calibration.travel_um, 50000U, "negative scale reading gives the travel");
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_TAKE_UP_STEPS;
    fake_encoder_um = -49000;
    (void)axis_calibration_run_poll(&calibration, &reversed, &result);
    expect_u32((uint32_t)fake_last_steps, AXIS_CALIBRATION_RUN_MEASURE_STEPS, "reversed motor measures with positive steps");
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_MEASURE_STEPS;
    fake_encoder_um = -23750;
    expect_u32(axis_calibration_run_poll(&calibration, &reversed, &result), AXIS_CALIBRATION_RUN_RESULT_OK, "reversed ends");
    expect_u32(result.distance_um, 25250U, "distance is measured in either direction");
}

static void test_failures_during_seek(void) {
    const MotionAxisHardwareConfig_t config = config_toward_end(1U);
    AxisCalibrationRun_t calibration = {0};
    AxisCalibration_t result = {0};

    reset_fakes();
    fake_profile_result = AXIS_BENCH_STATUS_BUSY;
    expect_u32(axis_calibration_run_begin(&calibration, fake_expected_axis, &config), AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED, "seek start fails");
    expect_u32(axis_calibration_run_is_active(&calibration), 0U, "failed start leaves it idle");

    reset_fakes();
    (void)axis_calibration_run_begin(&calibration, fake_expected_axis, &config);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_END_NOT_REACHED,
        "seek that runs out of steps fails"
    );

    reset_fakes();
    (void)axis_calibration_run_begin(&calibration, fake_expected_axis, &config);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    fake_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_END_NOT_REACHED,
        "seek stopped by the start switch fails"
    );

    reset_fakes();
    (void)axis_calibration_run_begin(&calibration, fake_expected_axis, &config);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    fake_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    fake_limit_trigger_um = 0;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE,
        "end switch at the scale zero fails"
    );

    reset_fakes();
    (void)axis_calibration_run_begin(&calibration, fake_expected_axis, &config);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    fake_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    fake_limit_trigger_um = 1000;
    fake_profile_result = AXIS_BENCH_STATUS_HARDWARE_ERROR;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
        "take-up move that cannot start fails"
    );
}

static void test_failures_during_take_up(void) {
    const MotionAxisHardwareConfig_t config = config_toward_end(1U);
    AxisCalibrationRun_t calibration = {0};
    AxisCalibration_t result = {0};

    reset_fakes();
    reach_take_up(&calibration, &config, 100000);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
        "interrupted take-up fails"
    );

    reset_fakes();
    reach_take_up(&calibration, &config, 100000);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_TAKE_UP_STEPS - 1U;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
        "take-up with lost steps fails"
    );

    reset_fakes();
    reach_take_up(&calibration, &config, 100000);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_TAKE_UP_STEPS;
    fake_profile_result = AXIS_BENCH_STATUS_HARDWARE_ERROR;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
        "measuring move that cannot start fails"
    );
}

static void test_failures_during_measure(void) {
    const MotionAxisHardwareConfig_t config = config_toward_end(1U);
    AxisCalibrationRun_t calibration = {0};
    AxisCalibration_t result = {0};

    reset_fakes();
    reach_measure(&calibration, &config, 100000, 99000);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_STOPPED;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
        "interrupted measuring move fails"
    );

    reset_fakes();
    reach_measure(&calibration, &config, 100000, 99000);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_MEASURE_STEPS - 1U;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED,
        "measuring move with lost steps fails"
    );

    reset_fakes();
    reach_measure(&calibration, &config, 100000, 99000);
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_MEASURE_STEPS;
    expect_u32(
        axis_calibration_run_poll(&calibration, &config, &result),
        AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE,
        "measuring move the scale did not see fails"
    );
}

static void test_abort(void) {
    const MotionAxisHardwareConfig_t config = config_toward_end(1U);
    AxisCalibrationRun_t calibration = {0};

    reset_fakes();
    axis_calibration_run_abort(&calibration);
    expect_u32(fake_stop_requests, 0U, "idle abort does not stop the axis");

    (void)axis_calibration_run_begin(&calibration, fake_expected_axis, &config);
    axis_calibration_run_abort(&calibration);
    expect_u32(fake_stop_requests, 1U, "active abort stops the axis");
    expect_u32(axis_calibration_run_is_active(&calibration), 0U, "aborted procedure is idle");
}

/* The same procedure drives Y: every call goes to Y, and an aborted Y run stays bound to Y. */
static void test_other_axis(void) {
    const MotionAxisHardwareConfig_t config = config_toward_end(0U);
    AxisCalibrationRun_t calibration = {0};
    AxisCalibration_t result = {0};

    reset_fakes();
    fake_expected_axis = MOTION_AXIS_Y;
    reach_measure(&calibration, &config, -100000, -99000);
    expect_u32((uint32_t)fake_last_steps, AXIS_CALIBRATION_RUN_MEASURE_STEPS, "Y measures back toward its start");
    fake_status = MOTION_AXIS_HARDWARE_STATUS_COMPLETE;
    fake_jog_steps_completed = AXIS_CALIBRATION_RUN_MEASURE_STEPS;
    fake_encoder_um = -74000;
    expect_u32(axis_calibration_run_poll(&calibration, &config, &result), AXIS_CALIBRATION_RUN_RESULT_OK, "Y result");
    expect_u32(result.distance_um, 25000U, "Y distance comes from the Y scale reader");
    expect_u32(result.travel_um, 100000U, "Y travel comes from the Y switch point");
    expect_u32(calibration.axis, MOTION_AXIS_Y, "finished run stays bound to Y");

    (void)axis_calibration_run_begin(&calibration, MOTION_AXIS_Y, &config);
    axis_calibration_run_abort(&calibration);
    expect_u32(calibration.axis, MOTION_AXIS_Y, "aborted run stays bound to Y");
    expect_u32(fake_wrong_axis_calls, 0U, "no call reaches another axis");
}

int main(void) {
    test_successful_calibration();
    test_failures_during_seek();
    test_failures_during_take_up();
    test_failures_during_measure();
    test_abort();
    test_other_axis();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Axis calibration run unit tests failed: %u\n", (unsigned int)test_failures);
        return 1;
    }
    (void)printf("Axis calibration run unit tests passed.\n");
    return 0;
}

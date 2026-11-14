#include "motion/step_stream_feeder.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_HALF_RECORDS 8U
#define TEST_PULSE_TICKS 2500U
#define TEST_IDLE_TICKS 7000U
#define TEST_QUEUE_SIZE 16U
#define TEST_LANES 2U
#define TEST_WORDS (TEST_HALF_RECORDS * STEP_STREAM_GENERATOR_RECORD_WORDS)

static uint32_t test_failures;

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

/* Fake lanes: two halves each, EMPTY after prepare; the test releases a half the way the DMA interrupt would. */
static uint16_t fake_halves[TEST_LANES][2][TEST_WORDS];
static uint8_t fake_empty[TEST_LANES][2];
static uint32_t fake_prepare_calls;
static uint32_t fake_start_calls;
static uint32_t fake_stop_calls;
static uint32_t fake_direction_calls;
static uint8_t fake_direction_set[TEST_LANES];
static uint8_t fake_direction_value[TEST_LANES];
static uint32_t fake_direction_lane_log[8];
static uint32_t fake_last_startup_ns;
static uint8_t fake_fail_prepare;
static uint8_t fake_fail_acquire;
static uint8_t fake_fail_commit;
static uint8_t fake_fail_direction;
static uint8_t fake_fail_start;
static uint8_t fake_failing_lane;
static uint8_t fake_bad_half;
static uint8_t fake_null_records;

/* Fake plan buffer: the queue, its epoch and the calls that the feeder makes on it. */
static MotionPlanBlock_t fake_plan[TEST_QUEUE_SIZE];
static uint32_t fake_plan_count;
static uint32_t fake_epoch;
static uint32_t fake_discarded_total;
static uint32_t fake_peek_calls;
static StepStreamFeederPlanResult_t fake_peek_forced_result;
static uint32_t fake_peek_forced_after;
static StepStreamFeederPlanResult_t fake_discard_forced_result;
/* A block that the plan buffer receives right after the given peek call, the way MotionTask pushes in between. */
static uint8_t fake_push_after_first_end;
static uint8_t fake_push_fired;
static MotionPlanBlock_t fake_push_block;
/* A clear of the plan buffer right after the given peek call (0: none), the way a planner error does it. */
static uint32_t fake_clear_after_peek_call;

static StepStreamFeederAxisResult_t fake_prepare(void) {
    ++fake_prepare_calls;
    if (fake_fail_prepare != 0U) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    memset(fake_empty, 1, sizeof(fake_empty));
    return STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t fake_acquire_half(uint32_t lane, uint8_t *half, uint16_t **records) {
    if (fake_fail_acquire != 0U && fake_failing_lane == lane) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    for (uint8_t index = 0U; index < 2U; ++index) {
        if (fake_empty[lane][index] != 0U) {
            *half = fake_bad_half != 0U ? 2U : index;
            *records = fake_null_records != 0U ? NULL : fake_halves[lane][index];
            return STEP_STREAM_FEEDER_AXIS_OK;
        }
    }
    return STEP_STREAM_FEEDER_AXIS_NO_REQUEST;
}

static StepStreamFeederAxisResult_t fake_commit_half(uint32_t lane, uint8_t half) {
    if (fake_fail_commit != 0U && fake_failing_lane == lane) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    fake_empty[lane][half] = 0U;
    return STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t fake_set_direction(uint32_t lane, uint8_t positive) {
    if (fake_direction_calls < sizeof(fake_direction_lane_log) / sizeof(fake_direction_lane_log[0])) {
        fake_direction_lane_log[fake_direction_calls] = lane;
    }
    ++fake_direction_calls;
    fake_direction_set[lane] = 1U;
    fake_direction_value[lane] = positive;
    return fake_fail_direction != 0U && fake_failing_lane == lane ? STEP_STREAM_FEEDER_AXIS_ERROR
                                                                  : STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t fake_start(uint32_t startup_period_ns) {
    ++fake_start_calls;
    fake_last_startup_ns = startup_period_ns;
    return fake_fail_start != 0U ? STEP_STREAM_FEEDER_AXIS_ERROR : STEP_STREAM_FEEDER_AXIS_OK;
}

static void fake_stop(void) { ++fake_stop_calls; }

static uint32_t fake_plan_epoch(void) { return fake_epoch; }

static void queue_block(const MotionPlanBlock_t *block);

static StepStreamFeederPlanResult_t fake_peek_block_now(uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block) {
    if (fake_peek_forced_result != STEP_STREAM_FEEDER_PLAN_OK && fake_peek_calls > fake_peek_forced_after) {
        return fake_peek_forced_result;
    }
    if (epoch != fake_epoch) {
        return STEP_STREAM_FEEDER_PLAN_STALE;
    }
    if (offset >= fake_plan_count) {
        return STEP_STREAM_FEEDER_PLAN_NONE;
    }
    *block = fake_plan[offset];
    return STEP_STREAM_FEEDER_PLAN_OK;
}

static StepStreamFeederPlanResult_t fake_peek_block(uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block) {
    ++fake_peek_calls;
    const StepStreamFeederPlanResult_t result = fake_peek_block_now(epoch, offset, block);
    if (fake_push_after_first_end != 0U && result == STEP_STREAM_FEEDER_PLAN_NONE) {
        fake_push_after_first_end = 0U;
        fake_push_fired = 1U;
        queue_block(&fake_push_block);
    }
    if (fake_clear_after_peek_call != 0U && fake_peek_calls == fake_clear_after_peek_call) {
        ++fake_epoch;
        fake_plan_count = 0U;
    }
    return result;
}

static StepStreamFeederPlanResult_t fake_discard_blocks(uint32_t epoch, uint32_t count) {
    if (fake_discard_forced_result != STEP_STREAM_FEEDER_PLAN_OK) {
        return fake_discard_forced_result;
    }
    if (epoch != fake_epoch) {
        return STEP_STREAM_FEEDER_PLAN_STALE;
    }
    if (count > fake_plan_count) {
        return STEP_STREAM_FEEDER_PLAN_ERROR;
    }
    memmove(fake_plan, &fake_plan[count], (fake_plan_count - count) * sizeof(fake_plan[0]));
    fake_plan_count -= count;
    fake_discarded_total += count;
    return STEP_STREAM_FEEDER_PLAN_OK;
}

static void queue_block(const MotionPlanBlock_t *block) {
    fake_plan[fake_plan_count] = *block;
    ++fake_plan_count;
}

/* 100 kHz step rate: one step every 10000 ns, i.e. one 10000-tick record at a 1 GHz timer clock. */
static MotionPlanBlock_t make_block3(int64_t x_steps, int64_t y_steps, int64_t z_steps) {
    MotionPlanBlock_t block = {0};
    const int64_t steps[3] = {x_steps, y_steps, z_steps};
    uint64_t master = 0U;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        const uint64_t magnitude = (uint64_t)(steps[axis] < 0 ? -steps[axis] : steps[axis]);
        block.step_delta[axis] = steps[axis];
        master = magnitude > master ? magnitude : master;
        block.moving_axes_mask = (uint8_t)(block.moving_axes_mask | (steps[axis] != 0 ? 1U << axis : 0U));
        block.positive_direction_mask = (uint8_t)(block.positive_direction_mask | (steps[axis] > 0 ? 1U << axis : 0U));
    }
    block.master_step_count = master;
    block.step_event_rate_numerator = 100000U;
    block.step_event_rate_denominator = 1U;
    return block;
}

static MotionPlanBlock_t make_block(int64_t x_steps, int64_t y_steps) { return make_block3(x_steps, y_steps, 0); }

static StepStreamFeederConfig_t make_config(uint32_t lane_count) {
    StepStreamFeederConfig_t config = {
        .generator =
            {
                .timer_clock_hz = 1000000000U,
                .pulse_ticks = TEST_PULSE_TICKS,
                .low_ticks = 2500U,
                .min_record_ticks = 100U,
            },
        .lane_count = lane_count,
        .startup_period_ns = 5000U,
        .idle_record_ticks = TEST_IDLE_TICKS,
        .half_record_count = TEST_HALF_RECORDS,
    };
    /* Lane 0 drives X, lane 1 drives Y. */
    config.axis_index[0] = 0U;
    config.axis_index[1] = 1U;
    return config;
}

static StepStreamFeederPorts_t make_ports(void) {
    return (StepStreamFeederPorts_t){
        .prepare = fake_prepare,
        .acquire_half = fake_acquire_half,
        .commit_half = fake_commit_half,
        .set_direction = fake_set_direction,
        .start = fake_start,
        .stop = fake_stop,
        .plan_epoch = fake_plan_epoch,
        .peek_block = fake_peek_block,
        .discard_blocks = fake_discard_blocks,
    };
}

static void reset_fakes(void) {
    memset(fake_halves, 0, sizeof(fake_halves));
    memset(fake_empty, 0, sizeof(fake_empty));
    memset(fake_direction_set, 0, sizeof(fake_direction_set));
    memset(fake_direction_value, 0xFF, sizeof(fake_direction_value));
    memset(fake_direction_lane_log, 0xFF, sizeof(fake_direction_lane_log));
    memset(fake_plan, 0, sizeof(fake_plan));
    fake_prepare_calls = 0U;
    fake_start_calls = 0U;
    fake_stop_calls = 0U;
    fake_direction_calls = 0U;
    fake_last_startup_ns = 0U;
    fake_fail_prepare = 0U;
    fake_fail_acquire = 0U;
    fake_fail_commit = 0U;
    fake_fail_direction = 0U;
    fake_fail_start = 0U;
    fake_failing_lane = 0U;
    fake_bad_half = 0U;
    fake_null_records = 0U;
    fake_plan_count = 0U;
    fake_epoch = 5U;
    fake_discarded_total = 0U;
    fake_peek_calls = 0U;
    fake_peek_forced_result = STEP_STREAM_FEEDER_PLAN_OK;
    fake_peek_forced_after = 0U;
    fake_discard_forced_result = STEP_STREAM_FEEDER_PLAN_OK;
    fake_push_after_first_end = 0U;
    fake_push_fired = 0U;
    fake_clear_after_peek_call = 0U;
    memset(&fake_push_block, 0, sizeof(fake_push_block));
}

static void setup_lanes(StepStreamFeeder_t *feeder, uint32_t lane_count) {
    reset_fakes();
    const StepStreamFeederConfig_t config = make_config(lane_count);
    const StepStreamFeederPorts_t ports = make_ports();
    expect_u32(step_stream_feeder_init(feeder, &config, &ports), STEP_STREAM_FEEDER_STATUS_OK, "setup init");
}

/* One lane: X. */
static void setup(StepStreamFeeder_t *feeder) { setup_lanes(feeder, 1U); }

static void release(uint32_t lane, uint8_t half) { fake_empty[lane][half] = 1U; }

static uint32_t count_step_records_of(uint32_t lane, uint32_t half) {
    uint32_t steps = 0U;
    for (uint32_t index = 0U; index < TEST_HALF_RECORDS; ++index) {
        if (fake_halves[lane][half][index * 3U + 2U] != 0U) {
            ++steps;
        }
    }
    return steps;
}

static uint32_t count_step_records(uint32_t half) { return count_step_records_of(0U, half); }

/* Time of the records that precede the first idle record of the half; both lanes must agree on it for the same
 * blocks. */
static uint32_t block_ticks_of(uint32_t lane, uint32_t half) {
    uint32_t ticks = 0U;
    for (uint32_t index = 0U; index < TEST_HALF_RECORDS; ++index) {
        const uint16_t *record = &fake_halves[lane][half][index * 3U];
        if (record[2] == 0U && (uint32_t)record[0] + 1U == TEST_IDLE_TICKS) {
            break;
        }
        ticks += (uint32_t)record[0] + 1U;
    }
    return ticks;
}

static void test_make_generator_config(void) {
    StepStreamGeneratorConfig_t config = {0};
    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 2500U, 2000U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_OK,
        "config ok"
    );
    expect_u32(config.timer_clock_hz, 275000000U, "config clock");
    expect_u32(config.pulse_ticks, 1375U, "config pulse ticks");
    expect_u32(config.low_ticks, 688U, "config low ticks rounded up");
    expect_u32(config.min_record_ticks, 550U, "config min record ticks");
    expect_u32(config.max_record_ticks, 0U, "no limit stays the format maximum");

    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 2500U, 2000U, 40000U, &config),
        STEP_STREAM_FEEDER_STATUS_OK,
        "config with a record limit"
    );
    expect_u32(config.max_record_ticks, 11000U, "the record limit is converted to ticks");

    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 0U, 2000U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_OK,
        "zero low time is allowed"
    );
    expect_u32(config.low_ticks, 0U, "zero low ticks");

    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 2500U, 2000U, 0U, NULL),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config null"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(0U, 5000U, 2500U, 2000U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config zero clock"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(1000U, 0U, 2500U, 2000U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config zero pulse"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(1000U, 5000U, 2500U, 0U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config zero min record"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, UINT32_MAX, 1U, 1U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config pulse ticks overflow"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, 1U, UINT32_MAX, 1U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config low ticks overflow"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, 1U, 1U, UINT32_MAX, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config min record ticks overflow"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, 1U, 1U, 1U, UINT32_MAX, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config max record ticks overflow"
    );
}

static void test_make_config(void) {
    StepStreamFeederConfig_t config = {0};
    const uint32_t axes[2] = {2U, 0U};
    expect_u32(
        step_stream_feeder_make_config(
            275000000U,
            5000U,
            2500U,
            2000U,
            40000U,
            10000U,
            2000U,
            1365U,
            axes,
            2U,
            &config
        ),
        STEP_STREAM_FEEDER_STATUS_OK,
        "full config ok"
    );
    expect_u32(config.generator.pulse_ticks, 1375U, "full config pulse ticks");
    expect_u32(config.generator.max_record_ticks, 11000U, "full config record limit");
    expect_u32(config.idle_record_ticks, 2750U, "idle record ticks");
    expect_u32(config.startup_period_ns, 2000U, "startup copied");
    expect_u32(config.half_record_count, 1365U, "half count copied");
    expect_u32(config.lane_count, 2U, "lane count copied");
    expect_u32(config.axis_index[0], 2U, "first lane axis copied");
    expect_u32(config.axis_index[1], 0U, "second lane axis copied");

    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 0U, 10000U, 2000U, 1365U, axes, 2U, NULL),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config null"
    );
    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 0U, 10000U, 2000U, 1365U, NULL, 2U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config without axes"
    );
    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 0U, 10000U, 2000U, 1365U, axes, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config without lanes"
    );
    expect_u32(
        step_stream_feeder_make_config(
            275000000U,
            5000U,
            2500U,
            2000U,
            0U,
            10000U,
            2000U,
            1365U,
            axes,
            STEP_STREAM_FEEDER_MAX_LANES + 1U,
            &config
        ),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config with too many lanes"
    );
    expect_u32(
        step_stream_feeder_make_config(0U, 5000U, 2500U, 2000U, 0U, 10000U, 2000U, 1365U, axes, 2U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config zero clock"
    );
    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 0U, 0U, 2000U, 1365U, axes, 2U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config zero idle"
    );
    expect_u32(
        step_stream_feeder_make_config(275000000U, 0U, 2500U, 2000U, 0U, 10000U, 2000U, 1365U, axes, 2U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config bad generator part"
    );
    expect_u32(
        step_stream_feeder_make_config(UINT32_MAX, 1U, 1U, 1U, 0U, UINT32_MAX, 2000U, 1365U, axes, 2U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config idle ticks overflow"
    );
}

static void test_init_validation(void) {
    StepStreamFeeder_t feeder;
    StepStreamFeederConfig_t config = make_config(2U);
    StepStreamFeederPorts_t ports = make_ports();

    expect_u32(
        step_stream_feeder_init(NULL, &config, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "null feeder"
    );
    expect_u32(
        step_stream_feeder_init(&feeder, NULL, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "null config"
    );
    expect_u32(
        step_stream_feeder_init(&feeder, &config, NULL),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "null ports"
    );

    StepStreamFeederPorts_t broken = ports;
    broken.prepare = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no prepare"
    );
    broken = ports;
    broken.acquire_half = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no acquire"
    );
    broken = ports;
    broken.commit_half = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no commit"
    );
    broken = ports;
    broken.set_direction = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no direction"
    );
    broken = ports;
    broken.start = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no start"
    );
    broken = ports;
    broken.stop = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no stop"
    );
    broken = ports;
    broken.plan_epoch = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no epoch"
    );
    broken = ports;
    broken.peek_block = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no peek"
    );
    broken = ports;
    broken.discard_blocks = NULL;
    expect_u32(
        step_stream_feeder_init(&feeder, &config, &broken),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "no discard"
    );

    StepStreamFeederConfig_t bad = config;
    bad.lane_count = 0U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no lanes");
    bad = config;
    bad.lane_count = STEP_STREAM_FEEDER_MAX_LANES + 1U;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "too many lanes"
    );
    bad = config;
    bad.axis_index[1] = MOTION_PLANNER_AXIS_COUNT;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "axis index"
    );
    bad = config;
    bad.axis_index[1] = bad.axis_index[0];
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "one axis, two lanes"
    );
    bad = config;
    bad.startup_period_ns = 0U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "startup");
    bad = config;
    bad.half_record_count = 0U;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "half count"
    );
    bad = config;
    bad.idle_record_ticks = 0U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "idle zero");
    bad = config;
    bad.idle_record_ticks = STEP_STREAM_GENERATOR_MAX_RECORD_TICKS + 1U;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "idle too long"
    );
    bad = config;
    bad.idle_record_ticks = config.generator.min_record_ticks - 1U;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "idle too short"
    );

    bad = config;
    bad.generator.pulse_ticks = 0U;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "a generator configuration that no block could run"
    );
    bad = config;
    bad.generator.max_record_ticks = bad.generator.min_record_ticks;
    expect_u32(
        step_stream_feeder_init(&feeder, &bad, &ports),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "a record limit that cannot hold two minimal records"
    );
    expect_u32(step_stream_feeder_init(&feeder, &config, &ports), STEP_STREAM_FEEDER_STATUS_OK, "valid two-lane init");
    expect_u32(feeder.lane_axes_mask, 3U, "the lanes cover X and Y");
    expect_u32(feeder.end_block, UINT32_MAX, "a new feeder has an open stream");

    expect_u32(step_stream_feeder_service(NULL), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "service null");
    expect_u32(step_stream_feeder_get_phase(NULL), STEP_STREAM_FEEDER_PHASE_IDLE, "phase null");
    step_stream_feeder_reset(NULL);
    StepStreamFeederDiagnostics_t diagnostics = {0};
    step_stream_feeder_get_diagnostics(NULL, &diagnostics);
    step_stream_feeder_get_diagnostics(&feeder, NULL);
}

static void test_idle_without_blocks(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "idle service");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "stays idle");
    expect_u32(fake_prepare_calls, 0U, "no prepare without block");
    expect_u32(fake_start_calls, 0U, "no start without block");
    expect_u32(fake_direction_calls, 0U, "no direction without block");
}

static void test_short_block_stream_lifecycle(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    const MotionPlanBlock_t block = make_block(3, 0);
    queue_block(&block);

    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start service");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "streaming");
    expect_u32(fake_prepare_calls, 1U, "prepared once");
    expect_u32(fake_direction_calls, 1U, "direction set once");
    expect_u32(fake_direction_value[0], 1U, "positive direction");
    expect_u32(fake_start_calls, 1U, "started once");
    expect_u32(fake_last_startup_ns, 5000U, "startup period passed");
    expect_u32(fake_plan_count, 0U, "the started block is given back");
    expect_u32(fake_discarded_total, 1U, "exactly that block");

    expect_u32(fake_halves[0][0][0], 9999U, "step record ARR");
    expect_u32(fake_halves[0][0][1], 0U, "step record middle word");
    expect_u32(fake_halves[0][0][2], TEST_PULSE_TICKS, "first step pulse");
    expect_u32(count_step_records(0U), 3U, "three step records");
    expect_u32(fake_halves[0][0][3 * 3U], TEST_IDLE_TICKS - 1U, "idle record ARR");
    expect_u32(fake_halves[0][0][3 * 3U + 2U], 0U, "idle record has no pulse");
    expect_u32(count_step_records(1U), 0U, "second half is idle");
    expect_u32(fake_halves[0][1][0], TEST_IDLE_TICKS - 1U, "second half idle ARR");

    /* Half 0 played: it held steps, so the stream goes on with an idle refill. */
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill service");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "still streaming");
    expect_u32(fake_stop_calls, 0U, "not stopped while the other half is idle but this one had steps");
    expect_u32(count_step_records(0U), 0U, "refilled idle");

    /* Half 1 played: both halves idle and no block, so the stream ends. */
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "stop service");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "idle again");
    expect_u32(fake_stop_calls, 1U, "axis stopped");

    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.blocks_started, 1U, "blocks started");
    expect_u32(diagnostics.blocks_completed, 1U, "blocks completed");
    expect_u32(diagnostics.halves_filled, 3U, "halves filled");
    expect_u32(diagnostics.streams_started, 1U, "streams started");
    expect_u32(diagnostics.streams_stopped, 1U, "streams stopped");
}

static void test_late_block_joins_idle_stream(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    const MotionPlanBlock_t first = make_block(3, 0);
    const MotionPlanBlock_t late = make_block(2, 0);
    queue_block(&first);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");

    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "idle refill");
    /* Both halves are idle now, but a block of the same direction arrives before the stream stops. One lane has nothing
     * to get out of step with, so it takes the block even though it looked at the end of the plan before. */
    queue_block(&late);
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "late block");
    expect_u32(fake_stop_calls, 0U, "stream is not stopped");
    expect_u32(count_step_records(1U), 2U, "late block fills the released half");
    expect_u32(fake_prepare_calls, 1U, "same stream");
    expect_u32(feeder.end_block, UINT32_MAX, "a single lane never closes its stream");

    /* Half 0 is released next: it is filler, but the other half carries the late block, so the stream goes on. */
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill behind the late block");
    expect_u32(fake_stop_calls, 0U, "a half that carries a block keeps the stream alive");
}

static void test_block_spans_halves_and_chains(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    /* 20 steps: 8 + 8 + 4 records; the next block of the same direction joins right behind it. */
    const MotionPlanBlock_t first = make_block(20, 0);
    const MotionPlanBlock_t second = make_block(2, 0);
    queue_block(&first);
    queue_block(&second);

    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(count_step_records(0U), 8U, "half 0 full of steps");
    expect_u32(count_step_records(1U), 8U, "half 1 full of steps");
    expect_u32(fake_plan_count, 1U, "the second block is not started yet and stays queued");

    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 0");
    expect_u32(count_step_records(0U), 6U, "4 steps of the first block and 2 of the second");
    expect_u32(fake_direction_calls, 1U, "same direction needs no new DIR");
    expect_u32(fake_plan_count, 0U, "both blocks are given back");

    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 1");
    expect_u32(count_step_records(1U), 0U, "nothing left");
    expect_u32(fake_stop_calls, 0U, "half 0 held steps, no stop yet");

    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 0 again");
    expect_u32(fake_stop_calls, 0U, "the half just released held steps, so the stream goes on");

    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 1 again");
    expect_u32(fake_stop_calls, 1U, "idle stretch of both halves stops the stream");

    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.blocks_completed, 2U, "both blocks completed");
}

static void test_block_fills_half_exactly(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    const MotionPlanBlock_t block = make_block(8, 0);
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(count_step_records(0U), 8U, "block fills half 0 exactly");
    expect_u32(count_step_records(1U), 0U, "half 1 idle");
}

static void test_direction_change_stops_and_restarts(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    const MotionPlanBlock_t forward = make_block(3, 0);
    const MotionPlanBlock_t backward = make_block(-2, 0);
    queue_block(&forward);
    queue_block(&backward);

    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start forward");
    expect_u32(fake_direction_calls, 1U, "forward DIR");
    expect_u32(count_step_records(0U), 3U, "forward steps only");
    expect_u32(fake_start_calls, 1U, "one start so far");
    expect_u32(fake_plan_count, 1U, "the backward block waits in the plan");

    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill idle");
    expect_u32(fake_stop_calls, 0U, "no stop before the idle stretch is played");
    expect_u32(count_step_records(0U), 0U, "backward block is not mixed into the stream");

    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "stop and restart");
    expect_u32(fake_stop_calls, 1U, "stream stopped for the direction change");
    expect_u32(fake_prepare_calls, 2U, "second stream prepared");
    expect_u32(fake_direction_calls, 2U, "DIR set again");
    expect_u32(fake_direction_value[0], 0U, "second DIR negative");
    expect_u32(fake_start_calls, 2U, "second stream started");
    expect_u32(count_step_records(0U), 2U, "backward steps in the new stream");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "streaming backward");
}

/* The lanes of X and Y walk the same blocks: a lane that does not move in a block waits for as long as the block lasts.
 */
static void test_lanes_stay_in_step(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    /* Y only, Y only, X only: the direction of both axes is decided at the start, so all three blocks join one stream.
     */
    const MotionPlanBlock_t y_first = make_block(0, 3);
    const MotionPlanBlock_t y_second = make_block(0, 2);
    const MotionPlanBlock_t x_move = make_block(2, 0);
    queue_block(&y_first);
    queue_block(&y_second);
    queue_block(&x_move);

    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(fake_prepare_calls, 1U, "one stream for the three blocks");
    expect_u32(fake_direction_calls, 2U, "DIR of both axes, set before the start");
    expect_u32(fake_direction_set[0], 1U, "X DIR set");
    expect_u32(fake_direction_value[0], 1U, "X positive");
    expect_u32(fake_direction_set[1], 1U, "Y DIR set");
    expect_u32(fake_direction_value[1], 1U, "Y positive");
    expect_u32(fake_start_calls, 1U, "one start for both lanes");

    expect_u32(count_step_records_of(0U, 0U), 2U, "X steps only in its own block");
    expect_u32(count_step_records_of(1U, 0U), 5U, "Y steps of the first two blocks");
    /* 3 + 2 + 2 steps of 10000 ticks: every lane covers the same time, with steps or with waiting. */
    expect_u32(block_ticks_of(0U, 0U), 70000U, "X covers the three blocks");
    expect_u32(block_ticks_of(1U, 0U), 70000U, "Y covers the same time");
    expect_u32(fake_plan_count, 0U, "every lane has started every block");

    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.blocks_started, 3U, "blocks are counted once, not once per lane");
    expect_u32(diagnostics.blocks_completed, 3U, "blocks completed once, not once per lane");
}

/* A lane with few steps reaches many blocks ahead; the blocks stay queued until the slowest lane has started them. */
static void test_lookahead_retention(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    /* Block 0 has 20 X steps (a half takes 8 records), blocks 1 to 5 have 3 X steps each: Y waits one record per short
     * block, so its two halves reach block 5 while X is still in block 0. */
    const MotionPlanBlock_t long_x = make_block(20, 0);
    queue_block(&long_x);
    for (uint32_t index = 0U; index < 5U; ++index) {
        const MotionPlanBlock_t short_x = make_block(3, 0);
        queue_block(&short_x);
    }
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");

    expect_u32(count_step_records_of(0U, 0U), 8U, "X half 0 is full of steps of the first block");
    expect_u32(count_step_records_of(0U, 1U), 8U, "and half 1");
    expect_u32(feeder.lane[0].next_block, 1U, "X has started one block");
    expect_u32(feeder.lane[1].next_block, 6U, "Y has started all six");
    expect_u32(fake_discarded_total, 1U, "only the block that both lanes started is given back");
    expect_u32(fake_plan_count, 5U, "the others stay queued for the slower lane");
    expect_u32(feeder.front_block, 1U, "the front moved by the discarded block");

    /* X plays on and starts the following blocks; the plan drains behind it. */
    const uint32_t expected_next_x[3] = {3U, 5U, 6U};
    const uint32_t expected_queued[3] = {3U, 1U, 0U};
    for (uint32_t round = 0U; round < 3U; ++round) {
        release(0U, (uint8_t)(round % 2U));
        release(1U, (uint8_t)(round % 2U));
        expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "service while the lanes play");
        expect_u32(feeder.lane[0].next_block, expected_next_x[round], "X moves through the blocks");
        expect_u32(fake_plan_count, expected_queued[round], "the plan drains as X starts blocks");
    }
    expect_u32(fake_discarded_total, 6U, "each block is discarded exactly once");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "the stream is still alive");
    expect_u32(fake_stop_calls, 0U, "nothing was stopped on the way");
}

/* The plan ends: with several lanes the stream closes there, and a block that comes later waits for the next stream. */
static void test_multi_lane_stream_closes_at_the_end_of_the_plan(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t first = make_block(3, 0);
    queue_block(&first);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(feeder.end_block, 1U, "the first lane that found no block closed the stream after block 0");

    const MotionPlanBlock_t late = make_block(2, 0);
    queue_block(&late);
    release(0U, 0U);
    release(1U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill after the late block came");
    expect_u32(count_step_records_of(0U, 0U), 0U, "the late block is not started by a lane behind the closed end");
    expect_u32(count_step_records_of(1U, 0U), 0U, "nor by the other lane");
    expect_u32(fake_stop_calls, 0U, "the stream is still playing its last idle half");

    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "every lane played an idle stretch");
    expect_u32(fake_stop_calls, 1U, "the stream stopped");
    expect_u32(fake_prepare_calls, 2U, "and the late block opened the next one");
    expect_u32(fake_start_calls, 2U, "which started");
    expect_u32(count_step_records_of(0U, 0U), 2U, "with its steps on X");
    expect_u32(fake_plan_count, 0U, "the late block is given back");
}

/* The stream stops only when every lane is through: one lane finished does not stop the others. */
/* Y reaches the end of the plan while X is still in the first block; a block that comes later must not be started by X
 * either, or X would run it while Y plays idle records. */
static void test_lanes_at_different_positions_close_at_the_same_block(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t long_x = make_block(20, 0);
    const MotionPlanBlock_t short_x = make_block(3, 0);
    queue_block(&long_x);
    queue_block(&short_x);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(feeder.lane[0].next_block, 1U, "X is in the first block");
    expect_u32(feeder.lane[1].next_block, 2U, "Y has started both");
    expect_u32(feeder.end_block, 2U, "Y found the end of the plan after the second block");

    const MotionPlanBlock_t late = make_block(2, 0);
    queue_block(&late);
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "X refills its half 0");
    expect_u32(count_step_records(0U), 7U, "4 steps of the first block and 3 of the second");
    expect_u32(feeder.lane[0].next_block, 2U, "X started the second block but not the late one");
    expect_u32(fake_plan_count, 1U, "the late block is still queued, and only it");

    /* Everything idles out; the late block opens the next stream on both lanes together. */
    release(1U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "Y refills");
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "X refills half 1");
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "X refills half 0 again");
    expect_u32(fake_stop_calls, 0U, "Y has not played its second half yet");
    release(1U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "Y refills half 1");
    expect_u32(fake_stop_calls, 1U, "every lane is idle: the stream stops");
    expect_u32(fake_prepare_calls, 2U, "and the late block opens the next one");
    expect_u32(count_step_records_of(0U, 0U), 2U, "X steps in it");
    expect_u32(block_ticks_of(1U, 0U), 20000U, "and Y waits for the same time");
    expect_u32(fake_plan_count, 0U, "the late block is given back");
}

/* The same with a reversal: Y waits behind it while X is still before it; the stream stops only after X has played. */
static void test_lanes_at_different_positions_stop_at_the_same_reversal(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t forward = make_block(20, 0);
    const MotionPlanBlock_t backward = make_block(-2, 0);
    queue_block(&forward);
    queue_block(&backward);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(feeder.lane[0].next_block, 1U, "X is in the forward block");
    expect_u32(feeder.lane[1].next_block, 1U, "Y waits it out and stops at the reversal");
    expect_u32(feeder.end_block, UINT32_MAX, "the plan did not end");

    /* Y plays out its halves long before X is through its block. */
    release(1U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "Y refills");
    release(1U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "Y refills again");
    expect_u32(fake_stop_calls, 0U, "X still carries the forward block");

    /* X plays on: the last 4 steps, then idle halves. */
    release(0U, 0U);
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_OK,
        "X refills with the end of its block"
    );
    expect_u32(count_step_records(0U), 4U, "the last 4 steps");
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "X refills");
    expect_u32(fake_stop_calls, 0U, "the half that X just played carried steps");
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "X refills again");
    expect_u32(fake_stop_calls, 0U, "the half that X played before carried the last steps");
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "X refills once more");
    expect_u32(fake_stop_calls, 1U, "now every lane is idle: stop and start the backward stream");
    expect_u32(fake_direction_value[0], 0U, "X backward");
    expect_u32(count_step_records_of(0U, 0U), 2U, "with its steps");
}

static void test_stream_stops_only_when_every_lane_is_idle(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t forward = make_block(3, 0);
    const MotionPlanBlock_t backward = make_block(-2, 0);
    queue_block(&forward);
    queue_block(&backward);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");

    /* Lane 0 plays out and idles; lane 1 has not released its halves yet. */
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "lane 0 refills");
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "lane 0 refills again");
    expect_u32(fake_stop_calls, 0U, "lane 1 still carries block time, so the stream goes on");

    release(1U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "lane 1 refills");
    expect_u32(fake_stop_calls, 0U, "the half that lane 1 just played carried block time");
    release(1U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "lane 1 refills again");
    expect_u32(fake_stop_calls, 1U, "now every lane is idle: stop, and the backward block starts the next stream");
    expect_u32(fake_prepare_calls, 2U, "next stream");
    expect_u32(fake_direction_value[0], 0U, "X backward");
}

static void test_direction_is_decided_per_axis(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    /* X forward, X backward, Y backward: the lookahead stops at the reversal, so Y is not decided for the first stream.
     */
    const MotionPlanBlock_t x_forward = make_block(3, 0);
    const MotionPlanBlock_t x_backward = make_block(-2, 0);
    const MotionPlanBlock_t y_backward = make_block(0, -2);
    queue_block(&x_forward);
    queue_block(&x_backward);
    queue_block(&y_backward);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start on the forward block");
    expect_u32(fake_direction_calls, 1U, "only the axis that the first stream moves gets a DIR");
    expect_u32(fake_direction_lane_log[0], 0U, "and it is lane 0");
    expect_u32(fake_direction_value[0], 1U, "forward");
    expect_u32(fake_direction_set[1], 0U, "Y has no direction yet");

    release(0U, 0U);
    release(1U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "idle refill");
    release(0U, 1U);
    release(1U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "stop and restart");
    expect_u32(fake_stop_calls, 1U, "the first stream stopped");
    expect_u32(fake_direction_calls, 3U, "the second stream sets both axes");
    expect_u32(fake_direction_value[0], 0U, "X backward");
    expect_u32(fake_direction_set[1], 1U, "Y decided by the second stream");
    expect_u32(fake_direction_value[1], 0U, "Y backward");
    expect_u32(count_step_records_of(0U, 0U), 2U, "X steps in the second stream");
    expect_u32(count_step_records_of(1U, 0U), 2U, "Y steps behind the X block");
}

/* A block that comes after the lookahead cannot move an axis whose direction the stream has not decided. */
static void test_a_late_block_cannot_move_an_undecided_axis(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t x_move = make_block(3, 0);
    queue_block(&x_move);
    /* The Y block reaches the plan right after the lookahead has found its end: too late to be decided. */
    fake_push_block = make_block(0, -2);
    fake_push_after_first_end = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(fake_push_fired, 1U, "the Y block was pushed behind the lookahead");
    expect_u32(fake_direction_calls, 1U, "only X was decided");
    expect_u32(count_step_records_of(1U, 0U), 0U, "Y does not step in the stream that cannot move it");
    expect_u32(fake_plan_count, 1U, "the Y block waits in the plan");
    expect_u32(feeder.end_block, UINT32_MAX, "it waits because of the direction, not because the plan ended");

    release(0U, 0U);
    release(1U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "idle refill");
    release(0U, 1U);
    release(1U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "stop and restart");
    expect_u32(fake_stop_calls, 1U, "the first stream stopped");
    expect_u32(fake_direction_calls, 2U, "the next stream decides Y");
    expect_u32(fake_direction_lane_log[1], 1U, "on lane 1");
    expect_u32(fake_direction_value[1], 0U, "backward");
    expect_u32(count_step_records_of(1U, 0U), 2U, "and Y steps there");
}

static void test_block_for_an_axis_without_lane_is_an_error(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t z_only = make_block3(0, 0, 4);
    queue_block(&z_only);
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "a first block that moves Z cannot be run"
    );
    expect_u32(fake_prepare_calls, 0U, "no stream is opened for it");
    expect_u32(fake_stop_calls, 1U, "the fault stops the streams");

    /* Behind a good block the same block ends the lookahead and faults when a lane reaches it. */
    setup_lanes(&feeder, 2U);
    const MotionPlanBlock_t x_move = make_block(3, 0);
    queue_block(&x_move);
    queue_block(&z_only);
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "a lane that reaches the Z block faults"
    );
    expect_u32(fake_direction_calls, 1U, "the lookahead stopped before the Z block");
    expect_u32(fake_start_calls, 0U, "the stream never started");
}

static void test_generator_errors(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    MotionPlanBlock_t invalid = make_block(3, 0);
    invalid.master_step_count = 0U;
    queue_block(&invalid);

    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "init_from_block rejects the block"
    );
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_FAULT, "fault phase");
    expect_u32(fake_stop_calls, 1U, "axis stopped on fault");
    expect_u32(fake_start_calls, 0U, "never started");
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR, "fault is sticky");

    fake_plan_count = 0U;
    step_stream_feeder_reset(&feeder);
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "reset clears the fault");
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "works again after reset");

    /* 1 MHz step rate at a 1 GHz clock: 1000-tick interval is shorter than pulse + low. */
    MotionPlanBlock_t too_fast = make_block(3, 0);
    too_fast.step_event_rate_numerator = 1000000U;
    queue_block(&too_fast);
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "fill rejects a step interval that is too short"
    );

    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.generator_errors, 2U, "generator errors counted");
}

static void test_plan_port_errors(void) {
    StepStreamFeeder_t feeder;
    const MotionPlanBlock_t block = make_block(3, 0);

    /* A failing read at the start of a stream. */
    setup(&feeder);
    queue_block(&block);
    fake_peek_forced_result = STEP_STREAM_FEEDER_PLAN_ERROR;
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "peek fails at the start"
    );

    /* A clear that races with the first look is not a fault: the next call starts afresh. */
    setup(&feeder);
    queue_block(&block);
    fake_peek_forced_result = STEP_STREAM_FEEDER_PLAN_STALE;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "a stale first look is tolerated");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "nothing started");
    expect_u32(fake_prepare_calls, 0U, "no prepare either");
    fake_peek_forced_result = STEP_STREAM_FEEDER_PLAN_OK;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "the next call starts the stream");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "streaming");

    /* A read that fails behind the first block, while the lookahead scans. */
    setup(&feeder);
    queue_block(&block);
    queue_block(&block);
    fake_peek_forced_result = STEP_STREAM_FEEDER_PLAN_ERROR;
    fake_peek_forced_after = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR, "scan read fails");

    /* A clear in the middle of the scan changes nothing that has started: look again next time. */
    setup(&feeder);
    queue_block(&block);
    queue_block(&block);
    fake_peek_forced_result = STEP_STREAM_FEEDER_PLAN_STALE;
    fake_peek_forced_after = 1U;
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_OK,
        "a stale read behind the first block"
    );
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "is no start and no fault");
    expect_u32(fake_prepare_calls, 0U, "nothing was prepared");

    /* A block that cannot be given back. */
    setup(&feeder);
    queue_block(&block);
    fake_discard_forced_result = STEP_STREAM_FEEDER_PLAN_ERROR;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR, "discard fails");

    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.generator_errors, 1U, "plan errors count as generator errors");
}

/* A planner error clears the plan without stopping the stream. One lane loses only the blocks it had not started. */
static void test_single_lane_survives_a_clear(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    const MotionPlanBlock_t first = make_block(3, 0);
    queue_block(&first);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");

    ++fake_epoch;
    fake_plan_count = 0U;
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "the clear is no fault");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "the stream goes on");
    expect_u32(feeder.plan_epoch, fake_epoch, "the feeder follows the new epoch");

    /* A block that is planned after the clear joins the stream. */
    const MotionPlanBlock_t after_clear = make_block(2, 0);
    queue_block(&after_clear);
    release(0U, 1U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "service after the clear");
    expect_u32(count_step_records(1U), 2U, "the new block is started");
    expect_u32(fake_plan_count, 0U, "and given back");
    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.generator_errors, 0U, "no error was counted");
    expect_u32(fake_stop_calls, 0U, "nothing was stopped");
}

/* The clear comes between the read of a block and its return: the block has started, its return is moot. */
static void test_single_lane_survives_a_clear_before_the_discard(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    const MotionPlanBlock_t first = make_block(3, 0);
    queue_block(&first);
    /* Peek calls: the lookahead takes two (block, end of plan), the lane's read of the block is the third. */
    fake_clear_after_peek_call = 3U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "streaming");
    expect_u32(count_step_records(0U), 3U, "the block that was read runs");
    expect_u32(fake_discarded_total, 0U, "the clear had taken it already");
    expect_u32(feeder.plan_epoch, fake_epoch, "the feeder follows the new epoch");
    expect_u32(feeder.front_block, 1U, "and its front is behind the started block");
}

/* With several lanes the same clear is a fault: some lanes may have started blocks that the others can no longer start.
 */
static void test_several_lanes_do_not_survive_a_clear(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    /* A reversal behind the first block keeps the plan from ending, so the lanes read it again once the block is
     * played. */
    const MotionPlanBlock_t block = make_block(20, 0);
    const MotionPlanBlock_t reversal = make_block(-2, 0);
    queue_block(&block);
    queue_block(&reversal);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    ++fake_epoch;
    fake_plan_count = 0U;
    release(0U, 0U);
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "a clear behind the stream's back is a fault"
    );
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_FAULT, "fault phase");
    expect_u32(fake_stop_calls, 1U, "the streams are stopped");

    /* The same when the clear comes right before the return of a started block. */
    setup_lanes(&feeder, 2U);
    queue_block(&block);
    /* Peek calls: the lookahead takes two, lane 0 reads the block (3), lane 1 reads it (4) and returns it. */
    fake_clear_after_peek_call = 4U;
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "a clear before the return is a fault too"
    );

    /* And during the priming of a stream. */
    setup_lanes(&feeder, 2U);
    queue_block(&block);
    fake_clear_after_peek_call = 2U;
    expect_u32(
        step_stream_feeder_service(&feeder),
        STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
        "a clear while the halves are filled is a fault"
    );
    expect_u32(fake_start_calls, 0U, "the stream never started");
}

static void test_axis_errors(void) {
    StepStreamFeeder_t feeder;
    const MotionPlanBlock_t block = make_block(3, 0);

    setup(&feeder);
    fake_fail_prepare = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "prepare fails");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_FAULT, "prepare fault phase");

    setup(&feeder);
    fake_fail_direction = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "direction fails");

    setup(&feeder);
    fake_fail_acquire = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "acquire fails");

    setup(&feeder);
    fake_bad_half = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "half index out of range");

    setup(&feeder);
    fake_null_records = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "records missing");

    setup(&feeder);
    fake_fail_commit = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "commit fails");

    setup(&feeder);
    fake_fail_start = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "start fails");
    expect_u32(fake_stop_calls, 1U, "axis stopped after a failed start");

    StepStreamFeederDiagnostics_t diagnostics;
    step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
    expect_u32(diagnostics.axis_errors, 1U, "axis errors counted");
}

/* A failure on the second lane is as fatal as on the first. */
static void test_axis_errors_on_the_second_lane(void) {
    StepStreamFeeder_t feeder;
    const MotionPlanBlock_t block = make_block(3, 3);

    setup_lanes(&feeder, 2U);
    fake_fail_acquire = 1U;
    fake_failing_lane = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "acquire fails on lane 1");
    expect_u32(count_step_records_of(0U, 0U), 3U, "lane 0 was filled before lane 1 failed");

    setup_lanes(&feeder, 2U);
    fake_fail_commit = 1U;
    fake_failing_lane = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "commit fails on lane 1");

    setup_lanes(&feeder, 2U);
    fake_fail_direction = 1U;
    fake_failing_lane = 1U;
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_AXIS_ERROR, "direction fails on lane 1");
    expect_u32(fake_start_calls, 0U, "nothing starts after it");
}

static void test_counters_saturate(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    feeder.diagnostics.halves_filled = UINT32_MAX;
    const MotionPlanBlock_t block = make_block(3, 0);
    queue_block(&block);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "service with saturated counter");
    expect_u32(feeder.diagnostics.halves_filled, UINT32_MAX, "counter saturates");
}

static void test_reset_drops_state(void) {
    StepStreamFeeder_t feeder;
    setup_lanes(&feeder, 2U);
    /* 20 steps do not fit the two halves of X, so its generator is in the middle of the block when the fault comes. */
    const MotionPlanBlock_t forward = make_block(20, 0);
    const MotionPlanBlock_t backward = make_block(-2, 0);
    queue_block(&forward);
    queue_block(&backward);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(feeder.lane[0].next_block, 1U, "a block was started");
    expect_u32(feeder.lane[0].generator_active, 1U, "X is in the middle of it");
    expect_u32(feeder.lane[0].half_has_block[0], 1U, "its half carries the block");
    expect_u32(feeder.direction_known_mask, 1U, "X direction was decided");
    expect_u32(feeder.direction_positive_mask, 1U, "and it is forward");
    expect_u32(feeder.front_block, 1U, "the front moved behind the started block");

    ++fake_epoch;
    fake_plan_count = 0U;
    release(0U, 0U);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR, "a fault");
    expect_u32(feeder.fault_status, STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR, "the fault is recorded");

    step_stream_feeder_reset(&feeder);
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "idle after reset");
    expect_u32(feeder.fault_status, STEP_STREAM_FEEDER_STATUS_OK, "fault dropped");
    for (uint32_t lane = 0U; lane < 2U; ++lane) {
        expect_u32(feeder.lane[lane].next_block, 0U, "lane position dropped");
        expect_u32(feeder.lane[lane].generator_active, 0U, "generator dropped");
        expect_u32(feeder.lane[lane].half_has_block[0], 0U, "half flags dropped");
        expect_u32(feeder.lane[lane].carry.ns_to_ticks_remainder, 0U, "carry dropped");
    }
    expect_u32(feeder.front_block, 0U, "front dropped");
    expect_u32(feeder.end_block, UINT32_MAX, "stream open again");
    expect_u32(feeder.direction_known_mask, 0U, "directions dropped");
    expect_u32(feeder.direction_positive_mask, 0U, "direction values dropped");
    expect_u32(feeder.plan_epoch, 0U, "epoch dropped");
}

int main(void) {
    test_make_generator_config();
    test_make_config();
    test_init_validation();
    test_idle_without_blocks();
    test_short_block_stream_lifecycle();
    test_late_block_joins_idle_stream();
    test_block_spans_halves_and_chains();
    test_block_fills_half_exactly();
    test_direction_change_stops_and_restarts();
    test_lanes_stay_in_step();
    test_lookahead_retention();
    test_multi_lane_stream_closes_at_the_end_of_the_plan();
    test_stream_stops_only_when_every_lane_is_idle();
    test_lanes_at_different_positions_close_at_the_same_block();
    test_lanes_at_different_positions_stop_at_the_same_reversal();
    test_direction_is_decided_per_axis();
    test_a_late_block_cannot_move_an_undecided_axis();
    test_block_for_an_axis_without_lane_is_an_error();
    test_generator_errors();
    test_plan_port_errors();
    test_single_lane_survives_a_clear();
    test_single_lane_survives_a_clear_before_the_discard();
    test_several_lanes_do_not_survive_a_clear();
    test_axis_errors();
    test_axis_errors_on_the_second_lane();
    test_counters_saturate();
    test_reset_drops_state();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "%lu test(s) failed\n", (unsigned long)test_failures);
        return 1;
    }
    (void)printf("step_stream_feeder_unit_tests passed\n");
    return 0;
}

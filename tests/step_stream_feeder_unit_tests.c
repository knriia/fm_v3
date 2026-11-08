#include "motion/step_stream_feeder.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_HALF_RECORDS 8U
#define TEST_PULSE_TICKS 2500U
#define TEST_IDLE_TICKS 10000U
#define TEST_QUEUE_SIZE 8U
#define TEST_WORDS (TEST_HALF_RECORDS * STEP_STREAM_GENERATOR_RECORD_WORDS)

static uint32_t test_failures;

static void expect_true(uint8_t condition, const char *message) {
    if (condition == 0U) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
    }
}

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

/* Fake axis: two halves, EMPTY after prepare; the test releases a half the way the DMA interrupt would. */
static uint16_t fake_halves[2][TEST_WORDS];
static uint8_t fake_empty[2];
static uint32_t fake_prepare_calls;
static uint32_t fake_start_calls;
static uint32_t fake_stop_calls;
static uint32_t fake_direction_calls;
static uint8_t fake_last_direction;
static uint8_t fake_direction_log[4];
static uint32_t fake_last_startup_ns;
static uint8_t fake_fail_prepare;
static uint8_t fake_fail_acquire;
static uint8_t fake_fail_commit;
static uint8_t fake_fail_direction;
static uint8_t fake_fail_start;
static uint8_t fake_bad_half;
static uint8_t fake_null_records;

static MotionPlanBlock_t fake_queue[TEST_QUEUE_SIZE];
static uint32_t fake_queue_head;
static uint32_t fake_queue_count;

static StepStreamFeederAxisResult_t fake_prepare(void) {
    ++fake_prepare_calls;
    if (fake_fail_prepare != 0U) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    fake_empty[0] = 1U;
    fake_empty[1] = 1U;
    return STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t fake_acquire_half(uint8_t *half, uint16_t **records) {
    if (fake_fail_acquire != 0U) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    for (uint8_t index = 0U; index < 2U; ++index) {
        if (fake_empty[index] != 0U) {
            *half = fake_bad_half != 0U ? 2U : index;
            *records = fake_null_records != 0U ? NULL : fake_halves[index];
            return STEP_STREAM_FEEDER_AXIS_OK;
        }
    }
    return STEP_STREAM_FEEDER_AXIS_NO_REQUEST;
}

static StepStreamFeederAxisResult_t fake_commit_half(uint8_t half) {
    if (fake_fail_commit != 0U) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    fake_empty[half] = 0U;
    return STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t fake_set_direction(uint8_t positive) {
    if (fake_direction_calls < sizeof(fake_direction_log)) {
        fake_direction_log[fake_direction_calls] = positive;
    }
    ++fake_direction_calls;
    fake_last_direction = positive;
    return fake_fail_direction != 0U ? STEP_STREAM_FEEDER_AXIS_ERROR : STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t fake_start(uint32_t startup_period_ns) {
    ++fake_start_calls;
    fake_last_startup_ns = startup_period_ns;
    return fake_fail_start != 0U ? STEP_STREAM_FEEDER_AXIS_ERROR : STEP_STREAM_FEEDER_AXIS_OK;
}

static void fake_stop(void) { ++fake_stop_calls; }

static uint8_t fake_pop_block(MotionPlanBlock_t *block) {
    if (fake_queue_count == 0U) {
        return 0U;
    }
    *block = fake_queue[fake_queue_head];
    fake_queue_head = (fake_queue_head + 1U) % TEST_QUEUE_SIZE;
    --fake_queue_count;
    return 1U;
}

static void queue_block(const MotionPlanBlock_t *block) {
    fake_queue[(fake_queue_head + fake_queue_count) % TEST_QUEUE_SIZE] = *block;
    ++fake_queue_count;
}

/* 100 kHz step rate: one step every 10000 ns, i.e. one 10000-tick record at a 1 GHz timer clock. */
static MotionPlanBlock_t make_block(int64_t x_steps, int64_t y_steps) {
    MotionPlanBlock_t block = {0};
    const uint64_t x_abs = (uint64_t)(x_steps < 0 ? -x_steps : x_steps);
    const uint64_t y_abs = (uint64_t)(y_steps < 0 ? -y_steps : y_steps);
    block.step_delta[0] = x_steps;
    block.step_delta[1] = y_steps;
    block.master_step_count = x_abs > y_abs ? x_abs : y_abs;
    block.step_event_rate_numerator = 100000U;
    block.step_event_rate_denominator = 1U;
    block.moving_axes_mask = (uint8_t)((x_steps != 0 ? 1U : 0U) | (y_steps != 0 ? 2U : 0U));
    block.positive_direction_mask = (uint8_t)((x_steps > 0 ? 1U : 0U) | (y_steps > 0 ? 2U : 0U));
    return block;
}

static StepStreamFeederConfig_t make_config(void) {
    return (StepStreamFeederConfig_t){
        .generator =
            {
                .timer_clock_hz = 1000000000U,
                .pulse_ticks = TEST_PULSE_TICKS,
                .low_ticks = 2500U,
                .min_record_ticks = 100U,
            },
        .axis_index = 0U,
        .startup_period_ns = 5000U,
        .idle_record_ticks = TEST_IDLE_TICKS,
        .half_record_count = TEST_HALF_RECORDS,
    };
}

static StepStreamFeederPorts_t make_ports(void) {
    return (StepStreamFeederPorts_t){
        fake_prepare, fake_acquire_half, fake_commit_half, fake_set_direction, fake_start, fake_stop, fake_pop_block,
    };
}

static void reset_fakes(void) {
    memset(fake_halves, 0, sizeof(fake_halves));
    memset(fake_empty, 0, sizeof(fake_empty));
    memset(fake_direction_log, 0xFF, sizeof(fake_direction_log));
    memset(fake_queue, 0, sizeof(fake_queue));
    fake_prepare_calls = 0U;
    fake_start_calls = 0U;
    fake_stop_calls = 0U;
    fake_direction_calls = 0U;
    fake_last_direction = 0xFFU;
    fake_last_startup_ns = 0U;
    fake_fail_prepare = 0U;
    fake_fail_acquire = 0U;
    fake_fail_commit = 0U;
    fake_fail_direction = 0U;
    fake_fail_start = 0U;
    fake_bad_half = 0U;
    fake_null_records = 0U;
    fake_queue_head = 0U;
    fake_queue_count = 0U;
}

static void setup(StepStreamFeeder_t *feeder) {
    reset_fakes();
    const StepStreamFeederConfig_t config = make_config();
    const StepStreamFeederPorts_t ports = make_ports();
    expect_u32(step_stream_feeder_init(feeder, &config, &ports), STEP_STREAM_FEEDER_STATUS_OK, "setup init");
}

static uint32_t count_step_records(uint32_t half) {
    uint32_t steps = 0U;
    for (uint32_t index = 0U; index < TEST_HALF_RECORDS; ++index) {
        if (fake_halves[half][index * 3U + 2U] != 0U) {
            ++steps;
        }
    }
    return steps;
}

static void test_make_generator_config(void) {
    StepStreamGeneratorConfig_t config = {0};
    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 2500U, 2000U, &config),
        STEP_STREAM_FEEDER_STATUS_OK,
        "config ok"
    );
    expect_u32(config.timer_clock_hz, 275000000U, "config clock");
    expect_u32(config.pulse_ticks, 1375U, "config pulse ticks");
    expect_u32(config.low_ticks, 688U, "config low ticks rounded up");
    expect_u32(config.min_record_ticks, 550U, "config min record ticks");

    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 0U, 2000U, &config),
        STEP_STREAM_FEEDER_STATUS_OK,
        "zero low time is allowed"
    );
    expect_u32(config.low_ticks, 0U, "zero low ticks");

    expect_u32(
        step_stream_feeder_make_generator_config(275000000U, 5000U, 2500U, 2000U, NULL),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config null"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(0U, 5000U, 2500U, 2000U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config zero clock"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(1000U, 0U, 2500U, 2000U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config zero pulse"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(1000U, 5000U, 2500U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config zero min record"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, UINT32_MAX, 1U, 1U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config pulse ticks overflow"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, 1U, UINT32_MAX, 1U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config low ticks overflow"
    );
    expect_u32(
        step_stream_feeder_make_generator_config(UINT32_MAX, 1U, 1U, UINT32_MAX, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "config min record ticks overflow"
    );
}

static void test_make_config(void) {
    StepStreamFeederConfig_t config = {0};
    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 10000U, 2000U, 1365U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_OK,
        "full config ok"
    );
    expect_u32(config.generator.pulse_ticks, 1375U, "full config pulse ticks");
    expect_u32(config.idle_record_ticks, 2750U, "idle record ticks");
    expect_u32(config.startup_period_ns, 2000U, "startup copied");
    expect_u32(config.half_record_count, 1365U, "half count copied");
    expect_u32(config.axis_index, 0U, "axis copied");

    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 10000U, 2000U, 1365U, 0U, NULL),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config null"
    );
    expect_u32(
        step_stream_feeder_make_config(0U, 5000U, 2500U, 2000U, 10000U, 2000U, 1365U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config zero clock"
    );
    expect_u32(
        step_stream_feeder_make_config(275000000U, 5000U, 2500U, 2000U, 0U, 2000U, 1365U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config zero idle"
    );
    expect_u32(
        step_stream_feeder_make_config(275000000U, 0U, 2500U, 2000U, 10000U, 2000U, 1365U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config bad generator part"
    );
    expect_u32(
        step_stream_feeder_make_config(UINT32_MAX, 1U, 1U, 1U, UINT32_MAX, 2000U, 1365U, 0U, &config),
        STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
        "full config idle ticks overflow"
    );
}

static void test_init_validation(void) {
    StepStreamFeeder_t feeder;
    StepStreamFeederConfig_t config = make_config();
    StepStreamFeederPorts_t ports = make_ports();

    expect_u32(step_stream_feeder_init(NULL, &config, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "null feeder");
    expect_u32(step_stream_feeder_init(&feeder, NULL, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "null config");
    expect_u32(step_stream_feeder_init(&feeder, &config, NULL), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "null ports");

    StepStreamFeederPorts_t broken = ports;
    broken.prepare = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no prepare");
    broken = ports;
    broken.acquire_half = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no acquire");
    broken = ports;
    broken.commit_half = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no commit");
    broken = ports;
    broken.set_direction = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no direction");
    broken = ports;
    broken.start = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no start");
    broken = ports;
    broken.stop = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no stop");
    broken = ports;
    broken.pop_block = NULL;
    expect_u32(step_stream_feeder_init(&feeder, &config, &broken), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "no pop");

    StepStreamFeederConfig_t bad = config;
    bad.axis_index = MOTION_PLANNER_AXIS_COUNT;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "axis index");
    bad = config;
    bad.startup_period_ns = 0U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "startup");
    bad = config;
    bad.half_record_count = 0U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "half count");
    bad = config;
    bad.idle_record_ticks = 0U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "idle zero");
    bad = config;
    bad.idle_record_ticks = STEP_STREAM_GENERATOR_MAX_RECORD_TICKS + 1U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "idle too long");
    bad = config;
    bad.idle_record_ticks = config.generator.min_record_ticks - 1U;
    expect_u32(step_stream_feeder_init(&feeder, &bad, &ports), STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT, "idle too short");

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
    expect_u32(fake_last_direction, 1U, "positive direction");
    expect_u32(fake_start_calls, 1U, "started once");
    expect_u32(fake_last_startup_ns, 5000U, "startup period passed");

    expect_u32(fake_halves[0][0], 9999U, "step record ARR");
    expect_u32(fake_halves[0][1], 0U, "step record middle word");
    expect_u32(fake_halves[0][2], TEST_PULSE_TICKS, "first step pulse");
    expect_u32(count_step_records(0U), 3U, "three step records");
    expect_u32(fake_halves[0][3 * 3U], TEST_IDLE_TICKS - 1U, "idle record ARR");
    expect_u32(fake_halves[0][3 * 3U + 2U], 0U, "idle record has no pulse");
    expect_u32(count_step_records(1U), 0U, "second half is idle");
    expect_u32(fake_halves[1][0], TEST_IDLE_TICKS - 1U, "second half idle ARR");

    /* Half 0 played: it held steps, so the stream goes on with an idle refill. */
    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill service");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "still streaming");
    expect_u32(fake_stop_calls, 0U, "not stopped while the other half is idle but this one had steps");
    expect_u32(count_step_records(0U), 0U, "refilled idle");

    /* Half 1 played: both halves idle and no block, so the stream ends. */
    fake_empty[1] = 1U;
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

    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "idle refill");
    /* Both halves are idle now, but a block of the same direction arrives before the stream stops. */
    queue_block(&late);
    fake_empty[1] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "late block");
    expect_u32(fake_stop_calls, 0U, "stream is not stopped");
    expect_u32(count_step_records(1U), 2U, "late block fills the released half");
    expect_u32(fake_prepare_calls, 1U, "same stream");

    /* Half 0 is released next: it is filler, but the other half carries the late block, so the stream goes on. */
    fake_empty[0] = 1U;
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

    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 0");
    expect_u32(count_step_records(0U), 6U, "4 steps of the first block and 2 of the second");
    expect_u32(fake_direction_calls, 1U, "same direction needs no new DIR");

    fake_empty[1] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 1");
    expect_u32(count_step_records(1U), 0U, "nothing left");
    expect_u32(fake_stop_calls, 0U, "half 0 held steps, no stop yet");

    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill 0 again");
    expect_u32(fake_stop_calls, 0U, "the half just released held steps, so the stream goes on");

    fake_empty[1] = 1U;
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

    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill idle");
    expect_u32(fake_stop_calls, 0U, "no stop before the idle stretch is played");
    expect_u32(count_step_records(0U), 0U, "backward block is not mixed into the stream");

    fake_empty[1] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "stop and restart");
    expect_u32(fake_stop_calls, 1U, "stream stopped for the direction change");
    expect_u32(fake_prepare_calls, 2U, "second stream prepared");
    expect_u32(fake_direction_calls, 2U, "DIR set again");
    expect_u32(fake_direction_log[0], 1U, "first DIR positive");
    expect_u32(fake_direction_log[1], 0U, "second DIR negative");
    expect_u32(fake_start_calls, 2U, "second stream started");
    expect_u32(count_step_records(0U), 2U, "backward steps in the new stream");
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_STREAMING, "streaming backward");
}

static void test_block_without_axis_motion(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    /* Y-only block first: X idles through it and no DIR is written. */
    const MotionPlanBlock_t y_only = make_block(0, 3);
    const MotionPlanBlock_t x_move = make_block(2, 0);
    const MotionPlanBlock_t y_again = make_block(0, 2);
    queue_block(&y_only);
    queue_block(&y_again);
    queue_block(&x_move);

    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start on a Y block");
    expect_u32(fake_direction_calls, 0U, "no DIR for a block that does not move the axis");
    expect_u32(count_step_records(0U), 0U, "no X steps");
    expect_u32(fake_halves[0][0] != TEST_IDLE_TICKS - 1U ? 1U : 0U, 1U, "waiting records follow the block time");
    expect_u32(fake_start_calls, 1U, "stream started");

    /* The X block does not fit a stream whose direction is unknown: it waits for a new stream. */
    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill");
    fake_empty[1] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "refill again");
    fake_empty[0] = 1U;
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "restart for X");
    expect_u32(fake_stop_calls, 1U, "stopped once");
    expect_u32(fake_direction_calls, 1U, "DIR written for the X stream");
    expect_u32(fake_last_direction, 1U, "X positive");
    expect_u32(fake_start_calls, 2U, "X stream started");
}

static void test_stream_runs_through_a_long_block_without_axis_motion(void) {
    StepStreamFeeder_t feeder;
    setup(&feeder);
    /* 1000 Y steps last 10 ms: X only waits, but the stream must play the whole block before it may stop. */
    const MotionPlanBlock_t y_long = make_block(0, 1000);
    queue_block(&y_long);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start on a long Y block");

    uint32_t releases = 0U;
    while (step_stream_feeder_get_phase(&feeder) == STEP_STREAM_FEEDER_PHASE_STREAMING && releases < 100U) {
        StepStreamFeederDiagnostics_t diagnostics;
        step_stream_feeder_get_diagnostics(&feeder, &diagnostics);
        const uint32_t completed_before = diagnostics.blocks_completed;
        fake_empty[releases % 2U] = 1U;
        ++releases;
        expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "service while the block plays");
        if (fake_stop_calls != 0U) {
            expect_u32(completed_before, 1U, "the stream stops only after the block was fully written");
        }
    }
    expect_u32(fake_stop_calls, 1U, "the stream stops once the block is played");
    expect_true(releases > 4U, "the block spans many halves");
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
    setup(&feeder);
    const MotionPlanBlock_t forward = make_block(3, 0);
    const MotionPlanBlock_t backward = make_block(-2, 0);
    queue_block(&forward);
    queue_block(&backward);
    expect_u32(step_stream_feeder_service(&feeder), STEP_STREAM_FEEDER_STATUS_OK, "start");
    expect_u32(feeder.held_valid, 1U, "backward block is held");

    step_stream_feeder_reset(&feeder);
    expect_u32(step_stream_feeder_get_phase(&feeder), STEP_STREAM_FEEDER_PHASE_IDLE, "idle after reset");
    expect_u32(feeder.held_valid, 0U, "held block dropped");
    expect_u32(feeder.generator_active, 0U, "generator dropped");
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
    test_block_without_axis_motion();
    test_stream_runs_through_a_long_block_without_axis_motion();
    test_generator_errors();
    test_axis_errors();
    test_counters_saturate();
    test_reset_drops_state();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "%lu test(s) failed\n", (unsigned long)test_failures);
        return 1;
    }
    (void)printf("step_stream_feeder_unit_tests passed\n");
    return 0;
}

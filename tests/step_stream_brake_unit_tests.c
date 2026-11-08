#include "motion/step_stream_brake.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_CLOCK_HZ 275000000U
#define TEST_PULSE_TICKS 1375U
#define TEST_MIN_PERIOD_TICKS 2063U
#define TEST_IDLE_TICKS 2750U
#define TEST_DECELERATION 600000U
#define TEST_MAX_RATE_HZ 50000U
#define TEST_CAPACITY 4096U

static uint16_t test_storage[TEST_CAPACITY];

static StepStreamBrakeConfig_t test_config(void) {
    return (StepStreamBrakeConfig_t){
        .timer_clock_hz = TEST_CLOCK_HZ,
        .pulse_ticks = TEST_PULSE_TICKS,
        .min_period_ticks = TEST_MIN_PERIOD_TICKS,
        .idle_record_ticks = TEST_IDLE_TICKS,
        .deceleration_steps_per_s2 = TEST_DECELERATION,
        .max_step_rate_hz = TEST_MAX_RATE_HZ,
    };
}

static void test_init_rejects_bad_arguments(void) {
    StepStreamBrake_t brake;
    const StepStreamBrakeConfig_t good = test_config();
    assert(step_stream_brake_init(NULL, &good, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    assert(step_stream_brake_init(&brake, NULL, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    assert(step_stream_brake_init(&brake, &good, NULL, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    assert(step_stream_brake_init(&brake, &good, test_storage, 0U) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    assert(step_stream_brake_init(&brake, &good, test_storage, 65537U) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);

    StepStreamBrakeConfig_t bad = good;
    bad.timer_clock_hz = 0U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.pulse_ticks = 0U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.min_period_ticks = bad.pulse_ticks;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.min_period_ticks = STEP_STREAM_BRAKE_MAX_PERIOD_TICKS + 1U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.idle_record_ticks = 0U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.idle_record_ticks = STEP_STREAM_BRAKE_MAX_PERIOD_TICKS + 1U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.deceleration_steps_per_s2 = 0U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
    bad = good;
    bad.max_step_rate_hz = 0U;
    assert(step_stream_brake_init(&brake, &bad, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
}

static void test_init_rejects_an_empty_ramp(void) {
    StepStreamBrake_t brake;
    /* The ramp ends at 4197 Hz, so a start rate below that leaves nothing to brake. */
    StepStreamBrakeConfig_t config = test_config();
    config.max_step_rate_hz = 100U;
    assert(step_stream_brake_init(&brake, &config, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);

    /* A shortest period as long as the slowest record is already reached at the first entry. */
    config = test_config();
    config.min_period_ticks = STEP_STREAM_BRAKE_MAX_PERIOD_TICKS;
    assert(step_stream_brake_init(&brake, &config, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_INVALID_ARGUMENT);
}

static void test_init_reports_capacity(void) {
    StepStreamBrake_t brake;
    const StepStreamBrakeConfig_t config = test_config();
    assert(step_stream_brake_init(&brake, &config, test_storage, 10U) == STEP_STREAM_BRAKE_STATUS_CAPACITY);
}

static void test_ramp_follows_the_deceleration(void) {
    StepStreamBrake_t brake;
    const StepStreamBrakeConfig_t config = test_config();
    assert(step_stream_brake_init(&brake, &config, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_OK);

    /* (50000^2 - 4197^2) / (2 * 600000) = 2068.6, so 2069 entries from k = 0 to 2068: about 1 mm at 2 steps/um. */
    assert(brake.count >= 2068U && brake.count <= 2070U);
    for (uint32_t index = 1U; index < brake.count; ++index) {
        assert(brake.arr[index] >= brake.arr[index - 1U]);
    }
    /* Fastest first: 50 kHz is a period of 5500 ticks. */
    assert(brake.arr[0] + 1U >= 5499U && brake.arr[0] + 1U <= 5501U);

    /* Every entry sits on the constant deceleration: rate^2 = end_rate^2 + 2 * a * (steps before the stop). */
    const double end_rate = 4197.0;
    for (uint32_t index = 0U; index < brake.count; ++index) {
        const double rate = (double)TEST_CLOCK_HZ / (double)(brake.arr[index] + 1U);
        const double steps_before_stop = (double)(brake.count - 1U - index);
        const double expected = end_rate * end_rate + 2.0 * (double)TEST_DECELERATION * steps_before_stop;
        const double error = (rate * rate - expected) / expected;
        assert(error < 0.002 && error > -0.002);
    }
    /* Slowest last: a period that still fits one record. */
    assert(brake.arr[brake.count - 1U] + 1U <= STEP_STREAM_BRAKE_MAX_PERIOD_TICKS);
    assert(brake.arr[brake.count - 1U] + 1U >= 65000U);
    for (uint32_t index = 0U; index < brake.count; ++index) {
        assert(brake.arr[index] + 1U >= TEST_MIN_PERIOD_TICKS);
    }
}

static void test_ramp_stops_at_the_shortest_period(void) {
    StepStreamBrake_t brake;
    StepStreamBrakeConfig_t config = test_config();
    config.max_step_rate_hz = 200000U;
    config.min_period_ticks = 5500U;
    assert(step_stream_brake_init(&brake, &config, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_OK);
    assert(brake.arr[0] + 1U >= 5500U);
    assert(brake.arr[0] + 1U <= 5510U);
}

static void test_find_start(void) {
    StepStreamBrake_t brake;
    const StepStreamBrakeConfig_t config = test_config();
    assert(step_stream_brake_init(&brake, &config, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_OK);

    assert(step_stream_brake_find_start(NULL, 100U) == 0U);
    StepStreamBrake_t empty = {0};
    assert(step_stream_brake_find_start(&empty, 100U) == 0U);

    /* Faster than the ramp starts at its beginning; slower than all of it leaves nothing to brake. */
    assert(step_stream_brake_find_start(&brake, 1U) == 0U);
    assert(step_stream_brake_find_start(&brake, brake.arr[0]) == 0U);
    assert(step_stream_brake_find_start(&brake, UINT16_MAX) == brake.count);
    assert(step_stream_brake_find_start(&brake, (uint16_t)(brake.arr[brake.count - 1U] + 1U)) == brake.count);

    const uint32_t middle = brake.count / 2U;
    const uint32_t found = step_stream_brake_find_start(&brake, brake.arr[middle]);
    assert(brake.arr[found] >= brake.arr[middle]);
    assert(found == 0U || brake.arr[found - 1U] < brake.arr[middle]);
    assert(step_stream_brake_find_start(&brake, (uint16_t)(brake.arr[middle] + 1U)) >= found);
}

static void test_cursor_and_fill(void) {
    StepStreamBrake_t brake;
    const StepStreamBrakeConfig_t config = test_config();
    assert(step_stream_brake_init(&brake, &config, test_storage, TEST_CAPACITY) == STEP_STREAM_BRAKE_STATUS_OK);

    step_stream_brake_cursor_init(NULL, &brake, 0U);
    StepStreamBrakeCursor_t cursor;
    step_stream_brake_cursor_init(&cursor, NULL, 5U);
    assert(cursor.brake == NULL && cursor.next == 0U);
    step_stream_brake_cursor_init(&cursor, &brake, brake.count + 50U);
    assert(cursor.next == brake.count);

    /* Two records of the ramp, then filler. */
    step_stream_brake_cursor_init(&cursor, &brake, brake.count - 2U);
    uint16_t records[5U * STEP_STREAM_BRAKE_RECORD_WORDS] = {0};
    assert(step_stream_brake_fill(&cursor, records, 5U) == 2U);
    assert(cursor.next == brake.count);
    for (uint32_t index = 0U; index < 2U; ++index) {
        assert(records[index * 3U] == brake.arr[brake.count - 2U + index]);
        assert(records[index * 3U + 1U] == 0U);
        assert(records[index * 3U + 2U] == TEST_PULSE_TICKS);
    }
    for (uint32_t index = 2U; index < 5U; ++index) {
        assert(records[index * 3U] == TEST_IDLE_TICKS - 1U);
        assert(records[index * 3U + 1U] == 0U);
        assert(records[index * 3U + 2U] == 0U);
    }

    /* An exhausted ramp writes only filler and reports no step. */
    assert(step_stream_brake_fill(&cursor, records, 5U) == 0U);

    assert(step_stream_brake_fill(NULL, records, 5U) == 0U);
    StepStreamBrakeCursor_t detached = {0};
    assert(step_stream_brake_fill(&detached, records, 5U) == 0U);
    step_stream_brake_cursor_init(&cursor, &brake, 0U);
    assert(step_stream_brake_fill(&cursor, NULL, 5U) == 0U);
    assert(cursor.next == 0U);
    assert(step_stream_brake_fill(&cursor, records, 0U) == 0U);
}

int main(void) {
    test_init_rejects_bad_arguments();
    test_init_rejects_an_empty_ramp();
    test_init_reports_capacity();
    test_ramp_follows_the_deceleration();
    test_ramp_stops_at_the_shortest_period();
    test_find_start();
    test_cursor_and_fill();
    (void)puts("step_stream_brake_unit_tests passed");
    return 0;
}

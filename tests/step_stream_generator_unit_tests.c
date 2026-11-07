#include "motion/step_stream_generator.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

static int check_condition(int condition, const char *expression, int line) {
    if (!condition) {
        fprintf(stderr, "line %d: %s\n", line, expression);
        return 0;
    }
    return 1;
}

#define CHECK(condition) \
    do { \
        if (!check_condition((condition), #condition, __LINE__)) { \
            return 0; \
        } \
    } while (0)

static StepStreamGeneratorConfig_t default_config(void) {
    const StepStreamGeneratorConfig_t config = {
        .timer_clock_hz = 1000000000U,
        .pulse_ticks = 5U,
        .low_ticks = 3U,
        .min_record_ticks = 1U,
    };
    return config;
}

static size_t record_period(const uint16_t *records, size_t index) {
    return (size_t)records[index * STEP_STREAM_GENERATOR_RECORD_WORDS] + 1U;
}

static size_t count_pulses(const uint16_t *records, size_t record_count) {
    size_t pulses = 0U;
    for (size_t index = 0U; index < record_count; ++index) {
        if (records[index * STEP_STREAM_GENERATOR_RECORD_WORDS + 2U] != 0U) {
            ++pulses;
        }
    }
    return pulses;
}

static size_t collect_records(
    StepStreamGenerator_t *generator,
    uint16_t *records,
    size_t capacity,
    size_t max_records,
    StepStreamGeneratorStatus_t *last_status
) {
    size_t total = 0U;
    StepStreamGeneratorStatus_t status = STEP_STREAM_GENERATOR_STATUS_OK;
    while (status != STEP_STREAM_GENERATOR_STATUS_DONE) {
        size_t written = 0U;
        status = step_stream_generator_fill(
            generator,
            records + total * STEP_STREAM_GENERATOR_RECORD_WORDS,
            max_records,
            &written
        );
        CHECK(written <= max_records);
        CHECK(total + written <= capacity);
        total += written;
        CHECK(status == STEP_STREAM_GENERATOR_STATUS_OK ||
              status == STEP_STREAM_GENERATOR_STATUS_DONE);
        CHECK(written != 0U);
    }
    if (last_status != NULL) {
        *last_status = status;
    }
    return total;
}

static int test_invalid_arguments(void) {
    StepStreamGeneratorConfig_t config = default_config();
    StepStreamGenerator_t generator = {0};
    StepStreamGeneratorCarry_t carry = {.ns_to_ticks_remainder = 0U};
    uint16_t records[3] = {0};
    size_t written = 99U;

    CHECK(step_stream_generator_init(NULL, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_init(&generator, NULL, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_init(&generator, &config, 1U, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);

    config.timer_clock_hz = 0U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    config = default_config();
    config.pulse_ticks = 0U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    config.pulse_ticks = STEP_STREAM_GENERATOR_MAX_RECORD_TICKS;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    config = default_config();
    config.min_record_ticks = 0U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    config.min_record_ticks = STEP_STREAM_GENERATOR_MAX_RECORD_TICKS + 1U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    config = default_config();
    config.low_ticks = STEP_STREAM_GENERATOR_MAX_RECORD_TICKS;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);

    carry.ns_to_ticks_remainder = (uint32_t)STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND;
    config = default_config();
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, &carry) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);

    CHECK(step_stream_generator_fill(NULL, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(written == 0U);
    CHECK(step_stream_generator_init(&generator, &config, 1U, 100U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, NULL, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(written == 0U);
    CHECK(step_stream_generator_fill(&generator, records, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_fill(&generator, records, 0U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_fill(&generator, records, SIZE_MAX, &written) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);

    StepStreamGenerator_t uninitialized = {0};
    CHECK(step_stream_generator_fill(&uninitialized, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_get_carry(NULL, &carry) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_get_carry(&generator, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_get_carry(&uninitialized, &carry) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    return 1;
}

static MotionPlanBlock_t valid_block(void) {
    MotionPlanBlock_t block = {0};
    block.step_delta[0] = 3;
    block.master_step_count = 10U;
    block.step_event_rate_numerator = 4U;
    block.step_event_rate_denominator = 1U;
    return block;
}

static int test_block_wrapper_and_duration(void) {
    StepStreamGeneratorConfig_t config = default_config();
    config.timer_clock_hz = 1000U;
    config.pulse_ticks = 10U;
    config.low_ticks = 5U;
    config.min_record_ticks = 5U;
    MotionPlanBlock_t block = valid_block();
    block.step_delta[0] = -3;
    StepStreamGenerator_t generator;
    uint16_t records[12] = {0};
    size_t written = 0U;
    StepStreamGeneratorCarry_t carry = {0};

    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 3U);
    CHECK(record_period(records, 0U) == 833U);
    CHECK(record_period(records, 1U) == 833U);
    CHECK(record_period(records, 2U) == 834U);
    CHECK(count_pulses(records, written) == 3U);
    CHECK(records[1] == 0U);
    CHECK(step_stream_generator_fill(&generator, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 0U);
    CHECK(step_stream_generator_get_carry(&generator, &carry) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(carry.ns_to_ticks_remainder == 0U);

    CHECK(step_stream_generator_init_from_block(NULL, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_init_from_block(&generator, NULL, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_init_from_block(&generator, &config, NULL, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block,
                                                MOTION_PLANNER_AXIS_COUNT, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);

    block.step_delta[0] = 11;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    block = valid_block();
    block.master_step_count = 0U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    block.step_delta[0] = 0;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    block = valid_block();
    block.step_event_rate_numerator = 0U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    block.step_event_rate_numerator = 4U;
    block.step_event_rate_denominator = 0U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_INVALID_ARGUMENT);
    block = valid_block();
    block.step_event_rate_denominator = UINT64_MAX / STEP_STREAM_GENERATOR_NANOSECONDS_PER_SECOND + 1U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW);
    block = valid_block();
    block.master_step_count = UINT64_MAX;
    block.step_event_rate_denominator = 1U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW);
    block = valid_block();
    block.step_delta[0] = 0;
    block.master_step_count = 2U;
    block.step_event_rate_numerator = 3U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    return 1;
}

static int test_idle_and_split_boundaries(void) {
    StepStreamGeneratorConfig_t config = default_config();
    config.min_record_ticks = 100U;
    uint16_t records[16] = {0};
    size_t written = 0U;
    StepStreamGenerator_t generator;

    config.timer_clock_hz = 1U;
    config.pulse_ticks = 1U;
    config.low_ticks = 0U;
    config.min_record_ticks = 1U;
    CHECK(step_stream_generator_init(&generator, &config, 0U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_IDLE_INTERVAL_TOO_SHORT);
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT);

    config = default_config();
    config.pulse_ticks = 1U;
    config.low_ticks = 0U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 1U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT);

    config = default_config();

    CHECK(step_stream_generator_init(&generator, &config, 0U, 65536U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 1U);
    CHECK(record_period(records, 0U) == 65536U);
    CHECK(records[2] == 0U);

    CHECK(step_stream_generator_init(&generator, &config, 0U, 65537U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 2U);
    CHECK(record_period(records, 0U) == 32769U);
    CHECK(record_period(records, 1U) == 32768U);
    CHECK(count_pulses(records, written) == 0U);

    CHECK(step_stream_generator_init(&generator, &config, 0U, 65537U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(written == 1U);
    CHECK(step_stream_generator_fill(&generator, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 1U);

    config.min_record_ticks = 32769U;
    CHECK(step_stream_generator_init(&generator, &config, 0U, 65537U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_IDLE_INTERVAL_TOO_SHORT);
    CHECK(written == 0U);

    config.min_record_ticks = 1U;
    config.pulse_ticks = 10U;
    config.low_ticks = 5U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 65537U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 2U);
    CHECK(records[2] == 10U);
    CHECK(records[5] == 0U);

    config.pulse_ticks = 40000U;
    config.low_ticks = 1U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 65537U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT);
    config.pulse_ticks = 5U;
    config.low_ticks = 6U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 10U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT);

    config.pulse_ticks = 5U;
    config.low_ticks = 3U;
    config.min_record_ticks = 11U;
    CHECK(step_stream_generator_init(&generator, &config, 1U, 10U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_STEP_INTERVAL_TOO_SHORT);
    return 1;
}

static int test_resume_and_exact_sums(void) {
    StepStreamGeneratorConfig_t config = default_config();
    config.timer_clock_hz = 1000000U;
    config.min_record_ticks = 1U;
    enum { CAPACITY = 64 };
    uint16_t one_call[CAPACITY * STEP_STREAM_GENERATOR_RECORD_WORDS] = {0};
    uint16_t resumed[CAPACITY * STEP_STREAM_GENERATOR_RECORD_WORDS] = {0};
    StepStreamGenerator_t generator;
    StepStreamGeneratorStatus_t status;
    size_t one_count;
    size_t resumed_count;

    CHECK(step_stream_generator_init(&generator, &config, 7U, 7000000U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    one_count = collect_records(&generator, one_call, CAPACITY, CAPACITY, &status);
    CHECK(status == STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(one_count == 7U);
    CHECK(count_pulses(one_call, one_count) == 7U);

    for (size_t max_records = 1U; max_records <= 7U; max_records += max_records == 1U ? 1U : 5U) {
        CHECK(step_stream_generator_init(&generator, &config, 7U, 7000000U, NULL) ==
              STEP_STREAM_GENERATOR_STATUS_OK);
        memset(resumed, 0, sizeof(resumed));
        resumed_count = collect_records(&generator, resumed, CAPACITY, max_records, &status);
        CHECK(status == STEP_STREAM_GENERATOR_STATUS_DONE);
        CHECK(resumed_count == one_count);
        CHECK(memcmp(resumed, one_call, one_count * STEP_STREAM_GENERATOR_RECORD_WORDS * sizeof(uint16_t)) == 0);
    }

    config.timer_clock_hz = 1000000000U;
    config.pulse_ticks = 1U;
    config.low_ticks = 0U;
    const uint64_t exact_durations[] = {1U, 2U, 3U, 7U, 10U, 65536U, 65537U};
    const uint64_t step_counts[] = {0U, 1U, 2U, 3U, 7U};
    for (size_t duration_index = 0U; duration_index < ARRAY_SIZE(exact_durations); ++duration_index) {
        for (size_t step_index = 0U; step_index < ARRAY_SIZE(step_counts); ++step_index) {
            const uint64_t duration = exact_durations[duration_index];
            const uint64_t step_count = step_counts[step_index];
            if (step_count != 0U && duration < 2U * step_count) {
                continue;
            }
            CHECK(step_stream_generator_init(&generator, &config, step_count, duration, NULL) ==
                  STEP_STREAM_GENERATOR_STATUS_OK);
            uint16_t records[256 * STEP_STREAM_GENERATOR_RECORD_WORDS] = {0};
            const size_t record_count = collect_records(&generator, records, 256U, 256U, &status);
            CHECK(status == STEP_STREAM_GENERATOR_STATUS_DONE);
            uint64_t period_sum = 0U;
            for (size_t index = 0U; index < record_count; ++index) {
                CHECK(record_period(records, index) <= STEP_STREAM_GENERATOR_MAX_RECORD_TICKS);
                CHECK(record_period(records, index) >= config.min_record_ticks);
                period_sum += record_period(records, index);
            }
            CHECK(period_sum == duration);
            CHECK(count_pulses(records, record_count) == (step_count == 0U ? 0U : (size_t)step_count));
        }
    }
    return 1;
}

static int test_carry_and_overflow(void) {
    StepStreamGeneratorConfig_t config = default_config();
    config.timer_clock_hz = 3U;
    config.pulse_ticks = 1U;
    config.low_ticks = 0U;
    StepStreamGeneratorCarry_t carry = {0};
    StepStreamGeneratorCarry_t next_carry = {0};
    StepStreamGenerator_t generator;
    uint16_t records[4] = {0};
    uint64_t total_ticks = 0U;
    const uint64_t block_duration_ns = 400000000U;

    for (size_t block = 0U; block < 1000U; ++block) {
        size_t written = 0U;
        CHECK(step_stream_generator_init(&generator, &config, 0U, block_duration_ns, &carry) ==
              STEP_STREAM_GENERATOR_STATUS_OK);
        CHECK(step_stream_generator_fill(&generator, records, ARRAY_SIZE(records) / 3U, &written) ==
              STEP_STREAM_GENERATOR_STATUS_DONE);
        CHECK(written == 1U);
        total_ticks += record_period(records, 0U);
        CHECK(step_stream_generator_get_carry(&generator, &next_carry) ==
              STEP_STREAM_GENERATOR_STATUS_OK);
        carry = next_carry;
    }
    CHECK(total_ticks == 1200U);
    CHECK(carry.ns_to_ticks_remainder == 0U);

    carry.ns_to_ticks_remainder = 999999999U;
    CHECK(step_stream_generator_init(&generator, &config, 0U, block_duration_ns, &carry) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    size_t written = 0U;
    CHECK(step_stream_generator_fill(&generator, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);

    config.timer_clock_hz = UINT32_MAX;
    CHECK(step_stream_generator_init(&generator, &config, 0U, UINT64_MAX, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, 1U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_ARITHMETIC_OVERFLOW);
    return 1;
}

static int test_speed_profiles(void) {
    StepStreamGeneratorConfig_t config = {
        .timer_clock_hz = 275000000U,
        .pulse_ticks = 1000U,
        .low_ticks = 500U,
        .min_record_ticks = 100U,
    };
    MotionPlanBlock_t block = {0};
    block.step_delta[0] = 3;
    block.master_step_count = 3U;
    block.step_event_rate_numerator = 150000U;
    block.step_event_rate_denominator = 1U;
    StepStreamGenerator_t generator;
    uint16_t records[32 * STEP_STREAM_GENERATOR_RECORD_WORDS] = {0};
    size_t written = 0U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, 32U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 3U);
    CHECK(count_pulses(records, written) == 3U);
    CHECK(record_period(records, 0U) == 1833U);
    CHECK(record_period(records, 1U) == 1833U);
    CHECK(record_period(records, 2U) == 1834U);

    block.master_step_count = 1U;
    block.step_delta[0] = 1;
    block.step_event_rate_numerator = 400U;
    config.pulse_ticks = 1375U;
    config.low_ticks = 688U;
    CHECK(step_stream_generator_init_from_block(&generator, &config, &block, 0U, NULL) ==
          STEP_STREAM_GENERATOR_STATUS_OK);
    CHECK(step_stream_generator_fill(&generator, records, 32U, &written) ==
          STEP_STREAM_GENERATOR_STATUS_DONE);
    CHECK(written == 11U);
    CHECK(count_pulses(records, written) == 1U);
    size_t period_sum = 0U;
    for (size_t index = 0U; index < written; ++index) {
        CHECK(record_period(records, index) == 62500U);
        period_sum += record_period(records, index);
    }
    CHECK(period_sum == 687500U);
    return 1;
}

int main(void) {
    if (!test_invalid_arguments() || !test_block_wrapper_and_duration() ||
        !test_idle_and_split_boundaries() || !test_resume_and_exact_sums() ||
        !test_carry_and_overflow() || !test_speed_profiles()) {
        return 1;
    }
    puts("step_stream_generator_unit_tests: PASS");
    return 0;
}

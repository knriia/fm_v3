#include "step_stream_format.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static void test_fill_accepts_period_boundaries(void) {
    uint16_t buffer[STEP_STREAM_FORMAT_DMA_BURST_LENGTH];

    assert(step_stream_format_fill(buffer, 1U, 1U, 0U) == STEP_STREAM_FORMAT_STATUS_OK);
    assert(buffer[0] == 0U);
    assert(buffer[1] == 0U);
    assert(buffer[2] == 0U);

    assert(step_stream_format_fill(buffer, 1U, 65536U, 65535U) == STEP_STREAM_FORMAT_STATUS_OK);
    assert(buffer[0] == UINT16_MAX);
    assert(buffer[1] == 0U);
    assert(buffer[2] == UINT16_MAX);
}

static void test_fill_rejects_invalid_pulse_and_period(void) {
    uint16_t buffer[STEP_STREAM_FORMAT_DMA_BURST_LENGTH];

    assert(step_stream_format_fill(buffer, 1U, 0U, 0U) == STEP_STREAM_FORMAT_STATUS_INVALID_ARGUMENT);
    assert(step_stream_format_fill(buffer, 1U, 65537U, 1U) == STEP_STREAM_FORMAT_STATUS_INVALID_ARGUMENT);
    assert(step_stream_format_fill(buffer, 1U, 100U, 100U) == STEP_STREAM_FORMAT_STATUS_INVALID_ARGUMENT);
    assert(step_stream_format_fill(buffer, 1U, 100U, 101U) == STEP_STREAM_FORMAT_STATUS_INVALID_ARGUMENT);
}

static void test_steps_done_accounts_for_burst_and_preload(void) {
    const uint32_t record_count = 4U;
    const uint32_t total_transfers = record_count * STEP_STREAM_FORMAT_DMA_BURST_LENGTH;
    const uint32_t half_transfers = total_transfers / 2U;

    assert(step_stream_format_steps_done(0U, record_count, total_transfers) == 0U);
    assert(step_stream_format_steps_done(0U, record_count, total_transfers - STEP_STREAM_FORMAT_DMA_BURST_LENGTH) == 0U);
    assert(step_stream_format_steps_done(0U, record_count, half_transfers) == 1U);
    assert(step_stream_format_steps_done(1U, record_count, half_transfers) == 1U);
    assert(step_stream_format_steps_done(1U, record_count, total_transfers) == record_count - 1U);
    assert(step_stream_format_steps_done(2U, record_count, total_transfers) == record_count - 1U);
}

static void test_steps_done_rejects_invalid_ndtr_and_handles_zero_played_records(void) {
    const uint32_t record_count = 4U;
    const uint32_t total_transfers = record_count * STEP_STREAM_FORMAT_DMA_BURST_LENGTH;

    assert(step_stream_format_steps_done(0U, record_count, total_transfers) == 0U);
    assert(step_stream_format_steps_done(0U, record_count, total_transfers + 1U) == 0U);
    assert(step_stream_format_steps_done(0U, 3U, total_transfers) == 0U);
}

int main(void) {
    test_fill_accepts_period_boundaries();
    test_fill_rejects_invalid_pulse_and_period();
    test_steps_done_accounts_for_burst_and_preload();
    test_steps_done_rejects_invalid_ndtr_and_handles_zero_played_records();
    puts("Step stream format unit tests passed");
    return 0;
}

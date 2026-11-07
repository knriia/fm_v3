#include "step_stream_format.h"

#include <stdint.h>

#define STEP_STREAM_FORMAT_MAX_PERIOD_TICKS 65536U

StepStreamFormatStatus_t
step_stream_format_fill(uint16_t *buffer, size_t record_count, uint32_t period_ticks, uint32_t pulse_ticks) {
    if (buffer == NULL || record_count == 0U || period_ticks == 0U ||
        period_ticks > STEP_STREAM_FORMAT_MAX_PERIOD_TICKS || pulse_ticks >= period_ticks ||
        record_count > SIZE_MAX / STEP_STREAM_FORMAT_DMA_BURST_LENGTH) {
        return STEP_STREAM_FORMAT_STATUS_INVALID_ARGUMENT;
    }

    const uint16_t arr = (uint16_t)(period_ticks - 1U);
    const uint16_t ccr = (uint16_t)pulse_ticks;
    for (size_t record = 0U; record < record_count; ++record) {
        const size_t offset = record * STEP_STREAM_FORMAT_DMA_BURST_LENGTH;
        buffer[offset] = arr;
        buffer[offset + 1U] = 0U;
        buffer[offset + 2U] = ccr;
    }
    return STEP_STREAM_FORMAT_STATUS_OK;
}

uint32_t step_stream_format_steps_done(uint32_t completed_half_buffers, uint32_t record_count, uint32_t ndtr) {
    if (record_count == 0U || (record_count & 1U) != 0U) {
        return 0U;
    }

    const uint64_t total_transfers = (uint64_t)record_count * STEP_STREAM_FORMAT_DMA_BURST_LENGTH;
    if ((uint64_t)ndtr > total_transfers) {
        return 0U;
    }

    const uint64_t half_records = record_count / 2U;
    const uint64_t half_transfers = half_records * STEP_STREAM_FORMAT_DMA_BURST_LENGTH;
    const uint64_t completed_records = (uint64_t)completed_half_buffers * half_records;
    const uint64_t half_position = (completed_half_buffers & 1U) == 0U ? 0U : 1U;
    uint64_t current_transfers;

    if (half_position == 0U) {
        current_transfers = (uint64_t)ndtr >= half_transfers ? total_transfers - ndtr : 0U;
    } else {
        current_transfers = (uint64_t)ndtr <= half_transfers ? half_transfers - ndtr : half_transfers;
    }

    const uint64_t played_records = completed_records + current_transfers / STEP_STREAM_FORMAT_DMA_BURST_LENGTH;
    const uint64_t steps = played_records == 0U ? 0U : played_records - 1U;
    return steps > UINT32_MAX ? UINT32_MAX : (uint32_t)steps;
}

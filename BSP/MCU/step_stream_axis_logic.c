#include "step_stream_axis_logic.h"

#include "step_stream_format.h"

#define STEP_STREAM_AXIS_LOGIC_SNAPSHOT_ATTEMPTS 4U

uint8_t step_stream_axis_logic_prepare_stand_ring(StepStreamRing_t *ring) {
    step_stream_ring_init(ring);
    const uint32_t first = (uint32_t)step_stream_ring_mark_filled(ring, 0U);
    const uint32_t second = (uint32_t)step_stream_ring_mark_filled(ring, 1U);
    const uint32_t started = (uint32_t)step_stream_ring_start(ring);
    /* STEP_STREAM_RING_STATUS_OK is 0, so any failure leaves a non-zero OR. */
    return (uint8_t)((first | second | started) != 0U);
}

StepStreamAxisLogicAction_t
step_stream_axis_logic_half_event(StepStreamRing_t *ring, uint8_t half_transfer, uint8_t auto_refill) {
    const StepStreamRingStatus_t status =
        half_transfer != 0U ? step_stream_ring_on_half_transfer(ring) : step_stream_ring_on_transfer_complete(ring);
    if (status == STEP_STREAM_RING_STATUS_UNDERRUN) {
        return STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN;
    }
    if (status != STEP_STREAM_RING_STATUS_OK) {
        return STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_SEQUENCE;
    }
    if (auto_refill != 0U) {
        /* The played half is the released one: half 0 after HT, half 1 after TC. */
        (void)step_stream_ring_mark_filled(ring, half_transfer != 0U ? 0U : 1U);
    }
    return STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED;
}

uint32_t step_stream_axis_logic_read_steps(
    const StepStreamAxisSnapshotSource_t *source,
    uint32_t completed_half_buffers,
    uint32_t record_count
) {
    uint32_t pending = 0U;
    uint32_t ndtr = 0U;
    for (uint32_t attempt = 0U; attempt < STEP_STREAM_AXIS_LOGIC_SNAPSHOT_ATTEMPTS; ++attempt) {
        const uint32_t pending_before = source->pending_half_buffers();
        ndtr = source->ndtr();
        pending = source->pending_half_buffers();
        if (pending == pending_before) {
            break;
        }
    }
    return step_stream_format_steps_done(completed_half_buffers + pending, record_count, ndtr);
}

uint32_t step_stream_axis_logic_boundary_steps(uint32_t completed_half_buffers, uint32_t record_count) {
    const uint32_t total_transfers = record_count * STEP_STREAM_FORMAT_DMA_BURST_LENGTH;
    const uint32_t ndtr = (completed_half_buffers & 1U) == 0U ? total_transfers : total_transfers / 2U;
    return step_stream_format_steps_done(completed_half_buffers, record_count, ndtr);
}

uint32_t step_stream_axis_logic_stale_steps(uint32_t raw_steps, uint32_t valid_steps) {
    return raw_steps > valid_steps ? raw_steps - valid_steps : 0U;
}

uint8_t step_stream_axis_logic_poll(uint8_t (*is_done)(void), uint32_t max_polls) {
    for (uint32_t poll = 0U; poll < max_polls; ++poll) {
        if (is_done() != 0U) {
            return 1U;
        }
    }
    return 0U;
}

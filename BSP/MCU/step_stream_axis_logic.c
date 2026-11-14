#include "step_stream_axis_logic.h"

#include "step_stream_format.h"

#include <stddef.h>

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
        const uint32_t pending_before = source->pending_half_buffers(source->context);
        ndtr = source->ndtr(source->context);
        pending = source->pending_half_buffers(source->context);
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

uint8_t step_stream_axis_logic_poll(uint8_t (*is_done)(const void *context), const void *context, uint32_t max_polls) {
    for (uint32_t poll = 0U; poll < max_polls; ++poll) {
        if (is_done(context) != 0U) {
            return 1U;
        }
    }
    return 0U;
}

void step_stream_axis_logic_brake_reset(StepStreamAxisBrake_t *brake) {
    if (brake == NULL) {
        return;
    }
    brake->cursor.brake = NULL;
    brake->cursor.next = 0U;
    brake->active = 0U;
    brake->half_has_steps[0] = 0U;
    brake->half_has_steps[1] = 0U;
    brake->deferred_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
}

static uint16_t *step_stream_axis_logic_record(uint16_t *buffer, uint32_t record) {
    return &buffer[(size_t)record * STEP_STREAM_FORMAT_DMA_BURST_LENGTH];
}

StepStreamAxisBrakeBegin_t step_stream_axis_logic_brake_begin(
    StepStreamAxisBrake_t *brake,
    const StepStreamBrake_t *table,
    StepStreamRing_t *ring,
    uint16_t *buffer,
    uint32_t record_count,
    uint32_t steps_done,
    uint32_t margin_records,
    uint8_t held_half
) {
    if (brake == NULL || table == NULL || table->count == 0U || ring == NULL || buffer == NULL || record_count < 4U ||
        (record_count & 1U) != 0U || margin_records == 0U || margin_records >= record_count / 2U ||
        ring->state != STEP_STREAM_RING_STATE_RUNNING) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT;
    }

    const uint32_t half_records = record_count / 2U;
    /* The DMA has handed this many records to the timer; the last of them waits in the preload. */
    const uint32_t played = (uint32_t)(((uint64_t)steps_done + 1U) % record_count);
    const uint8_t playing = (uint8_t)(played / half_records);
    if (playing != ring->current_half) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_RETRY;
    }
    const uint8_t other = (uint8_t)(1U - playing);
    const uint8_t other_state = ring->half_states[other];
    if (ring->half_states[playing] != STEP_STREAM_RING_HALF_PLAYING ||
        (other_state != STEP_STREAM_RING_HALF_FULL && other_state != STEP_STREAM_RING_HALF_EMPTY)) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT;
    }

    const uint32_t insert = (played + margin_records) % record_count;
    const uint32_t previous = (insert + record_count - 1U) % record_count;
    const uint16_t *last = step_stream_axis_logic_record(buffer, previous);
    /* CCR 0 is a filler record: the axis does not step here. */
    if (last[2] == 0U) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW;
    }
    const uint32_t start = step_stream_brake_find_start(table, last[0]);
    if (start >= table->count) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW;
    }
    const uint8_t insert_half = (uint8_t)(insert / half_records);
    /* The ramp would start in a half that is not FULL, and the DMA reaches it within the margin: its old records are
     * a lap old and nothing can be done in time. */
    if (insert_half == other && other_state == STEP_STREAM_RING_HALF_EMPTY) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW;
    }

    step_stream_axis_logic_brake_reset(brake);
    step_stream_brake_cursor_init(&brake->cursor, table, start);
    const uint32_t first_length = ((uint32_t)insert_half + 1U) * half_records - insert;
    brake->half_has_steps[insert_half] =
        (uint8_t)(step_stream_brake_fill(&brake->cursor, step_stream_axis_logic_record(buffer, insert), first_length) !=
                  0U);
    if (insert_half == playing) {
        if (other_state == STEP_STREAM_RING_HALF_FULL) {
            brake->half_has_steps[other] =
                (uint8_t)(step_stream_brake_fill(
                              &brake->cursor,
                              step_stream_axis_logic_record(buffer, (uint32_t)other * half_records),
                              half_records
                          ) != 0U);
        } else if (held_half == other) {
            /* The task is filling it and will commit it: the commit writes the ramp over what the task put there. */
            brake->deferred_half = other;
            brake->half_has_steps[other] = 1U;
        } else {
            /* Nobody holds it, so no commit will come: fill it here. */
            brake->half_has_steps[other] =
                (uint8_t)(step_stream_brake_fill(
                              &brake->cursor,
                              step_stream_axis_logic_record(buffer, (uint32_t)other * half_records),
                              half_records
                          ) != 0U);
            /* It was EMPTY a moment ago and only this function and the task write it, so this cannot fail. */
            (void)step_stream_ring_mark_filled(ring, other);
        }
    }
    brake->active = 1U;
    return STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED;
}

StepStreamAxisBrakeEvent_t step_stream_axis_logic_brake_half_released(
    StepStreamAxisBrake_t *brake,
    StepStreamRing_t *ring,
    uint16_t *buffer,
    uint32_t record_count,
    uint8_t released_half
) {
    if (brake == NULL || ring == NULL || buffer == NULL || brake->active == 0U ||
        released_half >= STEP_STREAM_RING_HALF_COUNT || record_count < 4U || (record_count & 1U) != 0U) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT;
    }
    const uint32_t half_records = record_count / 2U;
    brake->half_has_steps[released_half] =
        (uint8_t)(step_stream_brake_fill(
                      &brake->cursor,
                      step_stream_axis_logic_record(buffer, (uint32_t)released_half * half_records),
                      half_records
                  ) != 0U);
    if (step_stream_ring_mark_filled(ring, released_half) != STEP_STREAM_RING_STATUS_OK) {
        return STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT;
    }
    const uint8_t playing = (uint8_t)(1U - released_half);
    return brake->half_has_steps[released_half] == 0U && brake->half_has_steps[playing] == 0U
               ? STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_COMPLETE
               : STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_CONTINUE;
}

uint8_t step_stream_axis_logic_brake_commit(
    StepStreamAxisBrake_t *brake,
    StepStreamRing_t *ring,
    uint16_t *buffer,
    uint32_t record_count,
    uint8_t half
) {
    if (brake == NULL || ring == NULL || buffer == NULL || brake->active == 0U || half != brake->deferred_half ||
        half >= STEP_STREAM_RING_HALF_COUNT) {
        return 0U;
    }
    const uint32_t half_records = record_count / 2U;
    brake->half_has_steps[half] = (uint8_t)(step_stream_brake_fill(
                                                &brake->cursor,
                                                step_stream_axis_logic_record(buffer, (uint32_t)half * half_records),
                                                half_records
                                            ) != 0U);
    brake->deferred_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    return (uint8_t)(step_stream_ring_mark_filled(ring, half) == STEP_STREAM_RING_STATUS_OK);
}

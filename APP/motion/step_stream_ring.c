#include "motion/step_stream_ring.h"

#include <stddef.h>

/* The half just played becomes EMPTY; the other one must be FULL before the DMA enters it. */
static StepStreamRingStatus_t step_stream_ring_half_event(StepStreamRing_t *ring, uint8_t played_half) {
    if (ring->state == STEP_STREAM_RING_STATE_NOT_STARTED) {
        return STEP_STREAM_RING_STATUS_INVALID_STATE;
    }
    if (ring->state == STEP_STREAM_RING_STATE_UNDERRUN) {
        return STEP_STREAM_RING_STATUS_UNDERRUN;
    }
    if (ring->state == STEP_STREAM_RING_STATE_SEQUENCE_ERROR || ring->current_half != played_half) {
        ring->state = STEP_STREAM_RING_STATE_SEQUENCE_ERROR;
        return STEP_STREAM_RING_STATUS_SEQUENCE_ERROR;
    }

    const uint8_t next_half = played_half == 0U ? 1U : 0U;
    ring->half_states[played_half] = STEP_STREAM_RING_HALF_EMPTY;
    ++ring->half_transfers;
    if (ring->half_states[next_half] != STEP_STREAM_RING_HALF_FULL) {
        ++ring->underruns;
        ring->state = STEP_STREAM_RING_STATE_UNDERRUN;
        return STEP_STREAM_RING_STATUS_UNDERRUN;
    }

    ring->half_states[next_half] = STEP_STREAM_RING_HALF_PLAYING;
    ring->current_half = next_half;
    return STEP_STREAM_RING_STATUS_OK;
}

void step_stream_ring_init(StepStreamRing_t *ring) {
    ring->state = STEP_STREAM_RING_STATE_NOT_STARTED;
    ring->half_states[0] = STEP_STREAM_RING_HALF_EMPTY;
    ring->half_states[1] = STEP_STREAM_RING_HALF_EMPTY;
    ring->current_half = 0U;
    ring->underruns = 0U;
    ring->half_transfers = 0U;
    ring->half_fills = 0U;
}

StepStreamRingStatus_t step_stream_ring_mark_filled(StepStreamRing_t *ring, uint8_t half) {
    if (ring == NULL || half >= STEP_STREAM_RING_HALF_COUNT) {
        return STEP_STREAM_RING_STATUS_INVALID_ARGUMENT;
    }
    if (ring->half_states[half] != STEP_STREAM_RING_HALF_EMPTY) {
        return STEP_STREAM_RING_STATUS_INVALID_STATE;
    }

    ring->half_states[half] = STEP_STREAM_RING_HALF_FULL;
    ++ring->half_fills;
    return STEP_STREAM_RING_STATUS_OK;
}

StepStreamRingStatus_t step_stream_ring_start(StepStreamRing_t *ring) {
    if (ring == NULL) {
        return STEP_STREAM_RING_STATUS_INVALID_ARGUMENT;
    }
    if (ring->state != STEP_STREAM_RING_STATE_NOT_STARTED) {
        return STEP_STREAM_RING_STATUS_INVALID_STATE;
    }
    if (ring->half_states[0] != STEP_STREAM_RING_HALF_FULL || ring->half_states[1] != STEP_STREAM_RING_HALF_FULL) {
        return STEP_STREAM_RING_STATUS_NOT_READY;
    }

    ring->half_states[0] = STEP_STREAM_RING_HALF_PLAYING;
    ring->current_half = 0U;
    ring->state = STEP_STREAM_RING_STATE_RUNNING;
    return STEP_STREAM_RING_STATUS_OK;
}

StepStreamRingStatus_t step_stream_ring_on_half_transfer(StepStreamRing_t *ring) {
    return ring == NULL ? STEP_STREAM_RING_STATUS_INVALID_ARGUMENT : step_stream_ring_half_event(ring, 0U);
}

StepStreamRingStatus_t step_stream_ring_on_transfer_complete(StepStreamRing_t *ring) {
    return ring == NULL ? STEP_STREAM_RING_STATUS_INVALID_ARGUMENT : step_stream_ring_half_event(ring, 1U);
}

StepStreamRingStatus_t step_stream_ring_next_fill_request(const StepStreamRing_t *ring, uint8_t *half) {
    if (ring == NULL || half == NULL) {
        return STEP_STREAM_RING_STATUS_INVALID_ARGUMENT;
    }
    for (uint8_t candidate = 0U; candidate < STEP_STREAM_RING_HALF_COUNT; ++candidate) {
        if (ring->half_states[candidate] == STEP_STREAM_RING_HALF_EMPTY) {
            *half = candidate;
            return STEP_STREAM_RING_STATUS_OK;
        }
    }
    return STEP_STREAM_RING_STATUS_NO_FILL_REQUEST;
}

StepStreamRingState_t step_stream_ring_get_state(const StepStreamRing_t *ring) { return ring->state; }

uint32_t step_stream_ring_get_underrun_count(const StepStreamRing_t *ring) { return ring->underruns; }

uint32_t step_stream_ring_get_half_transfer_count(const StepStreamRing_t *ring) { return ring->half_transfers; }

uint32_t step_stream_ring_get_half_fill_count(const StepStreamRing_t *ring) { return ring->half_fills; }

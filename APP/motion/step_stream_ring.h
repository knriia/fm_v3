#ifndef FM_V3_STEP_STREAM_RING_H
#define FM_V3_STEP_STREAM_RING_H

#include <stdint.h>

#define STEP_STREAM_RING_HALF_COUNT 2U

typedef enum {
    STEP_STREAM_RING_STATUS_OK = 0,
    STEP_STREAM_RING_STATUS_INVALID_ARGUMENT,
    STEP_STREAM_RING_STATUS_INVALID_STATE,
    STEP_STREAM_RING_STATUS_NOT_READY,
    STEP_STREAM_RING_STATUS_NO_FILL_REQUEST,
    STEP_STREAM_RING_STATUS_UNDERRUN,
    STEP_STREAM_RING_STATUS_SEQUENCE_ERROR
} StepStreamRingStatus_t;

typedef enum {
    STEP_STREAM_RING_STATE_NOT_STARTED = 0,
    STEP_STREAM_RING_STATE_RUNNING,
    STEP_STREAM_RING_STATE_UNDERRUN,
    STEP_STREAM_RING_STATE_SEQUENCE_ERROR
} StepStreamRingState_t;

typedef enum {
    STEP_STREAM_RING_HALF_EMPTY = 0,
    STEP_STREAM_RING_HALF_FULL,
    STEP_STREAM_RING_HALF_PLAYING
} StepStreamRingHalfState_t;

/* Owned by the caller. Each transition has one writer: the DMA interrupt moves PLAYING to EMPTY and FULL to
 * PLAYING, the task moves EMPTY to FULL, so no lock is needed. */
typedef struct {
    volatile StepStreamRingState_t state;
    volatile uint8_t half_states[STEP_STREAM_RING_HALF_COUNT];
    /* Half the DMA plays now; the next HT event expects 0, the next TC event expects 1. */
    volatile uint8_t current_half;
    volatile uint32_t underruns;
    volatile uint32_t half_transfers;
    volatile uint32_t half_fills;
} StepStreamRing_t;

/* Also returns a ring to the start; call only while the DMA of this ring is stopped. */
void step_stream_ring_init(StepStreamRing_t *ring);
/* Task side: EMPTY to FULL. */
StepStreamRingStatus_t step_stream_ring_mark_filled(StepStreamRing_t *ring, uint8_t half);
/* Task side: starts the cycle with half 0 playing; both halves must be FULL. */
StepStreamRingStatus_t step_stream_ring_start(StepStreamRing_t *ring);
/* ISR side, DMA half-transfer (HT) and transfer-complete (TC) events, alternating. A half that is not FULL when
 * the DMA enters it is an UNDERRUN: the caller must stop every axis. Both faults are sticky until init. */
StepStreamRingStatus_t step_stream_ring_on_half_transfer(StepStreamRing_t *ring);
StepStreamRingStatus_t step_stream_ring_on_transfer_complete(StepStreamRing_t *ring);
/* Task side: the first half that needs filling, or NO_FILL_REQUEST. */
StepStreamRingStatus_t step_stream_ring_next_fill_request(const StepStreamRing_t *ring, uint8_t *half);
StepStreamRingState_t step_stream_ring_get_state(const StepStreamRing_t *ring);
uint32_t step_stream_ring_get_underrun_count(const StepStreamRing_t *ring);
uint32_t step_stream_ring_get_half_transfer_count(const StepStreamRing_t *ring);
uint32_t step_stream_ring_get_half_fill_count(const StepStreamRing_t *ring);

#endif /* FM_V3_STEP_STREAM_RING_H */

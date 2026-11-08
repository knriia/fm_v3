#ifndef FM_V3_STEP_STREAM_AXIS_LOGIC_H
#define FM_V3_STEP_STREAM_AXIS_LOGIC_H

#include "motion/step_stream_ring.h"

#include <stdint.h>

/* Register-free decisions of step_stream_axis.c, kept apart so that host tests can cover them. */

typedef enum {
    /* The played half is released (EMPTY, or FULL again in stand mode); wake the filler. */
    STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED = 0,
    STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN,
    STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_SEQUENCE
} StepStreamAxisLogicAction_t;

typedef struct {
    /* Half-buffer events (HT or TC) whose flag is set in the DMA status register now. */
    uint32_t (*pending_half_buffers)(void);
    /* Remaining DMA transfers (NDTR). */
    uint32_t (*ndtr)(void);
} StepStreamAxisSnapshotSource_t;

/* Both halves FULL and the cycle started. Returns 0 on success. */
uint8_t step_stream_axis_logic_prepare_stand_ring(StepStreamRing_t *ring);
/* Runs the HT (half_transfer != 0) or TC event through the ring. With auto_refill the played half goes straight
 * back to FULL: only for a constant pattern, otherwise the task is the only writer of EMPTY to FULL. */
StepStreamAxisLogicAction_t
step_stream_axis_logic_half_event(StepStreamRing_t *ring, uint8_t half_transfer, uint8_t auto_refill);
/* Steps played, from a snapshot that did not change while it was taken (the DMA reloads NDTR at TC). */
uint32_t step_stream_axis_logic_read_steps(
    const StepStreamAxisSnapshotSource_t *source,
    uint32_t completed_half_buffers,
    uint32_t record_count
);
/* Steps valid at the moment a half-buffer event arrived: the DMA position is at the half boundary. */
uint32_t step_stream_axis_logic_boundary_steps(uint32_t completed_half_buffers, uint32_t record_count);
/* Steps played from stale records after a fault: raw minus valid, never negative. */
uint32_t step_stream_axis_logic_stale_steps(uint32_t raw_steps, uint32_t valid_steps);
/* Polls until is_done() returns non-zero; 1 if it did within max_polls, 0 on timeout. */
uint8_t step_stream_axis_logic_poll(uint8_t (*is_done)(void), uint32_t max_polls);

#endif /* FM_V3_STEP_STREAM_AXIS_LOGIC_H */

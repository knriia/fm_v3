#ifndef FM_V3_STEP_STREAM_AXIS_LOGIC_H
#define FM_V3_STEP_STREAM_AXIS_LOGIC_H

#include "motion/step_stream_brake.h"
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
    uint32_t (*pending_half_buffers)(const void *context);
    /* Remaining DMA transfers (NDTR). */
    uint32_t (*ndtr)(const void *context);
    /* Handed to both functions: the axis whose DMA stream is read. */
    const void *context;
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
/* Polls until is_done(context) returns non-zero; 1 if it did within max_polls, 0 on timeout. */
uint8_t step_stream_axis_logic_poll(uint8_t (*is_done)(const void *context), const void *context, uint32_t max_polls);

/* Braking on a limit switch. The interrupt replaces the records that the DMA has not played yet with the braking
 * ramp, starting a few records ahead of its position, and from then on refills every released half from the ramp
 * (filler records after its end). The axis stops when a released half and the half now playing carry no step.
 * While braking, the interrupt is the only writer of EMPTY to FULL: the task neither acquires nor commits a half,
 * except a half it already held when the braking began (the deferred half), which is filled at its commit. */
#define STEP_STREAM_AXIS_LOGIC_NO_HALF 0xFFU

typedef struct {
    StepStreamBrakeCursor_t cursor;
    uint8_t active;
    /* A half that holds a step of the ramp, or that the DMA has not yet drained of old ones. */
    uint8_t half_has_steps[STEP_STREAM_RING_HALF_COUNT];
    /* The EMPTY half that the task may have been writing when the braking began; NO_HALF if none. */
    uint8_t deferred_half;
} StepStreamAxisBrake_t;

typedef enum {
    /* The ramp is in the ring; the axis keeps stepping while it brakes. */
    STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED = 0,
    /* No ramp can be placed: the axis does not step at this point, is slower than the ramp, or the half that the ramp
     * would start in is not ready. The caller stops the axis at once. */
    STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW,
    /* A half event of the DMA has not been handled yet: handle it and try again. */
    STEP_STREAM_AXIS_LOGIC_BRAKE_RETRY,
    /* The ring or the arguments are inconsistent: the caller stops the axis with a fault. */
    STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT
} StepStreamAxisBrakeBegin_t;

typedef enum {
    STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_CONTINUE = 0,
    /* No step is left in the ring: the caller stops the axis. */
    STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_COMPLETE,
    STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT
} StepStreamAxisBrakeEvent_t;

void step_stream_axis_logic_brake_reset(StepStreamAxisBrake_t *brake);
/* steps_done is the count the axis reports (records the DMA has taken minus the one waiting in the preload);
 * the ramp starts margin_records after the DMA position. held_half is the EMPTY half that the task has acquired and
 * not yet committed, or NO_HALF: only such a half is left to its commit, any other EMPTY half is filled here.
 * Call it only after the pending half events were handled. */
StepStreamAxisBrakeBegin_t step_stream_axis_logic_brake_begin(
    StepStreamAxisBrake_t *brake,
    const StepStreamBrake_t *table,
    StepStreamRing_t *ring,
    uint16_t *buffer,
    uint32_t record_count,
    uint32_t steps_done,
    uint32_t margin_records,
    uint8_t held_half
);
/* The DMA has played the half and the ring has released it (EMPTY): fills it from the ramp and marks it FULL. */
StepStreamAxisBrakeEvent_t step_stream_axis_logic_brake_half_released(
    StepStreamAxisBrake_t *brake,
    StepStreamRing_t *ring,
    uint16_t *buffer,
    uint32_t record_count,
    uint8_t released_half
);
/* The task commits a half while braking: 1 if it was the deferred half and is now filled from the ramp and FULL,
 * 0 if the commit is to be ignored. */
uint8_t step_stream_axis_logic_brake_commit(
    StepStreamAxisBrake_t *brake,
    StepStreamRing_t *ring,
    uint16_t *buffer,
    uint32_t record_count,
    uint8_t half
);

#endif /* FM_V3_STEP_STREAM_AXIS_LOGIC_H */

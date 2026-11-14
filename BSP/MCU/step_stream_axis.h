#ifndef FM_V3_STEP_STREAM_AXIS_H
#define FM_V3_STEP_STREAM_AXIS_H

#include "motion/step_stream_brake.h"
#include "motion_axis_hardware.h"

#include <stdint.h>

typedef enum {
    STEP_STREAM_AXIS_STATUS_OK = 0,
    STEP_STREAM_AXIS_STATUS_BUSY,
    STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT,
    STEP_STREAM_AXIS_STATUS_AXIS_BUSY,
    STEP_STREAM_AXIS_STATUS_CLOCK_ERROR,
    /* Stream start: a half is not FULL yet. */
    STEP_STREAM_AXIS_STATUS_NOT_READY,
    /* Both halves are FULL or being played: nothing to fill now. */
    STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST,
    /* Stream mode: the ring was used by an earlier run; call prepare_stream first. */
    STEP_STREAM_AXIS_STATUS_NOT_PREPARED,
    /* The DMA stream did not disable, so it could not be set up; the axis stays IDLE with FAULT_DMA. */
    STEP_STREAM_AXIS_STATUS_DMA_ERROR
} StepStreamAxisStatus_t;

/* Records in the whole ring and in one half. A record is STEP_STREAM_FORMAT_DMA_BURST_LENGTH 16-bit words. */
#define STEP_STREAM_AXIS_RECORD_COUNT 2730U
#define STEP_STREAM_AXIS_HALF_RECORD_COUNT (STEP_STREAM_AXIS_RECORD_COUNT / 2U)

typedef enum { STEP_STREAM_AXIS_STATE_IDLE = 0, STEP_STREAM_AXIS_STATE_STREAMING } StepStreamAxisState_t;

typedef enum {
    STEP_STREAM_AXIS_FAULT_NONE = 0,
    /* The DMA entered a half that the task had not filled in time. */
    STEP_STREAM_AXIS_FAULT_UNDERRUN,
    /* HT/TC events came in the wrong order (the interrupt was late by a whole half or more). */
    STEP_STREAM_AXIS_FAULT_SEQUENCE,
    /* DMA transfer error, or the stream did not disable within the polling bound. */
    STEP_STREAM_AXIS_FAULT_DMA
} StepStreamAxisFault_t;

/* One stream channel: the timer, the DMA stream and the ring of one axis. The state lives in the instance; the
 * functions below take it as their first argument and never touch another axis. */
typedef struct StepStreamAxis StepStreamAxis_t;

/* The channel of the axis, or NULL when the axis has no stream channel (yet). The instances exist for the whole run. */
StepStreamAxis_t *step_stream_axis_get(MotionAxis_t axis);

/* All callbacks run in the DMA interrupt and must not block; each gets `context` back. */
typedef struct {
    /* A half has been played and is EMPTY: wake the task that fills it. */
    void (*on_half_released)(void *context);
    /* The axis has already stopped itself; the owner must stop the other axes and report the fault. */
    void (*on_fault)(void *context, StepStreamAxisFault_t fault);
    /* The braking on a limit switch has begun, or the axis was stopped at once because it could not begin: stop feeding
     * the ring. The axis finishes the stop on its own. */
    void (*on_brake)(void *context);
    void *context;
} StepStreamAxisListener_t;

/* Pass NULL to remove the callbacks. Returns BUSY while the axis is STREAMING. */
StepStreamAxisStatus_t step_stream_axis_set_listener(StepStreamAxis_t *axis, const StepStreamAxisListener_t *listener);
/* Sticky: set by a ring fault, cleared by the next successful start. */
StepStreamAxisFault_t step_stream_axis_get_fault(const StepStreamAxis_t *axis);
/* After an UNDERRUN stop, steps_done counts only the valid ones up to the half boundary; the stale records the DMA
 * played before the interrupt stopped it are reported here. */
uint32_t step_stream_axis_stale_steps(const StepStreamAxis_t *axis);

/* Stand mode: the buffer holds a constant pattern, so the interrupt marks every played half as filled again. */
StepStreamAxisStatus_t step_stream_axis_start(StepStreamAxis_t *axis, uint32_t frequency_hz, uint32_t pulse_width_ns);

/* Stream mode: the task is the only writer of EMPTY to FULL. Order: prepare_stream, fill both halves with
 * acquire_half + commit_half, start_stream; while STREAMING the task keeps refilling released halves.
 * One filler task only: acquire_half is not atomic, a second call before commit_half returns the same half.
 * Stand mode and stream mode share the ring and the buffer and exclude each other: a stand start overwrites
 * records that were prepared for a stream, so never start the stand between prepare_stream and start_stream. */

/* Resets the ring, the fault and the step counters of the last run; only while IDLE (BUSY otherwise). */
StepStreamAxisStatus_t step_stream_axis_prepare_stream(StepStreamAxis_t *axis);
/* The first EMPTY half: its index and STEP_STREAM_AXIS_HALF_RECORD_COUNT records to write. NO_FILL_REQUEST when
 * none, NOT_PREPARED on an IDLE axis whose ring still holds an earlier run. The half stays EMPTY (and the DMA never
 * plays it) until commit_half. */
StepStreamAxisStatus_t step_stream_axis_acquire_half(StepStreamAxis_t *axis, uint8_t *half, uint16_t **records);
/* EMPTY to FULL after the records were written. INVALID_ARGUMENT for a half that is not EMPTY. */
StepStreamAxisStatus_t step_stream_axis_commit_half(StepStreamAxis_t *axis, uint8_t half);
/* NOT_PREPARED without prepare_stream, NOT_READY unless both halves are FULL. The timer idles with no pulse for
 * startup_period_ns until the DMA delivers the first record, and that record takes effect one period later, so the
 * first pulse comes after two startup periods. The period must exceed the DMA burst latency: if the burst is not
 * done before the second update, the next record overwrites the first and a step is lost silently.
 * STEP_STREAM_AXIS_MIN_STARTUP_NS is a placeholder that has not been measured: check it on the scope. */
#define STEP_STREAM_AXIS_MIN_STARTUP_NS 2000U
StepStreamAxisStatus_t step_stream_axis_start_stream(StepStreamAxis_t *axis, uint32_t startup_period_ns);
/* Kernel clock of the axis timer in Hz, 0 when the clock tree is not usable. */
uint32_t step_stream_axis_get_timer_clock_hz(const StepStreamAxis_t *axis);
void step_stream_axis_stop(StepStreamAxis_t *axis);
uint32_t step_stream_axis_steps_done(const StepStreamAxis_t *axis);
StepStreamAxisState_t step_stream_axis_get_state(const StepStreamAxis_t *axis);

/* Braking on a limit switch (stream mode only). configure_brake builds the ramp once, while IDLE; without it a
 * request stops the axis at once. */
StepStreamAxisStatus_t step_stream_axis_configure_brake(StepStreamAxis_t *axis, const StepStreamBrakeConfig_t *config);
/* Callable from any interrupt: the DMA interrupt runs the braking, after the half events it has pending, so the two
 * never interleave. Ignored unless the axis is STREAMING. While braking, the ring is the interrupt's: acquire_half
 * finds nothing to fill and commit_half is ignored (except a half the task held at the start, which gets the ramp).
 * The axis goes IDLE once the ramp has been played. */
void step_stream_axis_request_brake(StepStreamAxis_t *axis);
/* 1 from the start of a braking until the axis is IDLE. */
uint8_t step_stream_axis_is_braking(const StepStreamAxis_t *axis);

typedef struct {
    uint32_t brakes_started;
    uint32_t brakes_completed;
    /* Requests that stopped the axis at once: it did not step there, was slower than the ramp, or the ring was not
     * ready for the ramp. */
    uint32_t stopped_at_once;
    uint32_t retries;
} StepStreamAxisBrakeStats_t;

void step_stream_axis_get_brake_stats(const StepStreamAxis_t *axis, StepStreamAxisBrakeStats_t *stats);

#endif /* FM_V3_STEP_STREAM_AXIS_H */

#ifndef FM_V3_STEP_STREAM_AXIS_H
#define FM_V3_STEP_STREAM_AXIS_H

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

/* Both callbacks run in the DMA interrupt and must not block. */
typedef struct {
    /* A half has been played and is EMPTY: wake the task that fills it. */
    void (*on_half_released)(void);
    /* The axis has already stopped itself; the owner must stop the other axes and report the fault. */
    void (*on_fault)(StepStreamAxisFault_t fault);
} StepStreamAxisListener_t;

/* Pass NULL to remove the callbacks. Returns BUSY while the axis is STREAMING. */
StepStreamAxisStatus_t step_stream_axis_set_listener(const StepStreamAxisListener_t *listener);
/* Sticky: set by a ring fault, cleared by the next successful start. */
StepStreamAxisFault_t step_stream_axis_get_fault(void);
/* After an UNDERRUN stop, steps_done counts only the valid ones up to the half boundary; the stale records the DMA
 * played before the interrupt stopped it are reported here. */
uint32_t step_stream_axis_stale_steps(void);

/* Stand mode: the buffer holds a constant pattern, so the interrupt marks every played half as filled again. */
StepStreamAxisStatus_t step_stream_axis_start(uint32_t frequency_hz, uint32_t pulse_width_ns);

/* Stream mode: the task is the only writer of EMPTY to FULL. Order: prepare_stream, fill both halves with
 * acquire_half + commit_half, start_stream; while STREAMING the task keeps refilling released halves.
 * One filler task only: acquire_half is not atomic, a second call before commit_half returns the same half.
 * Stand mode and stream mode share the ring and the buffer and exclude each other: a stand start overwrites
 * records that were prepared for a stream, so never start the stand between prepare_stream and start_stream. */

/* Resets the ring, the fault and the step counters of the last run; only while IDLE (BUSY otherwise). */
StepStreamAxisStatus_t step_stream_axis_prepare_stream(void);
/* The first EMPTY half: its index and STEP_STREAM_AXIS_HALF_RECORD_COUNT records to write. NO_FILL_REQUEST when
 * none, NOT_PREPARED on an IDLE axis whose ring still holds an earlier run. The half stays EMPTY (and the DMA never
 * plays it) until commit_half. */
StepStreamAxisStatus_t step_stream_axis_acquire_half(uint8_t *half, uint16_t **records);
/* EMPTY to FULL after the records were written. INVALID_ARGUMENT for a half that is not EMPTY. */
StepStreamAxisStatus_t step_stream_axis_commit_half(uint8_t half);
/* NOT_PREPARED without prepare_stream, NOT_READY unless both halves are FULL. The timer idles with no pulse for
 * startup_period_ns until the DMA delivers the first record, and that record takes effect one period later, so the
 * first pulse comes after two startup periods. The period must exceed the DMA burst latency: if the burst is not
 * done before the second update, the next record overwrites the first and a step is lost silently.
 * STEP_STREAM_AXIS_MIN_STARTUP_NS is a placeholder that has not been measured: check it on the scope. */
#define STEP_STREAM_AXIS_MIN_STARTUP_NS 2000U
StepStreamAxisStatus_t step_stream_axis_start_stream(uint32_t startup_period_ns);
void step_stream_axis_stop(void);
uint32_t step_stream_axis_steps_done(void);
StepStreamAxisState_t step_stream_axis_get_state(void);

#endif /* FM_V3_STEP_STREAM_AXIS_H */

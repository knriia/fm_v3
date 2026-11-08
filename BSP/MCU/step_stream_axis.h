#ifndef FM_V3_STEP_STREAM_AXIS_H
#define FM_V3_STEP_STREAM_AXIS_H

#include <stdint.h>

typedef enum {
    STEP_STREAM_AXIS_STATUS_OK = 0,
    STEP_STREAM_AXIS_STATUS_BUSY,
    STEP_STREAM_AXIS_STATUS_INVALID_ARGUMENT,
    STEP_STREAM_AXIS_STATUS_AXIS_BUSY,
    STEP_STREAM_AXIS_STATUS_CLOCK_ERROR
} StepStreamAxisStatus_t;

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
void step_stream_axis_stop(void);
uint32_t step_stream_axis_steps_done(void);
StepStreamAxisState_t step_stream_axis_get_state(void);

#endif /* FM_V3_STEP_STREAM_AXIS_H */

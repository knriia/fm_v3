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

StepStreamAxisStatus_t step_stream_axis_start(uint32_t frequency_hz, uint32_t pulse_width_ns);
void step_stream_axis_stop(void);
uint32_t step_stream_axis_steps_done(void);
StepStreamAxisState_t step_stream_axis_get_state(void);

#endif /* FM_V3_STEP_STREAM_AXIS_H */

#ifndef FM_V3_STEP_STREAM_FORMAT_H
#define FM_V3_STEP_STREAM_FORMAT_H

#include <stddef.h>
#include <stdint.h>

#define STEP_STREAM_FORMAT_DMA_BURST_LENGTH 3U

typedef enum { STEP_STREAM_FORMAT_STATUS_OK = 0, STEP_STREAM_FORMAT_STATUS_INVALID_ARGUMENT } StepStreamFormatStatus_t;

StepStreamFormatStatus_t
step_stream_format_fill(uint16_t *buffer, size_t record_count, uint32_t period_ticks, uint32_t pulse_ticks);

/* completed_half_buffers counts half-buffers already played by the circular DMA. */
uint32_t step_stream_format_steps_done(uint32_t completed_half_buffers, uint32_t record_count, uint32_t ndtr);

#endif /* FM_V3_STEP_STREAM_FORMAT_H */

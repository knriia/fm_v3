#ifndef FM_V3_MOTION_EVENT_SOURCE_H
#define FM_V3_MOTION_EVENT_SOURCE_H

#include "motion/motion_event.h"

#include <stddef.h>

typedef enum {
    MOTION_EVENT_SOURCE_STATUS_OK = 0,
    MOTION_EVENT_SOURCE_STATUS_COMPLETE,
    MOTION_EVENT_SOURCE_STATUS_UNDERRUN,
    MOTION_EVENT_SOURCE_STATUS_NOT_RUNNING,
    MOTION_EVENT_SOURCE_STATUS_INVALID_ARGUMENT
} MotionEventSourceStatus_t;

/* What one axis pulls its planned step events from. The axis driver owns no queue and knows no producer: it calls
 * begin once, then pulse_complete after each physical STEP pulse, both from the step ISR, and reset when a stream
 * ends. All three receive `context`. */
typedef struct {
    void *context;
    MotionEventSourceStatus_t (*begin)(void *context, MotionExecutionEvent_t *first_event);
    MotionEventSourceStatus_t (*pulse_complete)(void *context, MotionExecutionEvent_t *next_event);
    void (*reset)(void *context);
} MotionEventSource_t;

#endif /* FM_V3_MOTION_EVENT_SOURCE_H */

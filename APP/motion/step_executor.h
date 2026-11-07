#ifndef FM_V3_STEP_EXECUTOR_H
#define FM_V3_STEP_EXECUTOR_H

#include "motion/motion_event.h"
#include "motion/motion_event_source.h"

#include <stdint.h>

typedef enum {
    STEP_EXECUTOR_STATE_IDLE = 0,
    STEP_EXECUTOR_STATE_RUNNING,
    STEP_EXECUTOR_STATE_COMPLETE,
    STEP_EXECUTOR_STATE_UNDERRUN
} StepExecutorState_t;

/* One planned step stream of one axis. The owner keeps the instance and registers its source with the axis driver.
 * Until per-axis buffers exist every instance takes events from the shared execution buffer, so only one instance
 * may run at a time. */
typedef struct {
    StepExecutorState_t state;
    uint8_t current_is_last;
    /* Written only by the step ISR; a 32-bit read from a task is atomic. */
    volatile uint32_t underruns;
} StepExecutor_t;

/* The ISR calls begin once, then pulse_complete after each physical STEP pulse. */
MotionEventSourceStatus_t step_executor_begin(StepExecutor_t *executor, MotionExecutionEvent_t *first_event);
MotionEventSourceStatus_t step_executor_pulse_complete(StepExecutor_t *executor, MotionExecutionEvent_t *next_event);
MotionEventSourceStatus_t step_executor_get_status(const StepExecutor_t *executor);
void step_executor_reset(StepExecutor_t *executor);
/* Times the buffer ran dry in the middle of a motion since start; each one ends that motion with a fault. */
uint32_t step_executor_get_underrun_count(const StepExecutor_t *executor);

/* The source the axis driver pulls from; `executor` must outlive it. */
MotionEventSource_t step_executor_event_source(StepExecutor_t *executor);

#endif /* FM_V3_STEP_EXECUTOR_H */

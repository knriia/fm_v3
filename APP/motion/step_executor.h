#ifndef FM_V3_STEP_EXECUTOR_H
#define FM_V3_STEP_EXECUTOR_H

#include "motion/motion_event.h"

#include <stdint.h>

typedef enum {
    STEP_EXECUTOR_STATUS_OK = 0,
    STEP_EXECUTOR_STATUS_COMPLETE,
    STEP_EXECUTOR_STATUS_UNDERRUN,
    STEP_EXECUTOR_STATUS_NOT_RUNNING,
    STEP_EXECUTOR_STATUS_INVALID_ARGUMENT
} StepExecutorStatus_t;

/* The ISR calls begin once, then pulse_complete after each physical STEP pulse. */
StepExecutorStatus_t step_executor_begin(MotionExecutionEvent_t *first_event);
StepExecutorStatus_t step_executor_pulse_complete(MotionExecutionEvent_t *next_event);
StepExecutorStatus_t step_executor_get_status(void);
void step_executor_reset(void);
/* Times the buffer ran dry in the middle of a motion since start; each one ends that motion with a fault. */
uint32_t step_executor_get_underrun_count(void);

#endif /* FM_V3_STEP_EXECUTOR_H */

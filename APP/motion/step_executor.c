#include "motion/step_executor.h"

#include "motion/execution_buffer.h"
#include "motion/motion_planner.h"

#include <stddef.h>

typedef enum {
    STEP_EXECUTOR_STATE_IDLE = 0,
    STEP_EXECUTOR_STATE_RUNNING,
    STEP_EXECUTOR_STATE_COMPLETE,
    STEP_EXECUTOR_STATE_UNDERRUN
} StepExecutorState_t;

static StepExecutorState_t step_executor_state;
static uint8_t step_executor_current_is_last;
/* Written only by the step ISR; a 32-bit read from a task is atomic. */
static volatile uint32_t step_executor_underruns;

static StepExecutorStatus_t step_executor_take_event(MotionExecutionEvent_t *event) {
    if (execution_buffer_try_pop(event) != EXECUTION_BUFFER_STATUS_OK) {
        step_executor_state = STEP_EXECUTOR_STATE_UNDERRUN;
        return STEP_EXECUTOR_STATUS_UNDERRUN;
    }
    if (event->interval_ns == 0U || event->step_mask == 0U || event->last_in_block > 1U ||
        (event->step_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U ||
        (event->positive_direction_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U) {
        step_executor_state = STEP_EXECUTOR_STATE_IDLE;
        return STEP_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }

    step_executor_current_is_last = event->last_in_block != 0U;
    return STEP_EXECUTOR_STATUS_OK;
}

StepExecutorStatus_t step_executor_begin(MotionExecutionEvent_t *first_event) {
    if (first_event == NULL) {
        return STEP_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    if (step_executor_state == STEP_EXECUTOR_STATE_RUNNING) {
        return STEP_EXECUTOR_STATUS_NOT_RUNNING;
    }

    step_executor_state = STEP_EXECUTOR_STATE_IDLE;
    step_executor_current_is_last = 0U;
    const StepExecutorStatus_t status = step_executor_take_event(first_event);
    if (status != STEP_EXECUTOR_STATUS_OK) {
        return status;
    }

    step_executor_state = STEP_EXECUTOR_STATE_RUNNING;
    return STEP_EXECUTOR_STATUS_OK;
}

StepExecutorStatus_t step_executor_pulse_complete(MotionExecutionEvent_t *next_event) {
    if (next_event == NULL) {
        return STEP_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    if (step_executor_state != STEP_EXECUTOR_STATE_RUNNING) {
        return step_executor_state == STEP_EXECUTOR_STATE_COMPLETE ? STEP_EXECUTOR_STATUS_COMPLETE
                                                                   : STEP_EXECUTOR_STATUS_NOT_RUNNING;
    }
    /* A finished block is the end of motion only when no next block is queued; otherwise motion continues. */
    if (step_executor_current_is_last != 0U && execution_buffer_count() == 0U) {
        step_executor_state = STEP_EXECUTOR_STATE_COMPLETE;
        return STEP_EXECUTOR_STATUS_COMPLETE;
    }

    const StepExecutorStatus_t status = step_executor_take_event(next_event);
    if (status == STEP_EXECUTOR_STATUS_UNDERRUN) {
        /* One per fault, and every fault needs a STOP, so the counter cannot realistically wrap. */
        ++step_executor_underruns;
    }
    return status;
}

StepExecutorStatus_t step_executor_get_status(void) {
    switch (step_executor_state) {
    case STEP_EXECUTOR_STATE_RUNNING:
        return STEP_EXECUTOR_STATUS_OK;
    case STEP_EXECUTOR_STATE_COMPLETE:
        return STEP_EXECUTOR_STATUS_COMPLETE;
    case STEP_EXECUTOR_STATE_UNDERRUN:
        return STEP_EXECUTOR_STATUS_UNDERRUN;
    default:
        return STEP_EXECUTOR_STATUS_NOT_RUNNING;
    }
}

uint32_t step_executor_get_underrun_count(void) { return step_executor_underruns; }

void step_executor_reset(void) {
    step_executor_state = STEP_EXECUTOR_STATE_IDLE;
    step_executor_current_is_last = 0U;
}

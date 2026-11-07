#include "motion/step_executor.h"

#include "motion/execution_buffer.h"
#include "motion/motion_planner.h"

#include <stddef.h>

static MotionEventSourceStatus_t step_executor_take_event(StepExecutor_t *executor, MotionExecutionEvent_t *event) {
    if (execution_buffer_try_pop(event) != EXECUTION_BUFFER_STATUS_OK) {
        executor->state = STEP_EXECUTOR_STATE_UNDERRUN;
        return MOTION_EVENT_SOURCE_STATUS_UNDERRUN;
    }
    if (event->interval_ns == 0U || event->step_mask == 0U || event->last_in_block > 1U ||
        (event->step_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U ||
        (event->positive_direction_mask & (uint8_t)~MOTION_PLANNER_ALL_AXES_MASK) != 0U) {
        executor->state = STEP_EXECUTOR_STATE_IDLE;
        return MOTION_EVENT_SOURCE_STATUS_INVALID_ARGUMENT;
    }

    executor->current_is_last = event->last_in_block != 0U;
    return MOTION_EVENT_SOURCE_STATUS_OK;
}

MotionEventSourceStatus_t step_executor_begin(StepExecutor_t *executor, MotionExecutionEvent_t *first_event) {
    if (first_event == NULL) {
        return MOTION_EVENT_SOURCE_STATUS_INVALID_ARGUMENT;
    }
    if (executor->state == STEP_EXECUTOR_STATE_RUNNING) {
        return MOTION_EVENT_SOURCE_STATUS_NOT_RUNNING;
    }

    executor->state = STEP_EXECUTOR_STATE_IDLE;
    executor->current_is_last = 0U;
    const MotionEventSourceStatus_t status = step_executor_take_event(executor, first_event);
    if (status != MOTION_EVENT_SOURCE_STATUS_OK) {
        return status;
    }

    executor->state = STEP_EXECUTOR_STATE_RUNNING;
    return MOTION_EVENT_SOURCE_STATUS_OK;
}

MotionEventSourceStatus_t step_executor_pulse_complete(StepExecutor_t *executor, MotionExecutionEvent_t *next_event) {
    if (next_event == NULL) {
        return MOTION_EVENT_SOURCE_STATUS_INVALID_ARGUMENT;
    }
    if (executor->state != STEP_EXECUTOR_STATE_RUNNING) {
        return executor->state == STEP_EXECUTOR_STATE_COMPLETE ? MOTION_EVENT_SOURCE_STATUS_COMPLETE
                                                               : MOTION_EVENT_SOURCE_STATUS_NOT_RUNNING;
    }
    /* A finished block is the end of motion only when no next block is queued; otherwise motion continues. */
    if (executor->current_is_last != 0U && execution_buffer_count() == 0U) {
        executor->state = STEP_EXECUTOR_STATE_COMPLETE;
        return MOTION_EVENT_SOURCE_STATUS_COMPLETE;
    }

    const MotionEventSourceStatus_t status = step_executor_take_event(executor, next_event);
    if (status == MOTION_EVENT_SOURCE_STATUS_UNDERRUN) {
        /* One per fault, and every fault needs a STOP, so the counter cannot realistically wrap. */
        ++executor->underruns;
    }
    return status;
}

MotionEventSourceStatus_t step_executor_get_status(const StepExecutor_t *executor) {
    switch (executor->state) {
    case STEP_EXECUTOR_STATE_RUNNING:
        return MOTION_EVENT_SOURCE_STATUS_OK;
    case STEP_EXECUTOR_STATE_COMPLETE:
        return MOTION_EVENT_SOURCE_STATUS_COMPLETE;
    case STEP_EXECUTOR_STATE_UNDERRUN:
        return MOTION_EVENT_SOURCE_STATUS_UNDERRUN;
    default:
        return MOTION_EVENT_SOURCE_STATUS_NOT_RUNNING;
    }
}

uint32_t step_executor_get_underrun_count(const StepExecutor_t *executor) { return executor->underruns; }

void step_executor_reset(StepExecutor_t *executor) {
    executor->state = STEP_EXECUTOR_STATE_IDLE;
    executor->current_is_last = 0U;
}

static MotionEventSourceStatus_t step_executor_source_begin(void *context, MotionExecutionEvent_t *first_event) {
    return step_executor_begin(context, first_event);
}

static MotionEventSourceStatus_t
step_executor_source_pulse_complete(void *context, MotionExecutionEvent_t *next_event) {
    return step_executor_pulse_complete(context, next_event);
}

static void step_executor_source_reset(void *context) { step_executor_reset(context); }

MotionEventSource_t step_executor_event_source(StepExecutor_t *executor) {
    return (MotionEventSource_t){
        .context = executor,
        .begin = step_executor_source_begin,
        .pulse_complete = step_executor_source_pulse_complete,
        .reset = step_executor_source_reset,
    };
}

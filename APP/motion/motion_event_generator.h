#ifndef FM_V3_MOTION_EVENT_GENERATOR_H
#define FM_V3_MOTION_EVENT_GENERATOR_H

#include "motion_planner.h"

#include <stdint.h>

#define MOTION_EVENT_GENERATOR_NANOSECONDS_PER_SECOND 1000000000ULL

typedef struct {
    uint64_t interval_ns;
    uint32_t sequence;
    uint8_t step_mask;
    uint8_t positive_direction_mask;
} MotionEvent_t;

typedef enum {
    MOTION_EVENT_GENERATOR_STATUS_OK = 0,
    MOTION_EVENT_GENERATOR_STATUS_COMPLETE,
    MOTION_EVENT_GENERATOR_STATUS_INVALID_ARGUMENT,
    MOTION_EVENT_GENERATOR_STATUS_INVALID_BLOCK,
    MOTION_EVENT_GENERATOR_STATUS_NOT_INITIALIZED
} MotionEventGeneratorStatus_t;

/* Caller-owned iterator state; no allocation, RTOS dependency, or hardware access. */
typedef struct {
    uint64_t absolute_step_delta[MOTION_PLANNER_AXIS_COUNT];
    uint64_t step_accumulator[MOTION_PLANNER_AXIS_COUNT];
    uint64_t master_step_count;
    uint64_t events_remaining;
    uint64_t interval_floor_ns;
    uint64_t interval_remainder;
    uint64_t interval_fraction_accumulator;
    uint64_t interval_fraction_denominator;
    uint32_t sequence;
    uint8_t positive_direction_mask;
    uint8_t initialized;
} MotionEventGenerator_t;

/* Produces one event at a time; interval_ns is the delay before that event. */
MotionEventGeneratorStatus_t
motion_event_generator_init(MotionEventGenerator_t *generator, const MotionPlanBlock_t *block);
MotionEventGeneratorStatus_t motion_event_generator_next(MotionEventGenerator_t *generator, MotionEvent_t *event);

#endif /* FM_V3_MOTION_EVENT_GENERATOR_H */

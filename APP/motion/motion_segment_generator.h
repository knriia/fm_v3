#ifndef FM_V3_MOTION_SEGMENT_GENERATOR_H
#define FM_V3_MOTION_SEGMENT_GENERATOR_H

#include "motion/motion_event.h"
#include "motion/motion_planner.h"

#include <stdint.h>

typedef enum {
    MOTION_SEGMENT_GENERATOR_STATUS_OK = 0,
    MOTION_SEGMENT_GENERATOR_STATUS_DONE,
    MOTION_SEGMENT_GENERATOR_STATUS_INVALID_ARGUMENT,
    MOTION_SEGMENT_GENERATOR_STATUS_INVALID_BLOCK,
    MOTION_SEGMENT_GENERATOR_STATUS_ARITHMETIC_OVERFLOW
} MotionSegmentGeneratorStatus_t;

typedef struct {
    MotionPlanBlock_t block;
    uint64_t axis_steps[MOTION_PLANNER_AXIS_COUNT];
    uint64_t axis_error[MOTION_PLANNER_AXIS_COUNT];
    uint64_t events_emitted;
    uint64_t interval_base_ns;
    uint64_t interval_remainder_ns;
    uint64_t interval_remainder_accumulator;
    uint8_t initialized;
} MotionSegmentGenerator_t;

MotionSegmentGeneratorStatus_t
motion_segment_generator_init(MotionSegmentGenerator_t *generator, const MotionPlanBlock_t *block);

MotionSegmentGeneratorStatus_t
motion_segment_generator_next(MotionSegmentGenerator_t *generator, MotionExecutionEvent_t *event);

#endif /* FM_V3_MOTION_SEGMENT_GENERATOR_H */

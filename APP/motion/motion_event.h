#ifndef FM_V3_MOTION_EVENT_H
#define FM_V3_MOTION_EVENT_H

#include <stdint.h>

typedef struct {
    uint64_t interval_ns;
    uint32_t sequence;
    uint8_t step_mask;
    uint8_t positive_direction_mask;
    uint8_t last_in_block;
} MotionExecutionEvent_t;

#endif /* FM_V3_MOTION_EVENT_H */

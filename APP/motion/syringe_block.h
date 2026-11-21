#ifndef FM_V3_SYRINGE_BLOCK_H
#define FM_V3_SYRINGE_BLOCK_H

#include "motion/motion_planner.h"

#include <stdint.h>

typedef enum {
    SYRINGE_BLOCK_STATUS_OK = 0,
    /* A zero step count: there is nothing to feed or retract. */
    SYRINGE_BLOCK_STATUS_NO_MOVEMENT,
    /* A NULL block, or a rate of zero. */
    SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT,
    /* The rate is above the limit of the step stream. */
    SYRINGE_BLOCK_STATUS_RATE_LIMIT_EXCEEDED
} SyringeBlockStatus_t;

/* The plan block of a syringe move without axes: the syringe channel alone, `step_delta` steps (positive feeds,
 * negative retracts) at a constant `step_rate_hz`. The syringe has no position, path or limits, so the block carries
 * none of them and the rate is given as steps per second directly (numerator the rate, denominator 1), like the blocks
 * of homing (home_block_make). `max_step_rate_hz` 0 means no limit is configured and every rate is refused. The block
 * is written only when the status is OK. */
SyringeBlockStatus_t syringe_block_make(
    uint32_t sequence,
    int32_t step_delta,
    uint32_t step_rate_hz,
    uint32_t max_step_rate_hz,
    MotionPlanBlock_t *block
);

#endif /* FM_V3_SYRINGE_BLOCK_H */

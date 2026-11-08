#ifndef FM_V3_HOME_BLOCK_H
#define FM_V3_HOME_BLOCK_H

#include "motion/axis_bench.h"
#include "motion/motion_planner.h"

#include <stdint.h>

/* Builds the plan block of one homing move of axis X: constant step rate, no ramp, `steps` steps toward the start or
 * the end switch. The switches, not the step count, end the move. Returns 0 and leaves `block` untouched when the
 * config is missing, or steps or rate is 0. */
uint8_t home_block_make(
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t step_rate_hz,
    MotionPlanBlock_t *block
);

#endif /* FM_V3_HOME_BLOCK_H */

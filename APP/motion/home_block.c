#include "motion/home_block.h"

#include <stddef.h>

uint8_t home_block_make(
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t step_rate_hz,
    MotionPlanBlock_t *block
) {
    if (hardware_config == NULL || block == NULL || steps == 0U || step_rate_hz == 0U) {
        return 0U;
    }
    const uint8_t toward_end = direction == AXIS_BENCH_TOWARD_END ? 1U : 0U;
    const uint8_t positive = toward_end == (hardware_config->positive_moves_toward_end != 0U) ? 1U : 0U;

    *block = (MotionPlanBlock_t){
        .step_delta = {positive != 0U ? (int64_t)steps : -(int64_t)steps},
        .master_step_count = steps,
        .step_event_rate_numerator = step_rate_hz,
        .step_event_rate_denominator = 1U,
        .moving_axes_mask = MOTION_PLANNER_AXIS_X_MASK,
        .positive_direction_mask = positive != 0U ? MOTION_PLANNER_AXIS_X_MASK : 0U,
    };
    return 1U;
}

#include "motion/home_block.h"

#include <stddef.h>

uint8_t home_block_make(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t step_rate_hz,
    MotionPlanBlock_t *block
) {
    if ((uint32_t)axis >= (uint32_t)MOTION_AXIS_HARDWARE_COUNT || hardware_config == NULL || block == NULL ||
        steps == 0U || step_rate_hz == 0U) {
        return 0U;
    }
    const uint8_t toward_end = direction == AXIS_BENCH_TOWARD_END ? 1U : 0U;
    const uint8_t positive = toward_end == (hardware_config->positive_moves_toward_end != 0U) ? 1U : 0U;
    const uint8_t axis_mask = (uint8_t)(1U << (uint32_t)axis);

    *block = (MotionPlanBlock_t){
        .master_step_count = steps,
        .step_event_rate_numerator = step_rate_hz,
        .step_event_rate_denominator = 1U,
        .moving_axes_mask = axis_mask,
        .positive_direction_mask = positive != 0U ? axis_mask : 0U,
    };
    block->step_delta[axis] = positive != 0U ? (int64_t)steps : -(int64_t)steps;
    return 1U;
}

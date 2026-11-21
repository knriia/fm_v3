#include "motion/syringe_block.h"

#include <stddef.h>

SyringeBlockStatus_t syringe_block_make(
    uint32_t sequence,
    int32_t step_delta,
    uint32_t step_rate_hz,
    uint32_t max_step_rate_hz,
    MotionPlanBlock_t *block
) {
    if (block == NULL || step_rate_hz == 0U) {
        return SYRINGE_BLOCK_STATUS_INVALID_ARGUMENT;
    }
    if (step_delta == 0) {
        return SYRINGE_BLOCK_STATUS_NO_MOVEMENT;
    }
    if (step_rate_hz > max_step_rate_hz) {
        return SYRINGE_BLOCK_STATUS_RATE_LIMIT_EXCEEDED;
    }
    const int64_t signed_steps = (int64_t)step_delta;
    *block = (MotionPlanBlock_t){
        .sequence = sequence,
        .master_step_count = (uint64_t)(signed_steps < 0 ? -signed_steps : signed_steps),
        .step_event_rate_numerator = step_rate_hz,
        .step_event_rate_denominator = 1U,
        .moving_axes_mask = MOTION_PLAN_CHANNEL_SYRINGE_MASK,
        .positive_direction_mask = step_delta > 0 ? MOTION_PLAN_CHANNEL_SYRINGE_MASK : 0U,
    };
    block->step_delta[MOTION_PLAN_CHANNEL_SYRINGE] = signed_steps;
    return SYRINGE_BLOCK_STATUS_OK;
}

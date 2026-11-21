#ifndef FM_V3_MOTION_PLANNER_H
#define FM_V3_MOTION_PLANNER_H

#include <stdint.h>

#define MOTION_PLANNER_AXIS_COUNT 3U
#define MOTION_PLANNER_AXIS_X_MASK 0x01U
#define MOTION_PLANNER_AXIS_Y_MASK 0x02U
#define MOTION_PLANNER_AXIS_Z_MASK 0x04U
#define MOTION_PLANNER_ALL_AXES_MASK 0x07U

/* The channels of a plan block: the planner axes first, then the syringe. The syringe is no axis (it has no position,
 * limits or homing): its steps follow from the path of the block, so it takes the next channel index and bit. */
#define MOTION_PLAN_CHANNEL_COUNT 4U
#define MOTION_PLAN_CHANNEL_SYRINGE 3U
#define MOTION_PLAN_CHANNEL_SYRINGE_MASK 0x08U

_Static_assert(MOTION_PLAN_CHANNEL_SYRINGE == MOTION_PLANNER_AXIS_COUNT, "the syringe channel follows the axes");
_Static_assert(MOTION_PLAN_CHANNEL_COUNT == MOTION_PLAN_CHANNEL_SYRINGE + 1U, "the syringe is the last channel");
_Static_assert(
    MOTION_PLAN_CHANNEL_SYRINGE_MASK == (1U << MOTION_PLAN_CHANNEL_SYRINGE),
    "the syringe mask is the bit of its channel"
);

typedef struct {
    int32_t min_position_um;
    int32_t max_position_um;
    /* Motor steps per micrometre as an exact ratio: steps = um * numerator / denominator. */
    uint32_t steps_per_um_numerator;
    uint32_t steps_per_um_denominator;
    uint32_t max_speed_um_per_s;
} MotionPlannerAxisConfig_t;

typedef struct {
    MotionPlannerAxisConfig_t axis[MOTION_PLANNER_AXIS_COUNT];
    uint32_t max_step_event_rate_hz;
    /* Syringe steps per micrometre of path as an exact ratio: steps = path_um * numerator / denominator. A zero
     * numerator means no syringe: blocks then carry no syringe steps. */
    uint32_t syringe_steps_per_um_numerator;
    uint32_t syringe_steps_per_um_denominator;
} MotionPlannerConfig_t;

typedef struct {
    int32_t position_um[MOTION_PLANNER_AXIS_COUNT];
    uint8_t homed_axes_mask;
    /* Fraction of a syringe step (in units of 1/syringe_steps_per_um_denominator step) left over by the blocks planned
     * so far; the next block starts from it, so the rounding does not pile up over many short blocks. It stays below
     * the denominator as long as the syringe configuration does not change; a larger value (a stale one) only adds a
     * whole step to the next block. */
    uint32_t syringe_step_remainder;
} MotionPlannerMachineState_t;

typedef struct {
    uint32_t sequence;
    int32_t target_um[MOTION_PLANNER_AXIS_COUNT];
    uint32_t feedrate_um_per_s;
} MotionPlannerRequest_t;

/* A constant-feedrate linear move block; timing is represented as an exact ratio. */
typedef struct {
    uint32_t sequence;
    /* Number of the block in the order in which the queue took it; the executor of the stream reports progress by it.
     * The planner leaves it 0, the queue owner assigns it. */
    uint32_t order;
    /* Per channel (MOTION_PLAN_CHANNEL_*): the axes, then the syringe. The masks below use the same bits. */
    int64_t step_delta[MOTION_PLAN_CHANNEL_COUNT];
    uint64_t path_length_um;
    uint64_t master_step_count;
    uint64_t step_event_rate_numerator;
    uint64_t step_event_rate_denominator;
    uint32_t feedrate_um_per_s;
    uint8_t moving_axes_mask;
    uint8_t positive_direction_mask;
} MotionPlanBlock_t;

typedef enum {
    MOTION_PLANNER_STATUS_OK = 0,
    MOTION_PLANNER_STATUS_NO_MOVEMENT,
    MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
    MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
    MOTION_PLANNER_STATUS_NOT_HOMED,
    MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
    MOTION_PLANNER_STATUS_INVALID_FEEDRATE,
    MOTION_PLANNER_STATUS_AXIS_SPEED_LIMIT_EXCEEDED,
    MOTION_PLANNER_STATUS_STEP_RATE_LIMIT_EXCEEDED,
    MOTION_PLANNER_STATUS_ARITHMETIC_OVERFLOW
} MotionPlannerStatus_t;

typedef struct {
    MotionPlannerStatus_t status;
    MotionPlanBlock_t block;
    /* The remainder that the machine state takes over when the block is accepted (see
     * MotionPlannerMachineState_t.syringe_step_remainder). Meaningful only when status is OK. */
    uint32_t syringe_step_remainder;
} MotionPlannerResult_t;

/* Pure planner: it validates an absolute XYZ move and returns one plan block. Only axes whose target differs from
 * their current position must be configured and homed, so a machine can run with some axes not fitted yet. */
MotionPlannerResult_t motion_planner_plan_linear(
    const MotionPlannerRequest_t *request,
    const MotionPlannerMachineState_t *machine_state,
    const MotionPlannerConfig_t *config
);

#endif /* FM_V3_MOTION_PLANNER_H */

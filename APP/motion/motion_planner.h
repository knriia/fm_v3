#ifndef FM_V3_MOTION_PLANNER_H
#define FM_V3_MOTION_PLANNER_H

#include <stdint.h>

#define MOTION_PLANNER_AXIS_COUNT 3U
#define MOTION_PLANNER_AXIS_X_MASK 0x01U
#define MOTION_PLANNER_AXIS_Y_MASK 0x02U
#define MOTION_PLANNER_AXIS_Z_MASK 0x04U
#define MOTION_PLANNER_ALL_AXES_MASK 0x07U

typedef struct {
    int32_t min_position_um;
    int32_t max_position_um;
    uint32_t steps_per_mm_numerator;
    uint32_t steps_per_mm_denominator;
    uint32_t max_speed_um_per_s;
} MotionPlannerAxisConfig_t;

typedef struct {
    MotionPlannerAxisConfig_t axis[MOTION_PLANNER_AXIS_COUNT];
    uint32_t max_step_event_rate_hz;
} MotionPlannerConfig_t;

typedef struct {
    int32_t position_um[MOTION_PLANNER_AXIS_COUNT];
    uint8_t homed_axes_mask;
} MotionPlannerMachineState_t;

typedef struct {
    uint32_t sequence;
    int32_t target_um[MOTION_PLANNER_AXIS_COUNT];
    uint32_t feedrate_um_per_s;
} MotionPlannerRequest_t;

/* A constant-feedrate linear move block; timing is represented as an exact ratio. */
typedef struct {
    uint32_t sequence;
    int64_t step_delta[MOTION_PLANNER_AXIS_COUNT];
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
} MotionPlannerResult_t;

/* Pure planner: it validates a homed absolute XYZ move and returns one plan block. */
MotionPlannerResult_t motion_planner_plan_linear(
    const MotionPlannerRequest_t *request,
    const MotionPlannerMachineState_t *machine_state,
    const MotionPlannerConfig_t *config
);

#endif /* FM_V3_MOTION_PLANNER_H */

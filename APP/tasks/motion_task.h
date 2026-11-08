#ifndef FM_V3_MOTION_TASK_H
#define FM_V3_MOTION_TASK_H

#include "motion/axis_calibration.h"
#include "motion/motion_planner.h"
#include "motion_axis_hardware.h"
#include "motion_stop.h"

#include <stdint.h>

#define MOTION_TASK_STACK_SIZE_BYTES (1024U * 2U)
#define MOTION_TASK_ERROR_RETRY_DELAY_MS 10U
/* Highest STEP rate the motor driver accepts; the speed limit of every calibrated axis follows from it. */
#define MOTION_TASK_MAX_STEP_RATE_HZ 150000U
/* STOP waits this long for every axis to come to rest before the step pipeline is reset. */
#define MOTION_TASK_STOP_SETTLE_TIMEOUT_MS 1000U

typedef struct {
    uint32_t commands_dequeued;
    uint32_t queue_receive_errors;
    uint32_t last_sequence;
    uint32_t last_command_code;
    uint32_t planner_calls;
    uint32_t planner_blocks_created;
    uint32_t planner_no_motion;
    uint32_t planner_errors;
    uint32_t last_planner_status;
    uint32_t last_planner_sequence;
} MotionTaskDiagnostics;

/* What the client sees about one axis; the scale position is read live. */
typedef struct {
    uint32_t calibration_phase;
    uint32_t calibration_result;
    uint32_t calibrated;
    uint32_t calibration_steps;
    uint32_t calibration_distance_um;
    uint32_t travel_um;
    int32_t min_position_um;
    int32_t max_position_um;
    uint32_t max_speed_um_per_s;
    uint32_t homed;
    int32_t planned_position_um;
    int32_t scale_position_um;
} MotionAxisDiagnostics;

typedef struct {
    MotionPlannerMachineState_t machine_state;
    MotionPlannerConfig_t planner_config;
    /* Kept in RAM only for now: a reboot needs a new calibration. */
    MachineCalibration_t calibration;
} MotionTaskContext_t;

void MotionTask(void *argument);
void motion_task_get_diagnostics(MotionTaskDiagnostics *diagnostics);
/* An axis outside MotionAxis_t leaves `diagnostics` untouched. */
void motion_task_get_axis_diagnostics(MotionAxis_t axis, MotionAxisDiagnostics *diagnostics);
MotionPlannerStatus_t motion_task_get_last_planner_status(void);
/* STEP pulse width and minimum low time of an axis driver, the timing the planned stream is built from. An axis
 * outside MotionAxis_t leaves both values untouched. */
void motion_task_get_step_timing(MotionAxis_t axis, uint32_t *step_pulse_width_ns, uint32_t *step_low_time_ns);
/* Deceleration of an axis when a limit switch stops it, in steps/s^2; 0 for an axis outside MotionAxis_t. */
uint32_t motion_task_get_limit_deceleration(MotionAxis_t axis);
/* Times a planned step stream ran dry in the middle of a motion since start, summed over all axes. */
uint32_t motion_task_get_stream_underrun_count(void);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void motion_task_test_seed_diagnostics(const MotionTaskDiagnostics *diagnostics);
void motion_task_test_reset_planner(void);
uint32_t motion_task_test_get_planner_call_count(void);
MotionPlannerResult_t motion_task_test_get_last_planner_result(void);
#endif

#endif /* FM_V3_MOTION_TASK_H */

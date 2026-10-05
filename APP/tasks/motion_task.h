#ifndef FM_V3_MOTION_TASK_H
#define FM_V3_MOTION_TASK_H

#include "motion/motion_planner.h"

#include <stdint.h>

#define MOTION_TASK_STACK_SIZE_BYTES (1024U * 2U)
#define MOTION_TASK_ERROR_RETRY_DELAY_MS 10U

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

typedef struct {
    MotionPlannerMachineState_t machine_state;
    MotionPlannerConfig_t planner_config;
} MotionTaskContext_t;

void MotionTask(void *argument);
void motion_task_get_diagnostics(MotionTaskDiagnostics *diagnostics);
MotionPlannerStatus_t motion_task_get_last_planner_status(void);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void motion_task_test_seed_diagnostics(const MotionTaskDiagnostics *diagnostics);
void motion_task_test_reset_planner(void);
uint32_t motion_task_test_get_planner_call_count(void);
MotionPlannerResult_t motion_task_test_get_last_planner_result(void);
#endif

#endif /* FM_V3_MOTION_TASK_H */

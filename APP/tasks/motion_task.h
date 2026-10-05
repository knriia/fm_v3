#ifndef FM_V3_MOTION_TASK_H
#define FM_V3_MOTION_TASK_H

#include <stdint.h>

#define MOTION_TASK_STACK_SIZE_BYTES (1024U * 2U)
#define MOTION_TASK_ERROR_RETRY_DELAY_MS 10U

typedef struct {
    uint32_t commands_dequeued;
    uint32_t queue_receive_errors;
    uint32_t last_sequence;
    uint32_t last_command_code;
} MotionTaskDiagnostics;

void MotionTask(void *argument);
void motion_task_get_diagnostics(MotionTaskDiagnostics *diagnostics);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void motion_task_test_seed_diagnostics(const MotionTaskDiagnostics *diagnostics);
#endif

#endif /* FM_V3_MOTION_TASK_H */

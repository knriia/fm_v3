#ifndef FM_V3_STEP_GENERATOR_TASK_H
#define FM_V3_STEP_GENERATOR_TASK_H

#include <stdint.h>

#define STEP_GENERATOR_TASK_STACK_SIZE_BYTES (1024U * 2U)

typedef struct {
    uint32_t blocks_started;
    uint32_t blocks_completed;
    uint32_t events_generated;
    uint32_t generator_errors;
    uint32_t buffer_full_waits;
    uint32_t axis_starts;
    uint32_t axis_start_errors;
    uint32_t stop_resets;
} StepGeneratorTaskDiagnostics;

/* Turns plan blocks into step events, keeps ExecutionBuffer filled and starts the axis when it is idle. */
void StepGeneratorTask(void *argument);
/* Called by MotionTask after it pushes a block into PlanBuffer. */
void step_generator_task_notify_block_ready(void);
/* STOP: stop feeding and starting the axis right away; callable from any task. */
void step_generator_task_halt(void);
/* After STOP, once the axis no longer moves: drop the started block and all queued step events, then go on. */
void step_generator_task_resume_after_stop(void);
void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics);

#endif /* FM_V3_STEP_GENERATOR_TASK_H */

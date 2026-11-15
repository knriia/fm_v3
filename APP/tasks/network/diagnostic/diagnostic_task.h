//
// Created by umnik on 23.09.2026.
//

#ifndef FM_V3_DIAGNOSTIC_TASK_H
#define FM_V3_DIAGNOSTIC_TASK_H

#include "motion/plan_buffer.h"
#include "task_context.h"

#define DIAGNOSTIC_NETWORK_TASK_PORT 1428
#define DIAGNOSTIC_NETWORK_TASK_INTERVAL_MS 1000
#define DIAGNOSTIC_NETWORK_SEND_TIMEOUT_MS 500
#define DIAGNOSTIC_TASK_STACK_SIZE_BYTES (1024U * 5U)

typedef struct {
    NetworkTaskContext network;
    /* The queue of plan blocks the frame reports on; it outlives the task. */
    PlanBuffer_t *plan_buffer;
} DiagnosticTaskContext;

/* argument is a DiagnosticTaskContext * that outlives the task. */
void DiagnosticTask(void *argument);

#endif // FM_V3_DIAGNOSTIC_TASK_H

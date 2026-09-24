#ifndef STARTUP_TASK_H
#define STARTUP_TASK_H

#include <stdint.h>

#define STARTUP_TASK_STACK_SIZE_BYTES (1024U * 4U)

typedef struct {
    uint32_t lwip_init_completed;
    uint32_t lwip_ready_flag_set_attempts;
    uint32_t lwip_ready_flag_set_errors;
} StartupTaskDiagnostics;

void StartStartupTask(void *argument);
void startup_task_get_diagnostics(StartupTaskDiagnostics *diagnostics);

#endif /* STARTUP_TASK_H */

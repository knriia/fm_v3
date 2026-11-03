#ifndef STARTUP_TASK_H
#define STARTUP_TASK_H

#include <stdint.h>

#define STARTUP_TASK_STACK_SIZE_BYTES (1024U * 4U)

void StartStartupTask(void *argument);

#endif /* STARTUP_TASK_H */

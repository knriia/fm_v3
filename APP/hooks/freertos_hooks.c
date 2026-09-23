#include "FreeRTOS.h"
#include "task.h"

#include "freertos_hooks.h"

static volatile uint32_t stack_overflow_count;
static volatile uint32_t malloc_failed_count;

static void freertos_hook_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;

    freertos_hook_counter_increment(&stack_overflow_count);

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void) {
    freertos_hook_counter_increment(&malloc_failed_count);

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void freertos_hooks_get_diagnostics(uint32_t *stack_overflows, uint32_t *malloc_failures) {
    if (stack_overflows != NULL) {
        *stack_overflows = stack_overflow_count;
    }

    if (malloc_failures != NULL) {
        *malloc_failures = malloc_failed_count;
    }
}

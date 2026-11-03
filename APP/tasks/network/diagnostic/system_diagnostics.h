#ifndef FM_V3_SYSTEM_DIAGNOSTICS_H
#define FM_V3_SYSTEM_DIAGNOSTICS_H

#include <stdint.h>

typedef struct {
    /* FreeRTOS ticks since start; one tick is 1 ms. */
    uint32_t tick_count;
    /* RCC_RSR: why the MCU was last reset (power, pin, watchdog, brown-out, software). */
    uint32_t reset_flags;
    uint32_t heap_free_bytes;
    uint32_t heap_min_free_bytes;
    uint32_t task_count;
    uint32_t cpu_usage_percent;
    uint32_t stack_overflow_count;
    uint32_t malloc_failed_count;
} SystemDiagnostics;

/* Fills the snapshot and returns the FreeRTOS run-time total that per-task CPU shares are measured against. */
uint32_t system_diagnostics_collect(SystemDiagnostics *diagnostics);

#endif /* FM_V3_SYSTEM_DIAGNOSTICS_H */

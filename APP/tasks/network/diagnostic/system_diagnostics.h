#ifndef FM_V3_SYSTEM_DIAGNOSTICS_H
#define FM_V3_SYSTEM_DIAGNOSTICS_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t base_address;
    uint32_t total_bytes;
    uint32_t used_bytes;
    uint32_t reserved_bytes;
    uint32_t free_bytes;
} MemoryRegionDiagnostics;

typedef struct __attribute__((packed)) {
    MemoryRegionDiagnostics itcm;
    MemoryRegionDiagnostics dtcm;
    MemoryRegionDiagnostics d1;
    MemoryRegionDiagnostics d2;
    MemoryRegionDiagnostics d3;
} MemoryDiagnostics;

typedef struct __attribute__((packed)) {
    uint32_t total_heap;
    uint32_t used_heap;
    uint32_t free_heap;
    uint32_t minimum_free_heap;
    uint32_t task_count;
    uint32_t task_snapshot_count;
    uint32_t task_snapshot_capacity;
    uint32_t task_snapshot_truncated;
    uint32_t scheduler_state;
    uint32_t tick_count;
    uint32_t runtime_counter_hz;
    uint32_t total_runtime_ticks;
    uint32_t idle_runtime_ticks;
    uint32_t cpu_usage_percent;
    uint32_t stack_overflow_count;
    uint32_t malloc_failed_count;
} FreeRtosDiagnostics;

typedef struct {
    uint32_t free_heap;
    uint32_t minimum_free_heap;
    uint32_t uid0;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t device_id;
    uint32_t revision_id;
    uint32_t reset_flags;
    uint32_t sys_clock_hz;
    uint32_t hclk_hz;
    uint32_t pclk1_hz;
    uint32_t pclk2_hz;
    uint32_t systick_ms;
    uint32_t scb_cfsr;
    uint32_t scb_hfsr;
    uint32_t scb_dfsr;
    uint32_t scb_mmfar;
    uint32_t scb_bfar;
    uint32_t scb_afsr;
    uint32_t scb_shcsr;
    uint32_t scb_ccr;
    uint32_t scb_icsr;
    uint32_t scb_vtor;
    uint32_t mpu_ctrl;
    uint32_t dwt_ctrl;
    uint32_t dwt_cyccnt;
    MemoryDiagnostics memory;
    FreeRtosDiagnostics freertos;
    uint32_t ipsr;
    uint32_t control;
    uint32_t basepri;
    uint32_t primask;
    uint32_t faultmask;
    uint32_t msp;
    uint32_t psp;
    uint32_t systick_ctrl;
    uint32_t systick_load;
    uint32_t systick_val;
    uint32_t nvic_iser0;
    uint32_t nvic_ispr0;
    uint32_t nvic_iabr0;
} SystemDiagnostics;

void system_diagnostics_collect(SystemDiagnostics *diagnostics);

#endif /* FM_V3_SYSTEM_DIAGNOSTICS_H */

#ifndef FM_V3_SYSTEM_DIAGNOSTICS_H
#define FM_V3_SYSTEM_DIAGNOSTICS_H

#include <stdint.h>

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
} SystemDiagnostics;

void system_diagnostics_collect(SystemDiagnostics *diagnostics);

#endif /* FM_V3_SYSTEM_DIAGNOSTICS_H */

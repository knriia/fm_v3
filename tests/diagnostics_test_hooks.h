#ifndef FM_V3_DIAGNOSTICS_TEST_HOOKS_H
#define FM_V3_DIAGNOSTICS_TEST_HOOKS_H

#include "system_diagnostics.h"

#include <stdint.h>

void system_diagnostics_test_collect_memory(MemoryDiagnostics *diagnostics);
void system_diagnostics_test_fill_memory_region(
    MemoryRegionDiagnostics *diagnostics,
    uint32_t base_address,
    uint32_t total_bytes,
    uintptr_t used_end,
    uint32_t reserved_bytes
);
void lwip_udp_diagnostics_test_set_packet_count(uint32_t packet_count);

#endif /* FM_V3_DIAGNOSTICS_TEST_HOOKS_H */

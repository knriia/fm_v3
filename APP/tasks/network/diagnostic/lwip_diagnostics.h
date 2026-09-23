#ifndef FM_V3_LWIP_DIAGNOSTICS_H
#define FM_V3_LWIP_DIAGNOSTICS_H

#include <stdint.h>

typedef struct {
    uint32_t xmit;
    uint32_t recv;
    uint32_t fw;
    uint32_t drop;
    uint32_t chkerr;
    uint32_t lenerr;
    uint32_t memerr;
    uint32_t rterr;
    uint32_t proterr;
    uint32_t opterr;
    uint32_t err;
    uint32_t cachehit;
} LwipProtocolDiagnostics;

typedef struct {
    uint32_t used;
    uint32_t max;
    uint32_t err;
} LwipSystemResourceDiagnostics;

typedef struct {
    uint32_t netif_up;
    uint32_t link_up;
    uint32_t flags;
    uint32_t mtu;
    uint32_t ip_address;
    uint32_t netmask;
    uint32_t gateway;
    uint32_t mem_err;
    uint32_t mem_available;
    uint32_t mem_used;
    uint32_t mem_max_used;
    uint32_t mem_illegal;
    uint32_t memp_used;
    uint32_t memp_max_used;
    uint32_t memp_errors;
    uint32_t memp_illegal;
    LwipSystemResourceDiagnostics semaphores;
    LwipSystemResourceDiagnostics mutexes;
    LwipSystemResourceDiagnostics mailboxes;
    LwipProtocolDiagnostics link;
    LwipProtocolDiagnostics etharp;
    LwipProtocolDiagnostics ip;
    LwipProtocolDiagnostics icmp;
    LwipProtocolDiagnostics tcp;
    uint32_t if_in_octets;
    uint32_t if_in_unicast_packets;
    uint32_t if_in_non_unicast_packets;
    uint32_t if_in_discards;
    uint32_t if_in_errors;
    uint32_t if_in_unknown_protocols;
    uint32_t if_out_octets;
    uint32_t if_out_unicast_packets;
    uint32_t if_out_non_unicast_packets;
    uint32_t if_out_discards;
    uint32_t if_out_errors;
} LwipDiagnostics;

void lwip_diagnostics_collect(LwipDiagnostics *diagnostics);

#endif /* FM_V3_LWIP_DIAGNOSTICS_H */

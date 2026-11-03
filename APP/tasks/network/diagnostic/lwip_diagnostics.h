#ifndef FM_V3_LWIP_DIAGNOSTICS_H
#define FM_V3_LWIP_DIAGNOSTICS_H

#include <stdint.h>

typedef struct {
    uint32_t max_used;
    uint32_t errors;
} LwipPoolDiagnostics;

/* LwIP resources that run out under load; every errors counter means an allocation or a post failed. */
typedef struct {
    uint32_t heap_max_used;
    uint32_t heap_errors;
    /* Errors summed over every memory pool, including the ones not listed below. */
    uint32_t pool_errors;
    LwipPoolDiagnostics tcp_pcb;
    LwipPoolDiagnostics tcp_seg;
    LwipPoolDiagnostics pbuf_pool;
    LwipPoolDiagnostics netconn;
    LwipPoolDiagnostics netbuf;
    LwipPoolDiagnostics tcpip_msg_api;
    LwipPoolDiagnostics tcpip_msg_input;
    /* A full mailbox, e.g. the tcpip thread queue, rejects the message. */
    uint32_t mailbox_errors;
    uint32_t semaphore_errors;
    uint32_t mutex_errors;
    uint32_t tcp_drops;
    uint32_t tcp_errors;
    uint32_t tcp_memory_errors;
    uint32_t if_in_discards;
    uint32_t if_out_discards;
} LwipDiagnostics;

void lwip_diagnostics_collect(LwipDiagnostics *diagnostics);

#endif /* FM_V3_LWIP_DIAGNOSTICS_H */

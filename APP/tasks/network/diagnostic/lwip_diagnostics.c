#include "lwip_diagnostics.h"

#include "lwip/memp.h"
#include "lwip/netif.h"
#include "lwip/stats.h"

#include <stddef.h>

extern struct netif gnetif;

static LwipPoolDiagnostics lwip_diagnostics_pool(memp_t pool) {
    const struct stats_mem *stats = lwip_stats.memp[pool];
    if (stats == NULL) {
        return (LwipPoolDiagnostics){0};
    }
    return (LwipPoolDiagnostics){.max_used = stats->max, .errors = stats->err};
}

void lwip_diagnostics_collect(LwipDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->heap_max_used = lwip_stats.mem.max;
    diagnostics->heap_errors = lwip_stats.mem.err;
    diagnostics->pool_errors = 0U;
    for (uint32_t pool = 0U; pool < (uint32_t)MEMP_MAX; ++pool) {
        diagnostics->pool_errors += lwip_diagnostics_pool((memp_t)pool).errors;
    }
    diagnostics->tcp_pcb = lwip_diagnostics_pool(MEMP_TCP_PCB);
    diagnostics->tcp_seg = lwip_diagnostics_pool(MEMP_TCP_SEG);
    diagnostics->pbuf_pool = lwip_diagnostics_pool(MEMP_PBUF_POOL);
    diagnostics->netconn = lwip_diagnostics_pool(MEMP_NETCONN);
    diagnostics->netbuf = lwip_diagnostics_pool(MEMP_NETBUF);
    diagnostics->tcpip_msg_api = lwip_diagnostics_pool(MEMP_TCPIP_MSG_API);
    diagnostics->tcpip_msg_input = lwip_diagnostics_pool(MEMP_TCPIP_MSG_INPKT);

    diagnostics->mailbox_errors = lwip_stats.sys.mbox.err;
    diagnostics->semaphore_errors = lwip_stats.sys.sem.err;
    diagnostics->mutex_errors = lwip_stats.sys.mutex.err;
    diagnostics->tcp_drops = lwip_stats.tcp.drop;
    diagnostics->tcp_errors = lwip_stats.tcp.err;
    diagnostics->tcp_memory_errors = lwip_stats.tcp.memerr;
    diagnostics->if_in_discards = gnetif.mib2_counters.ifindiscards;
    diagnostics->if_out_discards = gnetif.mib2_counters.ifoutdiscards;
}

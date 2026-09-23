#include "lwip_diagnostics.h"

#include "lwip.h"
#include "lwip/netif.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"

extern struct netif gnetif;

static void copy_protocol_stats(LwipProtocolDiagnostics *destination, const struct stats_proto *source) {
    destination->xmit = source->xmit;
    destination->recv = source->recv;
    destination->fw = source->fw;
    destination->drop = source->drop;
    destination->chkerr = source->chkerr;
    destination->lenerr = source->lenerr;
    destination->memerr = source->memerr;
    destination->rterr = source->rterr;
    destination->proterr = source->proterr;
    destination->opterr = source->opterr;
    destination->err = source->err;
    destination->cachehit = source->cachehit;
}

void lwip_diagnostics_collect(LwipDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->netif_up = netif_is_up(&gnetif);
    diagnostics->link_up = netif_is_link_up(&gnetif);
    diagnostics->flags = gnetif.flags;
    diagnostics->mtu = gnetif.mtu;
    diagnostics->ip_address = ip4_addr_get_u32(netif_ip4_addr(&gnetif));
    diagnostics->netmask = ip4_addr_get_u32(netif_ip4_netmask(&gnetif));
    diagnostics->gateway = ip4_addr_get_u32(netif_ip4_gw(&gnetif));

    diagnostics->mem_err = lwip_stats.mem.err;
    diagnostics->mem_available = lwip_stats.mem.avail;
    diagnostics->mem_used = lwip_stats.mem.used;
    diagnostics->mem_max_used = lwip_stats.mem.max;
    diagnostics->mem_illegal = lwip_stats.mem.illegal;

    diagnostics->memp_used = 0U;
    diagnostics->memp_max_used = 0U;
    diagnostics->memp_errors = 0U;
    diagnostics->memp_illegal = 0U;
    for (uint32_t index = 0U; index < MEMP_MAX; ++index) {
        if (lwip_stats.memp[index] != NULL) {
            diagnostics->memp_used += lwip_stats.memp[index]->used;
            diagnostics->memp_max_used += lwip_stats.memp[index]->max;
            diagnostics->memp_errors += lwip_stats.memp[index]->err;
            diagnostics->memp_illegal += lwip_stats.memp[index]->illegal;
        }
    }

    diagnostics->semaphores.used = lwip_stats.sys.sem.used;
    diagnostics->semaphores.max = lwip_stats.sys.sem.max;
    diagnostics->semaphores.err = lwip_stats.sys.sem.err;
    diagnostics->mutexes.used = lwip_stats.sys.mutex.used;
    diagnostics->mutexes.max = lwip_stats.sys.mutex.max;
    diagnostics->mutexes.err = lwip_stats.sys.mutex.err;
    diagnostics->mailboxes.used = lwip_stats.sys.mbox.used;
    diagnostics->mailboxes.max = lwip_stats.sys.mbox.max;
    diagnostics->mailboxes.err = lwip_stats.sys.mbox.err;

    copy_protocol_stats(&diagnostics->link, &lwip_stats.link);
    copy_protocol_stats(&diagnostics->etharp, &lwip_stats.etharp);
    copy_protocol_stats(&diagnostics->ip, &lwip_stats.ip);
    copy_protocol_stats(&diagnostics->icmp, &lwip_stats.icmp);
    copy_protocol_stats(&diagnostics->tcp, &lwip_stats.tcp);

    diagnostics->if_in_octets = gnetif.mib2_counters.ifinoctets;
    diagnostics->if_in_unicast_packets = gnetif.mib2_counters.ifinucastpkts;
    diagnostics->if_in_non_unicast_packets = gnetif.mib2_counters.ifinnucastpkts;
    diagnostics->if_in_discards = gnetif.mib2_counters.ifindiscards;
    diagnostics->if_in_errors = gnetif.mib2_counters.ifinerrors;
    diagnostics->if_in_unknown_protocols = gnetif.mib2_counters.ifinunknownprotos;
    diagnostics->if_out_octets = gnetif.mib2_counters.ifoutoctets;
    diagnostics->if_out_unicast_packets = gnetif.mib2_counters.ifoutucastpkts;
    diagnostics->if_out_non_unicast_packets = gnetif.mib2_counters.ifoutnucastpkts;
    diagnostics->if_out_discards = gnetif.mib2_counters.ifoutdiscards;
    diagnostics->if_out_errors = gnetif.mib2_counters.ifouterrors;
}

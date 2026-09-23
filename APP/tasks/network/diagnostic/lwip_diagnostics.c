#include "lwip_diagnostics.h"

#include "lwip.h"
#include "lwip/netif.h"
#include "lwip/priv/memp_priv.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"
#include "lwip_udp_diagnostics.h"

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

static void copy_mib2_stats(LwipMib2Diagnostics *destination, const struct stats_mib2 *source) {
    destination->ipinhdrerrors = source->ipinhdrerrors;
    destination->ipinaddrerrors = source->ipinaddrerrors;
    destination->ipinunknownprotos = source->ipinunknownprotos;
    destination->ipindiscards = source->ipindiscards;
    destination->ipindelivers = source->ipindelivers;
    destination->ipoutrequests = source->ipoutrequests;
    destination->ipoutdiscards = source->ipoutdiscards;
    destination->ipoutnoroutes = source->ipoutnoroutes;
    destination->ipreasmoks = source->ipreasmoks;
    destination->ipreasmfails = source->ipreasmfails;
    destination->ipfragoks = source->ipfragoks;
    destination->ipfragfails = source->ipfragfails;
    destination->ipfragcreates = source->ipfragcreates;
    destination->ipreasmreqds = source->ipreasmreqds;
    destination->ipforwdatagrams = source->ipforwdatagrams;
    destination->ipinreceives = source->ipinreceives;
    destination->tcpactiveopens = source->tcpactiveopens;
    destination->tcppassiveopens = source->tcppassiveopens;
    destination->tcpattemptfails = source->tcpattemptfails;
    destination->tcpestabresets = source->tcpestabresets;
    destination->tcpoutsegs = source->tcpoutsegs;
    destination->tcpretranssegs = source->tcpretranssegs;
    destination->tcpinsegs = source->tcpinsegs;
    destination->tcpinerrs = source->tcpinerrs;
    destination->tcpoutrsts = source->tcpoutrsts;
    destination->udpindatagrams = source->udpindatagrams;
    destination->udpnoports = source->udpnoports;
    destination->udpinerrors = source->udpinerrors;
    destination->udpoutdatagrams = source->udpoutdatagrams;
    destination->icmpinmsgs = source->icmpinmsgs;
    destination->icmpinerrors = source->icmpinerrors;
    destination->icmpindestunreachs = source->icmpindestunreachs;
    destination->icmpintimeexcds = source->icmpintimeexcds;
    destination->icmpinparmprobs = source->icmpinparmprobs;
    destination->icmpinsrcquenchs = source->icmpinsrcquenchs;
    destination->icmpinredirects = source->icmpinredirects;
    destination->icmpinechos = source->icmpinechos;
    destination->icmpinechoreps = source->icmpinechoreps;
    destination->icmpintimestamps = source->icmpintimestamps;
    destination->icmpintimestampreps = source->icmpintimestampreps;
    destination->icmpinaddrmasks = source->icmpinaddrmasks;
    destination->icmpinaddrmaskreps = source->icmpinaddrmaskreps;
    destination->icmpoutmsgs = source->icmpoutmsgs;
    destination->icmpouterrors = source->icmpouterrors;
    destination->icmpoutdestunreachs = source->icmpoutdestunreachs;
    destination->icmpouttimeexcds = source->icmpouttimeexcds;
    destination->icmpoutechos = source->icmpoutechos;
    destination->icmpoutechoreps = source->icmpoutechoreps;
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
    diagnostics->netif_link_type = gnetif.link_type;
    diagnostics->netif_link_speed = gnetif.link_speed;
    diagnostics->netif_timestamp = gnetif.ts;

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
        diagnostics->memp[index].used = 0U;
        diagnostics->memp[index].max = 0U;
        diagnostics->memp[index].err = 0U;
        diagnostics->memp[index].illegal = 0U;
        diagnostics->memp[index].capacity = memp_pools[index]->num;
        diagnostics->memp[index].element_size = memp_pools[index]->size;
        if (lwip_stats.memp[index] != NULL) {
            diagnostics->memp_used += lwip_stats.memp[index]->used;
            diagnostics->memp_max_used += lwip_stats.memp[index]->max;
            diagnostics->memp_errors += lwip_stats.memp[index]->err;
            diagnostics->memp_illegal += lwip_stats.memp[index]->illegal;
            diagnostics->memp[index].used = lwip_stats.memp[index]->used;
            diagnostics->memp[index].max = lwip_stats.memp[index]->max;
            diagnostics->memp[index].err = lwip_stats.memp[index]->err;
            diagnostics->memp[index].illegal = lwip_stats.memp[index]->illegal;
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
    diagnostics->tcpip_thread_stack_size = TCPIP_THREAD_STACKSIZE;
    diagnostics->tcpip_thread_priority = TCPIP_THREAD_PRIO;
    diagnostics->tcpip_mbox_size = TCPIP_MBOX_SIZE;
    diagnostics->tcp_mss = TCP_MSS;
    diagnostics->tcp_snd_buf = TCP_SND_BUF;
    diagnostics->tcp_wnd = TCP_WND;
    diagnostics->tcp_snd_queuelen = TCP_SND_QUEUELEN;
    diagnostics->pbuf_pool_size = PBUF_POOL_SIZE;
    diagnostics->pbuf_pool_bufsize = PBUF_POOL_BUFSIZE;
    copy_mib2_stats(&diagnostics->mib2, &lwip_stats.mib2);
    diagnostics->udp_packets_count = lwip_udp_diagnostics_get_packet_count();

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

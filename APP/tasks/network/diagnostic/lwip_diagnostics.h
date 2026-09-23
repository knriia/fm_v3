#ifndef FM_V3_LWIP_DIAGNOSTICS_H
#define FM_V3_LWIP_DIAGNOSTICS_H

#include <stdint.h>

#include "lwip/memp.h"

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

typedef struct __attribute__((packed)) {
    uint32_t used;
    uint32_t max;
    uint32_t err;
    uint32_t illegal;
    uint32_t capacity;
    uint32_t element_size;
} LwipMemoryPoolDiagnostics;

typedef struct __attribute__((packed)) {
    uint32_t ipinhdrerrors;
    uint32_t ipinaddrerrors;
    uint32_t ipinunknownprotos;
    uint32_t ipindiscards;
    uint32_t ipindelivers;
    uint32_t ipoutrequests;
    uint32_t ipoutdiscards;
    uint32_t ipoutnoroutes;
    uint32_t ipreasmoks;
    uint32_t ipreasmfails;
    uint32_t ipfragoks;
    uint32_t ipfragfails;
    uint32_t ipfragcreates;
    uint32_t ipreasmreqds;
    uint32_t ipforwdatagrams;
    uint32_t ipinreceives;
    uint32_t tcpactiveopens;
    uint32_t tcppassiveopens;
    uint32_t tcpattemptfails;
    uint32_t tcpestabresets;
    uint32_t tcpoutsegs;
    uint32_t tcpretranssegs;
    uint32_t tcpinsegs;
    uint32_t tcpinerrs;
    uint32_t tcpoutrsts;
    uint32_t udpindatagrams;
    uint32_t udpnoports;
    uint32_t udpinerrors;
    uint32_t udpoutdatagrams;
    uint32_t icmpinmsgs;
    uint32_t icmpinerrors;
    uint32_t icmpindestunreachs;
    uint32_t icmpintimeexcds;
    uint32_t icmpinparmprobs;
    uint32_t icmpinsrcquenchs;
    uint32_t icmpinredirects;
    uint32_t icmpinechos;
    uint32_t icmpinechoreps;
    uint32_t icmpintimestamps;
    uint32_t icmpintimestampreps;
    uint32_t icmpinaddrmasks;
    uint32_t icmpinaddrmaskreps;
    uint32_t icmpoutmsgs;
    uint32_t icmpouterrors;
    uint32_t icmpoutdestunreachs;
    uint32_t icmpouttimeexcds;
    uint32_t icmpoutechos;
    uint32_t icmpoutechoreps;
} LwipMib2Diagnostics;

typedef struct {
    uint32_t netif_up;
    uint32_t link_up;
    uint32_t flags;
    uint32_t mtu;
    uint32_t ip_address;
    uint32_t netmask;
    uint32_t gateway;
    uint32_t netif_link_type;
    uint32_t netif_link_speed;
    uint32_t netif_timestamp;
    uint32_t mem_err;
    uint32_t mem_available;
    uint32_t mem_used;
    uint32_t mem_max_used;
    uint32_t mem_illegal;
    uint32_t memp_used;
    uint32_t memp_max_used;
    uint32_t memp_errors;
    uint32_t memp_illegal;
    LwipMemoryPoolDiagnostics memp[MEMP_MAX];
    LwipSystemResourceDiagnostics semaphores;
    LwipSystemResourceDiagnostics mutexes;
    LwipSystemResourceDiagnostics mailboxes;
    LwipProtocolDiagnostics link;
    LwipProtocolDiagnostics etharp;
    LwipProtocolDiagnostics ip;
    LwipProtocolDiagnostics icmp;
    LwipProtocolDiagnostics tcp;
    uint32_t tcpip_thread_stack_size;
    uint32_t tcpip_thread_priority;
    uint32_t tcpip_mbox_size;
    uint32_t tcp_mss;
    uint32_t tcp_snd_buf;
    uint32_t tcp_wnd;
    uint32_t tcp_snd_queuelen;
    uint32_t pbuf_pool_size;
    uint32_t pbuf_pool_bufsize;
    LwipMib2Diagnostics mib2;
    uint32_t udp_packets_count;
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

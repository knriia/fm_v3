#ifndef FM_V3_TEST_LWIP_STATS_H
#define FM_V3_TEST_LWIP_STATS_H

#include "lwip/memp.h"

#include <stdint.h>

struct stats_mem {
    uint32_t err;
    uint32_t avail;
    uint32_t used;
    uint32_t max;
    uint32_t illegal;
};

struct stats_memp {
    uint32_t used;
    uint32_t max;
    uint32_t err;
    uint32_t illegal;
};

struct stats_sys_resource {
    uint32_t used;
    uint32_t max;
    uint32_t err;
};

struct stats_sys {
    struct stats_sys_resource sem;
    struct stats_sys_resource mutex;
    struct stats_sys_resource mbox;
};

struct stats_proto {
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
};

struct stats_mib2 {
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
};

struct stats {
    struct stats_mem mem;
    struct stats_memp *memp[MEMP_MAX];
    struct stats_sys sys;
    struct stats_proto link;
    struct stats_proto etharp;
    struct stats_proto ip;
    struct stats_proto icmp;
    struct stats_proto tcp;
    struct stats_mib2 mib2;
};

extern struct stats lwip_stats;

#endif /* FM_V3_TEST_LWIP_STATS_H */

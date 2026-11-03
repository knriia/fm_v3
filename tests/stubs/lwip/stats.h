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

struct stats {
    struct stats_mem mem;
    struct stats_mem *memp[MEMP_MAX];
    struct stats_sys sys;
    struct stats_proto tcp;
};

extern struct stats lwip_stats;

#endif /* FM_V3_TEST_LWIP_STATS_H */

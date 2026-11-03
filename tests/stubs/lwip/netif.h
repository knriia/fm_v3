#ifndef FM_V3_TEST_LWIP_NETIF_H
#define FM_V3_TEST_LWIP_NETIF_H

#include <stdint.h>

struct netif_mib2_counters {
    uint32_t ifindiscards;
    uint32_t ifoutdiscards;
};

struct netif {
    struct netif_mib2_counters mib2_counters;
};

#endif /* FM_V3_TEST_LWIP_NETIF_H */

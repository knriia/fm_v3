#ifndef FM_V3_TEST_LWIP_NETIF_H
#define FM_V3_TEST_LWIP_NETIF_H

#include <stdint.h>

#define TEST_NETIF_FLAG_UP 0x01U
#define TEST_NETIF_FLAG_LINK_UP 0x02U

struct ip4_addr {
    uint32_t addr;
};

struct netif_mib2_counters {
    uint32_t ifinoctets;
    uint32_t ifinucastpkts;
    uint32_t ifinnucastpkts;
    uint32_t ifindiscards;
    uint32_t ifinerrors;
    uint32_t ifinunknownprotos;
    uint32_t ifoutoctets;
    uint32_t ifoutucastpkts;
    uint32_t ifoutnucastpkts;
    uint32_t ifoutdiscards;
    uint32_t ifouterrors;
};

struct netif {
    uint8_t flags;
    uint16_t mtu;
    uint8_t link_type;
    uint32_t link_speed;
    uint32_t ts;
    struct ip4_addr ip_addr;
    struct ip4_addr netmask;
    struct ip4_addr gw;
    struct netif_mib2_counters mib2_counters;
};

static inline uint8_t netif_is_up(const struct netif *netif) {
    return (uint8_t)((netif->flags & TEST_NETIF_FLAG_UP) != 0U);
}

static inline uint8_t netif_is_link_up(const struct netif *netif) {
    return (uint8_t)((netif->flags & TEST_NETIF_FLAG_LINK_UP) != 0U);
}

static inline const struct ip4_addr *netif_ip4_addr(const struct netif *netif) { return &netif->ip_addr; }

static inline const struct ip4_addr *netif_ip4_netmask(const struct netif *netif) { return &netif->netmask; }

static inline const struct ip4_addr *netif_ip4_gw(const struct netif *netif) { return &netif->gw; }

static inline uint32_t ip4_addr_get_u32(const struct ip4_addr *address) { return address->addr; }

#endif /* FM_V3_TEST_LWIP_NETIF_H */

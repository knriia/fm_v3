#include "lwip_udp_diagnostics.h"

#include <limits.h>

static volatile uint32_t udp_packets_count;

void lwip_udp_diagnostics_record_packet(void) {
    if (udp_packets_count != UINT32_MAX) {
        ++udp_packets_count;
    }
}

uint32_t lwip_udp_diagnostics_get_packet_count(void) {
    return udp_packets_count;
}

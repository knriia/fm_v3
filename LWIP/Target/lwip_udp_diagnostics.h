#ifndef FM_V3_LWIP_UDP_DIAGNOSTICS_H
#define FM_V3_LWIP_UDP_DIAGNOSTICS_H

#include <stdint.h>

void lwip_udp_diagnostics_record_packet(void);
uint32_t lwip_udp_diagnostics_get_packet_count(void);

#endif /* FM_V3_LWIP_UDP_DIAGNOSTICS_H */

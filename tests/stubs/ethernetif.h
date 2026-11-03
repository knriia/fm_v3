#ifndef FM_V3_TEST_ETHERNETIF_H
#define FM_V3_TEST_ETHERNETIF_H

#include <stdint.h>

/* Only the fields the diagnostic snapshot reads; names match LWIP/Target/ethernetif.h. */
typedef struct {
    uint32_t hal_read_data_errors;
    uint32_t dma_receive_buffer_unavailable;
    uint32_t dropped_packets;
    uint32_t rx_pool_exhaustions;
    uint32_t rx_pool_max_used;
} EthernetRxDiagnostics;

#define ETHERNETIF_INPUT_THREAD_STACK_SIZE_BYTES 1024U
#define ETHERNETIF_LINK_THREAD_STACK_SIZE_BYTES 2048U

void ethernetif_get_rx_diagnostics(EthernetRxDiagnostics *diagnostics);
void Error_Handler(void);

#endif /* FM_V3_TEST_ETHERNETIF_H */

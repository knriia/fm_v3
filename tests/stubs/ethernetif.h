#ifndef FM_V3_TEST_ETHERNETIF_H
#define FM_V3_TEST_ETHERNETIF_H

#include <stdint.h>

typedef struct {
    uint32_t values[16];
} EthernetRxDiagnostics;

#define ETHERNETIF_INPUT_THREAD_STACK_SIZE_BYTES 1024U
#define ETHERNETIF_LINK_THREAD_STACK_SIZE_BYTES 1024U

void ethernetif_get_rx_diagnostics(EthernetRxDiagnostics *diagnostics);
void Error_Handler(void);

#endif /* FM_V3_TEST_ETHERNETIF_H */

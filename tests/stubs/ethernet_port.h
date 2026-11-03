#ifndef FM_V3_TEST_ETHERNET_PORT_H
#define FM_V3_TEST_ETHERNET_PORT_H

#include <stdint.h>

/* Only the fields the diagnostic snapshot reads; names match BSP/ETH/ethernet_port.h. */
typedef struct {
    uint32_t link_up;
    uint32_t speed;
    uint32_t duplex;
    uint32_t dma_error;
    uint32_t link_up_events;
    uint32_t link_down_events;
    uint32_t link_state_read_errors;
    uint32_t tx_errors;
    uint32_t tx_descriptor_overflows;
    uint32_t mutex_timeouts;
} EthernetPortDiagnostics;

typedef int HAL_StatusTypeDef;

#define HAL_OK 0

HAL_StatusTypeDef EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics);

#endif /* FM_V3_TEST_ETHERNET_PORT_H */

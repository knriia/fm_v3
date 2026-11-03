#ifndef ETHERNET_PORT_H
#define ETHERNET_PORT_H

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "stm32h7xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef EthernetPort_Init(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef EthernetPort_ReadData(void **pAppBuff);
HAL_StatusTypeDef EthernetPort_HandleLinkInterrupt(struct netif *netif);
void EthernetPort_CheckLinkState(struct netif *netif);
err_t EthernetPort_LowLevelOutput(struct netif *netif, struct pbuf *p);

/* What the diagnostic frame reports about the port: link state and the counters that point at a fault. */
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

HAL_StatusTypeDef EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* ETHERNET_PORT_H */

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

typedef struct {
    uint32_t link_up;
    uint32_t speed;
    uint32_t duplex;
    uint32_t netif_up;
    uint32_t hal_state;
    uint32_t hal_error;
    uint32_t dma_error;
    uint32_t dma_status;
    uint32_t dma_debug_status;
    uint32_t mac_control;
    uint32_t phy_address;
    uint32_t phy_id1;
    uint32_t phy_id2;
    uint32_t rx_descriptor_address;
    uint32_t tx_descriptor_address;
    uint32_t rx_descriptor_base;
    uint32_t tx_descriptor_base;
    uint32_t rx_descriptor_count;
    uint32_t tx_descriptor_count;
    uint32_t rx_descriptor_length;
    uint32_t tx_descriptor_length;
    uint32_t rx_buffer_size;
    uint32_t rx_buffer_count;
    uint32_t rx_descriptor_current;
    uint32_t tx_descriptor_current;
    uint32_t rx_descriptor_tail;
    uint32_t tx_descriptor_tail;
    uint32_t link_up_events;
    uint32_t link_down_events;
    uint32_t link_reconfigurations;
    uint32_t link_state_read_errors;
    uint32_t link_interrupt_errors;
    uint32_t phy_init_errors;
    uint32_t netif_state_errors;
    uint32_t hal_stop_errors;
    uint32_t hal_config_errors;
    uint32_t hal_start_errors;
    uint32_t tx_attempts;
    uint32_t tx_successes;
    uint32_t tx_errors;
    uint32_t tx_bytes;
    uint32_t tx_descriptor_overflows;
    uint32_t mutex_timeouts;
    uint8_t mac_address[6];
} EthernetPortDiagnostics;

HAL_StatusTypeDef EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* ETHERNET_PORT_H */

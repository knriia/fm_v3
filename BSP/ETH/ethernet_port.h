#ifndef ETHERNET_PORT_H
#define ETHERNET_PORT_H

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef EthernetPort_Init(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef EthernetPort_ReadData(void **pAppBuff);
HAL_StatusTypeDef EthernetPort_HandleLinkInterrupt(struct netif *netif);
void EthernetPort_CheckLinkState(struct netif *netif);
err_t EthernetPort_LowLevelOutput(struct netif *netif, struct pbuf *p);

#ifdef __cplusplus
}
#endif

#endif /* ETHERNET_PORT_H */

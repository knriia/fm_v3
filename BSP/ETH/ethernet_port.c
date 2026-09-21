#include "ethernet_port.h"

#include <string.h>
#include "dp83848.h"

#define ETH_DMA_TRANSMIT_TIMEOUT (20U)

extern ETH_HandleTypeDef heth;
extern ETH_TxPacketConfig TxConfig;

static DP83848_HandleTypeDef dp83848;
static uint32_t ethernet_link_configured;
static uint32_t ethernet_link_speed;
static uint32_t ethernet_link_duplex;

extern void Error_Handler(void);

HAL_StatusTypeDef EthernetPort_Init(ETH_HandleTypeDef *heth) {
    if (heth == NULL) {
        return HAL_ERROR;
    }

    ethernet_link_configured = 0U;

    if (DP83848_Init(&dp83848, heth) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_ETH_Start(heth);
}

void EthernetPort_CheckLinkState(struct netif *netif) {
    DP83848_LinkStateTypeDef link_state;
    ETH_HandleTypeDef *heth;

    if (netif == NULL || dp83848.heth == NULL) {
        return;
    }

    heth = dp83848.heth;

    if (DP83848_GetLinkState(&dp83848, &link_state) != HAL_OK) {
        return;
    }

    if (link_state.link_up == 0U) {
        ethernet_link_configured = 0U;
        netif_set_link_down(netif);
        netif_set_down(netif);
        return;
    }

    if (ethernet_link_configured == 0U || ethernet_link_speed != link_state.speed ||
        ethernet_link_duplex != link_state.duplex) {
        ETH_MACConfigTypeDef mac_config;

        /* HAL_ETH_SetMACConfig() requires the HAL handle to be READY. */
        if (heth->gState != HAL_ETH_STATE_READY && HAL_ETH_Stop(heth) != HAL_OK) {
            Error_Handler();
            return;
        }

        if (HAL_ETH_GetMACConfig(heth, &mac_config) != HAL_OK) {
            Error_Handler();
            return;
        }

        mac_config.Speed = link_state.speed;
        mac_config.DuplexMode = link_state.duplex;

        if (HAL_ETH_SetMACConfig(heth, &mac_config) != HAL_OK) {
            Error_Handler();
            return;
        }

        if (HAL_ETH_Start(heth) != HAL_OK) {
            Error_Handler();
            return;
        }

        ethernet_link_speed = link_state.speed;
        ethernet_link_duplex = link_state.duplex;
        ethernet_link_configured = 1U;
    }

    netif_set_link_up(netif);
    netif_set_up(netif);
}

err_t EthernetPort_LowLevelOutput(struct netif *netif, struct pbuf *p) {
    struct pbuf *q;
    uint32_t i = 0U;
    ETH_BufferTypeDef tx_buffer[ETH_TX_DESC_CNT] = {0};

    (void)netif;

    if (p == NULL) {
        return ERR_ARG;
    }

    memset(tx_buffer, 0, sizeof(tx_buffer));

    for (q = p; q != NULL; q = q->next) {
        if (i >= ETH_TX_DESC_CNT) {
            return ERR_IF;
        }

        tx_buffer[i].buffer = q->payload;
        tx_buffer[i].len = q->len;

        if (i > 0U) {
            tx_buffer[i - 1U].next = &tx_buffer[i];
        }

        if (q->next == NULL) {
            tx_buffer[i].next = NULL;
        }

        SCB_CleanDCache_by_Addr((uint32_t *)q->payload, q->len);
        i++;
    }

    TxConfig.Length = p->tot_len;
    TxConfig.TxBuffer = tx_buffer;
    TxConfig.pData = p;

    if (HAL_ETH_Transmit(&heth, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT) != HAL_OK) {
        return ERR_IF;
    }

    return ERR_OK;
}

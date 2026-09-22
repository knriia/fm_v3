#include "ethernet_port.h"

#include <string.h>
#include "dp83848.h"
#include "cmsis_os.h"
#include "lwip/netifapi.h"

#define ETH_DMA_TRANSMIT_TIMEOUT (20U)
#define ETH_PORT_MUTEX_TIMEOUT (20U)
#define ETH_CACHE_LINE_SIZE (32U)

extern ETH_HandleTypeDef heth;
extern ETH_TxPacketConfig TxConfig;

static DP83848_HandleTypeDef dp83848;
static uint32_t ethernet_link_configured;
static uint32_t ethernet_link_speed;
static uint32_t ethernet_link_duplex;
static uint32_t ethernet_netif_up;
static osMutexId_t ethernet_mutex;

HAL_StatusTypeDef EthernetPort_Init(ETH_HandleTypeDef *heth) {
    if (heth == NULL) {
        return HAL_ERROR;
    }

    ethernet_link_configured = 0U;
    ethernet_netif_up = 0U;
    ethernet_mutex = osMutexNew(NULL);

    if (ethernet_mutex == NULL) {
        return HAL_ERROR;
    }

    if (DP83848_Init(&dp83848, heth) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_ETH_Start_IT(heth);
}

HAL_StatusTypeDef EthernetPort_ReadData(void **pAppBuff) {
    HAL_StatusTypeDef status;

    if (pAppBuff == NULL) {
        return HAL_ERROR;
    }

    if (ethernet_mutex == NULL || osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        return HAL_BUSY;
    }

    status = HAL_ETH_ReadData(&heth, pAppBuff);
    (void)osMutexRelease(ethernet_mutex);

    /* HAL_ETH_ReadData uses HAL_ERROR for the normal "no complete packet" case. */
    if (status == HAL_ERROR && heth.gState == HAL_ETH_STATE_STARTED) {
        return HAL_BUSY;
    }

    return status;
}

void EthernetPort_CheckLinkState(struct netif *netif) {
    DP83848_LinkStateTypeDef link_state;
    ETH_HandleTypeDef *heth;

    if (netif == NULL || dp83848.heth == NULL || ethernet_mutex == NULL) {
        return;
    }

    if (osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        return;
    }

    heth = dp83848.heth;

    if (DP83848_GetLinkState(&dp83848, &link_state) != HAL_OK) {
        (void)osMutexRelease(ethernet_mutex);
        return;
    }

    if (link_state.link_up == 0U) {
        ethernet_link_configured = 0U;
        (void)osMutexRelease(ethernet_mutex);

        if (ethernet_netif_up != 0U) {
            if ((netifapi_netif_set_down(netif) == ERR_OK) && (netifapi_netif_set_link_down(netif) == ERR_OK)) {
                ethernet_netif_up = 0U;
            }
        }
        return;
    }

    if (ethernet_link_configured == 0U || ethernet_link_speed != link_state.speed ||
        ethernet_link_duplex != link_state.duplex || heth->gState != HAL_ETH_STATE_STARTED) {
        ETH_MACConfigTypeDef mac_config;

        if (heth->gState == HAL_ETH_STATE_STARTED && HAL_ETH_Stop_IT(heth) != HAL_OK) {
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        /* HAL_ETH_SetMACConfig() requires the HAL handle to be READY. */
        if (heth->gState != HAL_ETH_STATE_READY) {
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        if (HAL_ETH_GetMACConfig(heth, &mac_config) != HAL_OK) {
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        mac_config.Speed = link_state.speed;
        mac_config.DuplexMode = link_state.duplex;

        if (HAL_ETH_SetMACConfig(heth, &mac_config) != HAL_OK) {
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        if (HAL_ETH_Start_IT(heth) != HAL_OK) {
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        ethernet_link_speed = link_state.speed;
        ethernet_link_duplex = link_state.duplex;
        ethernet_link_configured = 1U;
    }

    (void)osMutexRelease(ethernet_mutex);

    if (ethernet_netif_up == 0U) {
        if ((netifapi_netif_set_up(netif) == ERR_OK) && (netifapi_netif_set_link_up(netif) == ERR_OK)) {
            ethernet_netif_up = 1U;
        }
    }
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

        const uintptr_t payload_start = (uintptr_t)q->payload & ~(uintptr_t)(ETH_CACHE_LINE_SIZE - 1U);
        const uintptr_t payload_end =
            ((uintptr_t)q->payload + q->len + ETH_CACHE_LINE_SIZE - 1U) & ~(uintptr_t)(ETH_CACHE_LINE_SIZE - 1U);

        SCB_CleanDCache_by_Addr((uint32_t *)payload_start, (int32_t)(payload_end - payload_start));
        i++;
    }

    TxConfig.Length = p->tot_len;
    TxConfig.TxBuffer = tx_buffer;
    TxConfig.pData = p;

    if (ethernet_mutex == NULL || osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        return ERR_TIMEOUT;
    }

    const HAL_StatusTypeDef transmit_status = HAL_ETH_Transmit(&heth, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);
    (void)osMutexRelease(ethernet_mutex);

    if (transmit_status != HAL_OK) {
        return ERR_IF;
    }

    return ERR_OK;
}

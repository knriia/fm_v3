#include "ethernet_port.h"
#include "ethernetif.h"

#include <string.h>
#include "dp83848.h"
#include "cmsis_os.h"
#include "lwip/netifapi.h"

#include "stm32h723xx.h"

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
static volatile EthernetPortDiagnostics ethernet_port_diagnostics;

static void ethernet_port_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

HAL_StatusTypeDef EthernetPort_Init(ETH_HandleTypeDef *heth) {
    if (heth == NULL) {
        return HAL_ERROR;
    }

    ethernet_link_configured = 0U;
    ethernet_link_speed = 0U;
    ethernet_link_duplex = 0U;
    ethernet_netif_up = 0U;
    memset((void *)&ethernet_port_diagnostics, 0, sizeof(ethernet_port_diagnostics));
    ethernet_mutex = osMutexNew(NULL);

    if (ethernet_mutex == NULL) {
        return HAL_ERROR;
    }

    if (DP83848_Init(&dp83848, heth) != HAL_OK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.phy_init_errors);
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_ETH_Start_IT(heth);
    if (status != HAL_OK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.hal_start_errors);
    }
    return status;
}

HAL_StatusTypeDef EthernetPort_ReadData(void **pAppBuff) {
    HAL_StatusTypeDef status;

    if (pAppBuff == NULL) {
        return HAL_ERROR;
    }

    if (ethernet_mutex == NULL || osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.mutex_timeouts);
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

HAL_StatusTypeDef EthernetPort_HandleLinkInterrupt(struct netif *netif) {
    uint32_t interrupt_status = 0U;
    HAL_StatusTypeDef status;

    if (netif == NULL || dp83848.heth == NULL || ethernet_mutex == NULL) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.link_interrupt_errors);
        return HAL_ERROR;
    }

    if (osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.mutex_timeouts);
        return HAL_BUSY;
    }

    status = DP83848_ReadInterruptStatus(&dp83848, &interrupt_status);
    (void)osMutexRelease(ethernet_mutex);

    if (status != HAL_OK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.link_interrupt_errors);
        return status;
    }

    if ((interrupt_status & DP83848_MISR_EVENT_STATUS_MASK) == 0U) {
        return HAL_OK;
    }

    /* Reading MISR clears the PHY event. Read the complete link state in the
     * task context so MAC/netif changes
     * never happen inside the ISR. */
    EthernetPort_CheckLinkState(netif);
    return HAL_OK;
}

void EthernetPort_CheckLinkState(struct netif *netif) {
    DP83848_LinkStateTypeDef link_state;
    ETH_HandleTypeDef *heth;

    if (netif == NULL || dp83848.heth == NULL || ethernet_mutex == NULL) {
        return;
    }

    if (osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.mutex_timeouts);
        return;
    }

    heth = dp83848.heth;

    if (DP83848_GetLinkState(&dp83848, &link_state) != HAL_OK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.link_state_read_errors);
        (void)osMutexRelease(ethernet_mutex);
        return;
    }

    if (link_state.link_up == 0U) {
        ethernet_link_configured = 0U;
        (void)osMutexRelease(ethernet_mutex);

        if (ethernet_netif_up != 0U) {
            const err_t netif_down_status = netifapi_netif_set_down(netif);
            const err_t link_down_status = netifapi_netif_set_link_down(netif);
            if ((netif_down_status == ERR_OK) && (link_down_status == ERR_OK)) {
                ethernet_netif_up = 0U;
                ethernet_port_counter_increment(&ethernet_port_diagnostics.link_down_events);
            } else {
                ethernet_port_counter_increment(&ethernet_port_diagnostics.netif_state_errors);
            }
        }
        return;
    }

    if (ethernet_link_configured == 0U || ethernet_link_speed != link_state.speed ||
        ethernet_link_duplex != link_state.duplex || heth->gState != HAL_ETH_STATE_STARTED) {
        ETH_MACConfigTypeDef mac_config;

        if (heth->gState == HAL_ETH_STATE_STARTED && HAL_ETH_Stop_IT(heth) != HAL_OK) {
            ethernet_port_counter_increment(&ethernet_port_diagnostics.hal_stop_errors);
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        /* HAL_ETH_SetMACConfig() requires the HAL handle to be READY. */
        if (heth->gState != HAL_ETH_STATE_READY) {
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        if (HAL_ETH_GetMACConfig(heth, &mac_config) != HAL_OK) {
            ethernet_port_counter_increment(&ethernet_port_diagnostics.hal_config_errors);
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        mac_config.Speed = link_state.speed;
        mac_config.DuplexMode = link_state.duplex;

        if (HAL_ETH_SetMACConfig(heth, &mac_config) != HAL_OK) {
            ethernet_port_counter_increment(&ethernet_port_diagnostics.hal_config_errors);
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        if (HAL_ETH_Start_IT(heth) != HAL_OK) {
            ethernet_port_counter_increment(&ethernet_port_diagnostics.hal_start_errors);
            (void)osMutexRelease(ethernet_mutex);
            return;
        }

        ethernet_link_speed = link_state.speed;
        ethernet_link_duplex = link_state.duplex;
        ethernet_link_configured = 1U;
        ethernet_port_counter_increment(&ethernet_port_diagnostics.link_reconfigurations);
    }

    (void)osMutexRelease(ethernet_mutex);

    if (ethernet_netif_up == 0U) {
        const err_t netif_up_status = netifapi_netif_set_up(netif);
        const err_t link_up_status = netifapi_netif_set_link_up(netif);
        if ((netif_up_status == ERR_OK) && (link_up_status == ERR_OK)) {
            ethernet_netif_up = 1U;
            ethernet_port_counter_increment(&ethernet_port_diagnostics.link_up_events);
        } else {
            ethernet_port_counter_increment(&ethernet_port_diagnostics.netif_state_errors);
        }
    }
}

err_t EthernetPort_LowLevelOutput(struct netif *netif, struct pbuf *p) {
    struct pbuf *q;
    uint32_t i = 0U;
    ETH_BufferTypeDef tx_buffer[ETH_TX_DESC_CNT] = {0};

    (void)netif;

    ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_attempts);

    if (p == NULL) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_errors);
        return ERR_ARG;
    }

    memset(tx_buffer, 0, sizeof(tx_buffer));

    for (q = p; q != NULL; q = q->next) {
        if (i >= ETH_TX_DESC_CNT) {
            ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_descriptor_overflows);
            ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_errors);
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
        ethernet_port_counter_increment(&ethernet_port_diagnostics.mutex_timeouts);
        ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_errors);
        return ERR_TIMEOUT;
    }

    const HAL_StatusTypeDef transmit_status = HAL_ETH_Transmit(&heth, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);
    (void)osMutexRelease(ethernet_mutex);

    if (transmit_status != HAL_OK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_errors);
        return ERR_IF;
    }

    ethernet_port_counter_increment(&ethernet_port_diagnostics.tx_successes);
    ethernet_port_diagnostics.tx_bytes += p->tot_len;
    return ERR_OK;
}

HAL_StatusTypeDef EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics) {
    uint32_t phy_id1 = 0U;
    uint32_t phy_id2 = 0U;

    if (diagnostics == NULL || dp83848.heth == NULL || ethernet_mutex == NULL) {
        return HAL_ERROR;
    }

    if (osMutexAcquire(ethernet_mutex, ETH_PORT_MUTEX_TIMEOUT) != osOK) {
        ethernet_port_counter_increment(&ethernet_port_diagnostics.mutex_timeouts);
        return HAL_BUSY;
    }

    memcpy(diagnostics, (const void *)&ethernet_port_diagnostics, sizeof(*diagnostics));
    diagnostics->link_up = ethernet_link_configured != 0U;
    diagnostics->speed = ethernet_link_speed;
    diagnostics->duplex = ethernet_link_duplex;
    diagnostics->netif_up = ethernet_netif_up;
    diagnostics->hal_state = (uint32_t)HAL_ETH_GetState(dp83848.heth);
    diagnostics->hal_error = HAL_ETH_GetError(dp83848.heth);
    diagnostics->dma_error = HAL_ETH_GetDMAError(dp83848.heth);
    diagnostics->dma_status = dp83848.heth->Instance->DMACSR;
    diagnostics->dma_debug_status = dp83848.heth->Instance->DMADSR;
    diagnostics->mac_control = dp83848.heth->Instance->MACCR;
    diagnostics->phy_address = dp83848.address;
    diagnostics->rx_descriptor_address = (uint32_t)(uintptr_t)dp83848.heth->Init.RxDesc;
    diagnostics->tx_descriptor_address = (uint32_t)(uintptr_t)dp83848.heth->Init.TxDesc;
    diagnostics->rx_descriptor_base = dp83848.heth->Instance->DMACRDLAR;
    diagnostics->tx_descriptor_base = dp83848.heth->Instance->DMACTDLAR;
    diagnostics->rx_descriptor_count = ETH_RX_DESC_CNT;
    diagnostics->tx_descriptor_count = ETH_TX_DESC_CNT;
    diagnostics->rx_descriptor_length = dp83848.heth->Instance->DMACRDRLR;
    diagnostics->tx_descriptor_length = dp83848.heth->Instance->DMACTDRLR;
    diagnostics->rx_buffer_size = dp83848.heth->Init.RxBuffLen;
    diagnostics->rx_buffer_count = ETH_RX_BUFFER_CNT;
    diagnostics->rx_descriptor_current = dp83848.heth->Instance->DMACCARDR;
    diagnostics->tx_descriptor_current = dp83848.heth->Instance->DMACCATDR;
    diagnostics->rx_descriptor_tail = dp83848.heth->Instance->DMACRDTPR;
    diagnostics->tx_descriptor_tail = dp83848.heth->Instance->DMACTDTPR;
    (void)DP83848_GetIdentity(&dp83848, &phy_id1, &phy_id2);
    diagnostics->phy_id1 = phy_id1;
    diagnostics->phy_id2 = phy_id2;
    if (dp83848.heth->Init.MACAddr != NULL) {
        memcpy(diagnostics->mac_address, dp83848.heth->Init.MACAddr, sizeof(diagnostics->mac_address));
    }

    (void)osMutexRelease(ethernet_mutex);
    return HAL_OK;
}

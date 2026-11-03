/*
 * Minimal DP83848 PHY adapter for STM32H7 HAL.
 *
 * The STM32 HAL owns the MAC/DMA. This module owns only the external PHY
 * management interface (MDIO/MDC) and link negotiation/status.
 */
#ifndef DP83848_H
#define DP83848_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DP83848 PHY address selected by the PHYAD strap pins. */
#ifndef DP83848_PHY_ADDRESS
#define DP83848_PHY_ADDRESS 0x01U
#endif

#define DP83848_BCR 0x00U
#define DP83848_BSR 0x01U
#define DP83848_PHYID1 0x02U
#define DP83848_PHYID2 0x03U
#define DP83848_PHYSCSR 0x10U
#define DP83848_MICR 0x11U
#define DP83848_MISR 0x12U
#define DP83848_SMR 0x19U

#define DP83848_BCR_RESET 0x8000U
#define DP83848_BCR_AUTONEG 0x1000U
#define DP83848_BCR_RESTART 0x0200U
#define DP83848_BCR_SPEED100 0x2000U
#define DP83848_BCR_FULLDUPLEX 0x0100U

#define DP83848_BSR_LINK 0x0004U

#define DP83848_PHYSCSR_AUTONEG_DONE 0x0100U
#define DP83848_PHYSCSR_SPEED_DUPLEX 0x0006U
#define DP83848_PHYSCSR_100_FULL 0x0004U
#define DP83848_PHYSCSR_10_FULL 0x0006U

#define DP83848_MICR_INT_OE 0x0001U
#define DP83848_MICR_INTEN 0x0002U
#define DP83848_MISR_LINK_INT 0x2000U
#define DP83848_MISR_SPEED_INT 0x1000U
#define DP83848_MISR_DUPLEX_INT 0x0800U
#define DP83848_MISR_AUTONEG_COMPLETE_INT 0x0400U
#define DP83848_MISR_EVENT_STATUS_MASK                                                                                 \
    (DP83848_MISR_LINK_INT | DP83848_MISR_SPEED_INT | DP83848_MISR_DUPLEX_INT | DP83848_MISR_AUTONEG_COMPLETE_INT)
#define DP83848_MISR_LINK_INT_ENABLE 0x0020U
#define DP83848_MISR_SPEED_INT_ENABLE 0x0010U
#define DP83848_MISR_DUPLEX_INT_ENABLE 0x0008U
#define DP83848_MISR_AUTONEG_COMPLETE_INT_ENABLE 0x0004U
#define DP83848_MISR_EVENT_ENABLE_MASK                                                                                 \
    (DP83848_MISR_LINK_INT_ENABLE | DP83848_MISR_SPEED_INT_ENABLE | DP83848_MISR_DUPLEX_INT_ENABLE |                   \
        DP83848_MISR_AUTONEG_COMPLETE_INT_ENABLE)

#define DP83848_INIT_TIMEOUT_MS 500U

typedef struct {
    ETH_HandleTypeDef *heth;
    uint32_t address;
} DP83848_HandleTypeDef;

typedef struct {
    uint32_t link_up;
    uint32_t speed;
    uint32_t duplex;
} DP83848_LinkStateTypeDef;


HAL_StatusTypeDef DP83848_Init(DP83848_HandleTypeDef *phy, ETH_HandleTypeDef *heth);
HAL_StatusTypeDef DP83848_GetLinkState(const DP83848_HandleTypeDef *phy, DP83848_LinkStateTypeDef *state);
HAL_StatusTypeDef DP83848_ReadInterruptStatus(const DP83848_HandleTypeDef *phy, uint32_t *status);

#ifdef __cplusplus
}
#endif

#endif /* DP83848_H */

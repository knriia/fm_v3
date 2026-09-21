#include "dp83848.h"

static HAL_StatusTypeDef dp83848_read(const DP83848_HandleTypeDef *phy, uint32_t reg, uint32_t *value) {
    return HAL_ETH_ReadPHYRegister(phy->heth, phy->address, reg, value);
}

static HAL_StatusTypeDef dp83848_write(const DP83848_HandleTypeDef *phy, uint32_t reg, uint32_t value) {
    return HAL_ETH_WritePHYRegister(phy->heth, phy->address, reg, value);
}

static HAL_StatusTypeDef dp83848_wait_reset(const DP83848_HandleTypeDef *phy) {
    const uint32_t start = HAL_GetTick();
    uint32_t bcr = 0U;

    do {
        if (dp83848_read(phy, DP83848_BCR, &bcr) != HAL_OK) {
            return HAL_ERROR;
        }

        if ((bcr & DP83848_BCR_RESET) == 0U) {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start) < DP83848_INIT_TIMEOUT_MS);

    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef dp83848_find_address(DP83848_HandleTypeDef *phy) {
    uint32_t address;
    uint32_t smr = 0U;
    uint32_t id1 = 0U;
    uint32_t id2 = 0U;

    if (DP83848_PHY_ADDRESS <= 31U) {
        phy->address = DP83848_PHY_ADDRESS;
        if (dp83848_read(phy, DP83848_PHYID1, &id1) != HAL_OK || dp83848_read(phy, DP83848_PHYID2, &id2) != HAL_OK ||
            (id1 == 0U) || (id1 == 0xFFFFU) || (id2 == 0U) || (id2 == 0xFFFFU)) {
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    for (address = 0U; address <= 31U; ++address) {
        phy->address = address;

        if (dp83848_read(phy, DP83848_SMR, &smr) != HAL_OK || (smr & 0x001FU) != address) {
            continue;
        }

        if (dp83848_read(phy, DP83848_PHYID1, &id1) == HAL_OK && dp83848_read(phy, DP83848_PHYID2, &id2) == HAL_OK &&
            id1 != 0U && id1 != 0xFFFFU && id2 != 0U && id2 != 0xFFFFU) {
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef DP83848_Init(DP83848_HandleTypeDef *phy, ETH_HandleTypeDef *heth) {
    uint32_t bcr = 0U;

    if (phy == NULL || heth == NULL) {
        return HAL_ERROR;
    }

    phy->heth = heth;

    if (dp83848_find_address(phy) != HAL_OK) {
        return HAL_ERROR;
    }

    if (dp83848_write(phy, DP83848_BCR, DP83848_BCR_RESET) != HAL_OK || dp83848_wait_reset(phy) != HAL_OK ||
        dp83848_read(phy, DP83848_BCR, &bcr) != HAL_OK) {
        return HAL_ERROR;
    }

    bcr &= ~(DP83848_BCR_SPEED100 | DP83848_BCR_FULLDUPLEX);
    bcr |= DP83848_BCR_AUTONEG | DP83848_BCR_RESTART;

    return dp83848_write(phy, DP83848_BCR, bcr);
}

HAL_StatusTypeDef DP83848_GetLinkState(const DP83848_HandleTypeDef *phy, DP83848_LinkStateTypeDef *state) {
    uint32_t bsr = 0U;
    uint32_t bcr = 0U;
    uint32_t physcsr = 0U;

    if (phy == NULL || state == NULL) {
        return HAL_ERROR;
    }

    state->link_up = 0U;
    state->speed = ETH_SPEED_10M;
    state->duplex = ETH_HALFDUPLEX_MODE;

    /* BSR link status is latch-low; read it twice. */
    if (dp83848_read(phy, DP83848_BSR, &bsr) != HAL_OK || dp83848_read(phy, DP83848_BSR, &bsr) != HAL_OK) {
        return HAL_ERROR;
    }

    if ((bsr & DP83848_BSR_LINK) == 0U) {
        return HAL_OK;
    }

    if (dp83848_read(phy, DP83848_BCR, &bcr) != HAL_OK) {
        return HAL_ERROR;
    }

    if ((bcr & DP83848_BCR_AUTONEG) != 0U) {
        if (dp83848_read(phy, DP83848_PHYSCSR, &physcsr) != HAL_OK) {
            return HAL_ERROR;
        }

        if ((physcsr & DP83848_PHYSCSR_AUTONEG_DONE) == 0U) {
            return HAL_OK;
        }

        switch (physcsr & DP83848_PHYSCSR_SPEED_DUPLEX) {
        case DP83848_PHYSCSR_100_FULL:
            state->speed = ETH_SPEED_100M;
            state->duplex = ETH_FULLDUPLEX_MODE;
            break;
        case 0x0000U:
            state->speed = ETH_SPEED_100M;
            state->duplex = ETH_HALFDUPLEX_MODE;
            break;
        case DP83848_PHYSCSR_10_FULL:
            state->speed = ETH_SPEED_10M;
            state->duplex = ETH_FULLDUPLEX_MODE;
            break;
        default:
            state->speed = ETH_SPEED_10M;
            state->duplex = ETH_HALFDUPLEX_MODE;
            break;
        }
    } else {
        state->speed = (bcr & DP83848_BCR_SPEED100) != 0U ? ETH_SPEED_100M : ETH_SPEED_10M;
        state->duplex = (bcr & DP83848_BCR_FULLDUPLEX) != 0U ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
    }

    state->link_up = 1U;
    return HAL_OK;
}

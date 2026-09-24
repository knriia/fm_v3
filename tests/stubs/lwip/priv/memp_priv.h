#ifndef FM_V3_TEST_LWIP_MEMP_PRIV_H
#define FM_V3_TEST_LWIP_MEMP_PRIV_H

#include "lwip/memp.h"

#include <stdint.h>

struct memp_desc {
    uint16_t num;
    uint16_t size;
};

extern const struct memp_desc *memp_pools[MEMP_MAX];

#endif /* FM_V3_TEST_LWIP_MEMP_PRIV_H */

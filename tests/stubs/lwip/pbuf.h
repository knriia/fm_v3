#ifndef FM_V3_TEST_LWIP_PBUF_H
#define FM_V3_TEST_LWIP_PBUF_H

#include "lwip/arch.h"

struct pbuf {
    struct pbuf *next;
    void *payload;
    u16_t tot_len;
    u16_t len;
};

u8_t pbuf_free(struct pbuf *buffer);

#endif /* FM_V3_TEST_LWIP_PBUF_H */

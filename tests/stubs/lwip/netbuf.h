#ifndef FM_V3_TEST_LWIP_NETBUF_H
#define FM_V3_TEST_LWIP_NETBUF_H

#include "lwip/err.h"

#include <stdint.h>

typedef uint16_t u16_t;

struct netbuf;

void netbuf_first(struct netbuf *buffer);
int8_t netbuf_next(struct netbuf *buffer);
err_t netbuf_data(struct netbuf *buffer, void **data, u16_t *length);
void netbuf_delete(struct netbuf *buffer);

#endif /* FM_V3_TEST_LWIP_NETBUF_H */
#ifndef FM_V3_TEST_LWIP_API_H
#define FM_V3_TEST_LWIP_API_H

#include "lwip/err.h"

#include <stddef.h>

struct netconn;
struct netbuf;

#define NETCONN_TCP 1
#define NETCONN_COPY 1
#define IP_ADDR_ANY ((void *)0)

struct netconn *netconn_new(int type);
err_t netconn_bind(struct netconn *connection, void *address, uint16_t port);
err_t netconn_listen(struct netconn *connection);
err_t netconn_accept(struct netconn *connection, struct netconn **new_connection);
err_t netconn_recv(struct netconn *connection, struct netbuf **buffer);
void netconn_set_sendtimeout(struct netconn *connection, int timeout);
err_t netconn_write_partly(
    struct netconn *connection,
    const void *data,
    size_t size,
    int apiflags,
    size_t *bytes_written
);
err_t netconn_close(struct netconn *connection);
void netconn_delete(struct netconn *connection);

#endif /* FM_V3_TEST_LWIP_API_H */

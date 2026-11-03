#ifndef FM_V3_TEST_LWIP_API_H
#define FM_V3_TEST_LWIP_API_H

#include "lwip/arch.h"
#include "lwip/err.h"

#include <stddef.h>

struct netconn;
struct netbuf;
struct pbuf;

#define NETCONN_TCP 1
#define NETCONN_COPY 1
#define NETCONN_DONTBLOCK 0x04
#define IP_ADDR_ANY ((void *)0)

enum netconn_evt {
    NETCONN_EVT_RCVPLUS,
    NETCONN_EVT_RCVMINUS,
    NETCONN_EVT_SENDPLUS,
    NETCONN_EVT_SENDMINUS,
    NETCONN_EVT_ERROR
};

typedef void (*netconn_callback)(struct netconn *connection, enum netconn_evt event, u16_t length);

struct netconn *netconn_new(int type);
struct netconn *netconn_new_with_callback(int type, netconn_callback callback);
err_t netconn_recv_tcp_pbuf_flags(struct netconn *connection, struct pbuf **buffer, u8_t apiflags);
err_t netconn_bind(struct netconn *connection, void *address, uint16_t port);
err_t netconn_listen(struct netconn *connection);
err_t netconn_accept(struct netconn *connection, struct netconn **new_connection);
err_t netconn_recv(struct netconn *connection, struct netbuf **buffer);
void netconn_set_sendtimeout(struct netconn *connection, int timeout);
void netconn_set_recvtimeout(struct netconn *connection, int timeout);
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

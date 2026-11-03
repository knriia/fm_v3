#ifndef FM_V3_TEST_LWIP_MEMP_H
#define FM_V3_TEST_LWIP_MEMP_H

/* The pools the diagnostics read by name, plus two unnamed ones; the real order depends on lwipopts.h. */
typedef enum {
    MEMP_UDP_PCB = 0,
    MEMP_TCP_PCB,
    MEMP_TCP_PCB_LISTEN,
    MEMP_TCP_SEG,
    MEMP_NETBUF,
    MEMP_NETCONN,
    MEMP_TCPIP_MSG_API,
    MEMP_TCPIP_MSG_INPKT,
    MEMP_PBUF_POOL,
    MEMP_MAX
} memp_t;

#endif /* FM_V3_TEST_LWIP_MEMP_H */

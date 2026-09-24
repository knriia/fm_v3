#ifndef FM_V3_TEST_LWIP_H
#define FM_V3_TEST_LWIP_H

#include "lwip/netif.h"

#define TCPIP_THREAD_STACKSIZE 1024U
#define TCPIP_THREAD_PRIO 24U
#define TCPIP_MBOX_SIZE 6U
#define TCP_MSS 1460U
#define TCP_SND_BUF 5840U
#define TCP_WND 5840U
#define TCP_SND_QUEUELEN 16U
#define PBUF_POOL_SIZE 16U
#define PBUF_POOL_BUFSIZE 1536U

void MX_LWIP_Init(void);

#endif /* FM_V3_TEST_LWIP_H */

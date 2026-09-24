/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Target/lwipopts.h
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "main.h"
#include "task_names.h"

/*-----------------------------------------------------------------------------*/
/* Current version of LwIP supported by CubeMx: 2.1.2 -*/
/*-----------------------------------------------------------------------------*/

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32CubeMX Specific Parameters (not defined in opt.h) ---------------------*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- WITH_RTOS enabled (Since FREERTOS is set) -----*/
#define WITH_RTOS 1
#define TCPIP_THREAD_NAME TCPIP_TASK_NAME
/* Temporary workaround to avoid conflict on errno defined in STM32CubeIDE and lwip sys_arch.c errno */
#undef LWIP_PROVIDE_ERRNO
/*----- CHECKSUM_BY_HARDWARE enabled -----*/
#define CHECKSUM_BY_HARDWARE 1
/*-----------------------------------------------------------------------------*/

/* LwIP Stack Parameters (modified compared to initialization value in opt.h) -*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- Default value in ETH configuration GUI in CubeMx: 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536
/*----- Default Value for LWIP_UDP: 1 ---*/
#define LWIP_UDP 0
/*----- Value in opt.h for MEM_ALIGNMENT: 1 -----*/
#define MEM_ALIGNMENT 4
/*----- Default Value for MEM_SIZE: 1600 ---*/
#define MEM_SIZE 8192
/*----- LwIP heap is placed by the linker in the dedicated D2 section. -----*/
/*----- Default Value for MEMP_NUM_TCP_SEG: 16 ---*/
#define MEMP_NUM_TCP_SEG 32
/*----- Default Value for MEMP_NUM_SYS_TIMEOUT: 3 ---*/
#define MEMP_NUM_SYS_TIMEOUT 8
/*----- Default Value for MEMP_NUM_NETCONN: 4 ---*/
#define MEMP_NUM_NETCONN 8
/*----- Value supported for H7 devices: 1 -----*/
#define LWIP_SUPPORT_CUSTOM_PBUF 1
/*----- Default Value for PBUF_POOL_BUFSIZE: 592 ---*/
#define PBUF_POOL_BUFSIZE 1536
/*----- Value in opt.h for LWIP_ETHERNET: LWIP_ARP || PPPOE_SUPPORT -*/
#define LWIP_ETHERNET 1
/*----- Value in opt.h for LWIP_DNS_SECURE: (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING | LWIP_DNS_SECURE_RAND_SRC_PORT) -*/
#define LWIP_DNS_SECURE 7
/*----- Default Value for TCP_MSS: 536 ---*/
#define TCP_MSS 1460
/*----- Default Value for TCP_SND_BUF: 2920 ---*/
#define TCP_SND_BUF 5840
/*----- Default Value for LWIP_NETIF_API: 0 ---*/
#define LWIP_NETIF_API 1
/*----- Value in opt.h for LWIP_NETIF_LINK_CALLBACK: 0 -----*/
#define LWIP_NETIF_LINK_CALLBACK 1
/*----- Value in opt.h for TCPIP_THREAD_STACKSIZE: 0 -----*/
#define TCPIP_THREAD_STACKSIZE 1024
/*----- Value in opt.h for TCPIP_THREAD_PRIO: 1 -----*/
#define TCPIP_THREAD_PRIO 24
/*----- Value in opt.h for TCPIP_MBOX_SIZE: 0 -----*/
#define TCPIP_MBOX_SIZE 6
/*----- Value in opt.h for SLIPIF_THREAD_STACKSIZE: 0 -----*/
#define SLIPIF_THREAD_STACKSIZE 1024
/*----- Value in opt.h for SLIPIF_THREAD_PRIO: 1 -----*/
#define SLIPIF_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_THREAD_STACKSIZE: 0 -----*/
#define DEFAULT_THREAD_STACKSIZE 1024
/*----- Value in opt.h for DEFAULT_THREAD_PRIO: 1 -----*/
#define DEFAULT_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_TCP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_TCP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_ACCEPTMBOX_SIZE: 0 -----*/
#define DEFAULT_ACCEPTMBOX_SIZE 6
/*----- Default Value for LWIP_SO_SNDTIMEO: 0 ---*/
#define LWIP_SO_SNDTIMEO 1
/*----- Value in opt.h for RECV_BUFSIZE_DEFAULT: INT_MAX -----*/
#define RECV_BUFSIZE_DEFAULT 2000000000
/*----- Default Value for LWIP_STATS: 0 ---*/
#define LWIP_STATS 1
/* Keep diagnostic counters usable during long-running traffic tests. */
#define LWIP_STATS_LARGE 1
/* Collect all statistics for the protocols and resources enabled below. */
#define LINK_STATS 1
#define ETHARP_STATS 1
#define IP_STATS 1
#define ICMP_STATS 1
#define TCP_STATS 1
#define MEM_STATS 1
#define MEMP_STATS 1
#define SYS_STATS 1
/* Collect interface-level packet and byte counters. */
#define MIB2_STATS 1
/*----- Value in opt.h for MIB2_STATS: 0 or SNMP_LWIP_MIB2 -----*/
/*----- Value in opt.h for CHECKSUM_GEN_IP: 1 -----*/
#define CHECKSUM_GEN_IP 0
/*----- Value in opt.h for CHECKSUM_GEN_UDP: 1 -----*/
#define CHECKSUM_GEN_UDP 0
/*----- Value in opt.h for CHECKSUM_GEN_TCP: 1 -----*/
#define CHECKSUM_GEN_TCP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP6: 1 -----*/
#define CHECKSUM_GEN_ICMP6 0
/*----- Value in opt.h for CHECKSUM_CHECK_IP: 1 -----*/
#define CHECKSUM_CHECK_IP 0
/*----- Value in opt.h for CHECKSUM_CHECK_UDP: 1 -----*/
#define CHECKSUM_CHECK_UDP 0
/*----- Value in opt.h for CHECKSUM_CHECK_TCP: 1 -----*/
#define CHECKSUM_CHECK_TCP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP6: 1 -----*/
#define CHECKSUM_CHECK_ICMP6 0
/*-----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/*
 * Ethernet memory placement contract:
 * - DMA descriptors and the LwIP heap are in RAM_D2. MPU marks RAM_D2 as
 *   non-cacheable because the ETH DMA accesses these areas directly.
 * - LwIP pools, including the zero-copy RX pool and TX pbufs, are in RAM_D1.
 *   RX DMA writes are followed by D-cache invalidation in ethernetif.c;
 *   TX DMA reads are preceded by D-cache cleaning in ethernet_port.c.
 */
extern unsigned char __lwip_heap_start__;
#define LWIP_RAM_HEAP_POINTER ((void *)&__lwip_heap_start__)

/* Keep lwIP memory pools in DMA-accessible D1 and aligned to 32 bytes. */
#undef LWIP_DECLARE_MEMORY_ALIGNED
#define LWIP_DECLARE_MEMORY_ALIGNED(variable_name, size) \
  u8_t variable_name[size] __attribute__((section(".LwIPPool"), aligned(32)))

/* ARP IPv4 fields are packed and may be only 16-bit aligned. */
#undef IPADDR_WORDALIGNED_COPY_TO_IP4_ADDR_T
#define IPADDR_WORDALIGNED_COPY_TO_IP4_ADDR_T(dest, src) do { \
  volatile unsigned char *d_ = (volatile unsigned char *)(dest); \
  const volatile unsigned char *s_ = (const volatile unsigned char *)(src); \
  d_[0] = s_[0]; d_[1] = s_[1]; d_[2] = s_[2]; d_[3] = s_[3]; \
} while (0)

#undef IPADDR_WORDALIGNED_COPY_FROM_IP4_ADDR_T
#define IPADDR_WORDALIGNED_COPY_FROM_IP4_ADDR_T(dest, src) do { \
  volatile unsigned char *d_ = (volatile unsigned char *)(dest); \
  const volatile unsigned char *s_ = (const volatile unsigned char *)(src); \
  d_[0] = s_[0]; d_[1] = s_[1]; d_[2] = s_[2]; d_[3] = s_[3]; \
} while (0)

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__LWIPOPTS__H__ */

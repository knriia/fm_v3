#ifndef FM_V3_TEST_ETHERNET_PORT_H
#define FM_V3_TEST_ETHERNET_PORT_H

#include <stdint.h>

typedef struct {
    uint32_t values[43];
    uint8_t mac_address[6];
} EthernetPortDiagnostics;

typedef int HAL_StatusTypeDef;

#define HAL_OK 0

HAL_StatusTypeDef EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics);

#endif /* FM_V3_TEST_ETHERNET_PORT_H */

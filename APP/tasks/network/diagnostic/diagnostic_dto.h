#ifndef FM_V3_DIAGNOSTIC_DTO_H
#define FM_V3_DIAGNOSTIC_DTO_H

#include "dto.h"
#include "ethernet_port.h"
#include "ethernetif.h"
#include "lwip_diagnostics.h"
#include "system_diagnostics.h"

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t stack_size_bytes;
    uint32_t stack_free_bytes;
    uint32_t stack_min_free_bytes;
    int32_t priority;
    uint32_t state;
    uint32_t port;
    uint32_t interval_ms;
    uint32_t send_timeout_ms;
    uint32_t connections_accepted;
    uint32_t connections_closed;
    uint32_t active_connection;
    uint32_t send_attempts;
    uint32_t send_successes;
    uint32_t send_errors;
    uint32_t partial_writes;
    uint32_t bytes_sent;
    uint32_t netconn_alloc_errors;
    uint32_t bind_errors;
    uint32_t listen_errors;
    uint32_t accept_errors;
    uint32_t snapshot_errors;
    int32_t last_error;
} DiagnosticTaskDiagnosticsDTO_t;

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    SystemDiagnostics system;
    EthernetRxDiagnostics ethernet_rx;
    EthernetPortDiagnostics ethernet_port;
    LwipDiagnostics lwip;
    DiagnosticTaskDiagnosticsDTO_t task;
} DiagnosticPayloadDTO_t;

typedef struct __attribute__((packed)) {
    NetworkFrameHeaderDTO_t header;
    DiagnosticPayloadDTO_t payload;
} DiagnosticFrameDTO_t;

typedef DiagnosticPayloadDTO_t DiagnosticDTO_t;

_Static_assert(sizeof(DiagnosticPayloadDTO_t) <= UINT16_MAX, "Diagnostic payload exceeds protocol limit");

#endif /* FM_V3_DIAGNOSTIC_DTO_H */

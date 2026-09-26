#ifndef FM_V3_DIAGNOSTIC_DTO_H
#define FM_V3_DIAGNOSTIC_DTO_H

#include "command_task.h"
#include "dto.h"
#include "ethernet_port.h"
#include "ethernetif.h"
#include "lwip_diagnostics.h"
#include "startup_task.h"
#include "system_diagnostics.h"
#include "telemetry_task.h"

#include <stdint.h>

typedef struct {
    uint32_t stack_size_bytes;
    uint32_t stack_free_bytes;
    uint32_t stack_min_free_bytes;
    uint32_t stack_base_address;
    int32_t priority;
    int32_t base_priority;
    uint32_t state;
    uint32_t runtime_ticks;
    uint32_t runtime_percent;
} TaskDiagnosticsDTO_t;

typedef struct {
    TaskDiagnosticsDTO_t runtime;
    EthernetRxDiagnostics ethernet_rx;
} EthIfTaskDiagnosticsDTO_t;

typedef struct {
    TaskDiagnosticsDTO_t runtime;
    EthernetPortDiagnostics ethernet_port;
} EthLinkTaskDiagnosticsDTO_t;

typedef struct {
    TaskDiagnosticsDTO_t runtime;
    LwipDiagnostics lwip;
} TcpipTaskDiagnosticsDTO_t;

typedef struct {
    TaskDiagnosticsDTO_t runtime;
    StartupTaskDiagnostics startup;
} StartupTaskDiagnosticsDTO_t;
typedef struct {
    TaskDiagnosticsDTO_t runtime;
    CommandTaskDiagnostics command;
} CommandTaskDiagnosticsDTO_t;

typedef struct {
    TaskDiagnosticsDTO_t runtime;
    TelemetryTaskDiagnostics telemetry;
} TelemetryTaskDiagnosticsDTO_t;

typedef struct {
    TaskDiagnosticsDTO_t runtime;
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

typedef struct {
    uint32_t sequence;
    SystemDiagnostics system;
    StartupTaskDiagnosticsDTO_t startup_task;
    CommandTaskDiagnosticsDTO_t command_task;
    TelemetryTaskDiagnosticsDTO_t telemetry_task;
    DiagnosticTaskDiagnosticsDTO_t diagnostic_task;
    EthIfTaskDiagnosticsDTO_t eth_if;
    EthLinkTaskDiagnosticsDTO_t eth_link;
    TcpipTaskDiagnosticsDTO_t tcpip_thread;
} DiagnosticPayloadDTO_t;

typedef struct {
    NetworkFrameHeaderDTO_t header;
    DiagnosticPayloadDTO_t payload;
} DiagnosticFrameDTO_t;

typedef DiagnosticPayloadDTO_t DiagnosticDTO_t;

_Static_assert(sizeof(DiagnosticPayloadDTO_t) <= UINT16_MAX, "Diagnostic payload exceeds protocol limit");
_Static_assert(sizeof(TaskDiagnosticsDTO_t) == 36U, "Invalid common task diagnostics size");
_Static_assert(sizeof(EthIfTaskDiagnosticsDTO_t) == 100U, "Invalid EthIf diagnostics size");
_Static_assert(sizeof(EthLinkTaskDiagnosticsDTO_t) == 216U, "Invalid EthLink diagnostics size");
_Static_assert(sizeof(MemoryRegionDiagnostics) == 20U, "Invalid memory region diagnostics size");
_Static_assert(sizeof(MemoryDiagnostics) == 100U, "Invalid memory diagnostics size");
_Static_assert(sizeof(FreeRtosDiagnostics) == 64U, "Invalid FreeRTOS diagnostics size");
_Static_assert(sizeof(SystemDiagnostics) == 320U, "Invalid system diagnostics size");
_Static_assert(sizeof(LwipMemoryPoolDiagnostics) == 24U, "Invalid LwIP memory pool diagnostics size");
_Static_assert(sizeof(LwipMib2Diagnostics) == 192U, "Invalid LwIP MIB2 diagnostics size");
_Static_assert(sizeof(LwipDiagnostics) == 916U, "Invalid LwIP diagnostics size");
_Static_assert(sizeof(TcpipTaskDiagnosticsDTO_t) == 952U, "Invalid tcpip diagnostics size");
_Static_assert(sizeof(StartupTaskDiagnosticsDTO_t) == 48U, "Invalid StartupTask diagnostics size");
_Static_assert(sizeof(CommandTaskDiagnosticsDTO_t) == 136U, "Invalid CommandTask diagnostics size");
_Static_assert(sizeof(TelemetryTaskDiagnosticsDTO_t) == 100U, "Invalid TelemetryTask diagnostics size");
_Static_assert(sizeof(DiagnosticTaskDiagnosticsDTO_t) == 104U, "Invalid DiagnosticTask diagnostics size");
_Static_assert(sizeof(DiagnosticPayloadDTO_t) == 1980U, "Invalid diagnostic payload size");
_Static_assert(sizeof(DiagnosticFrameDTO_t) == 1988U, "Invalid diagnostic frame size");

#endif /* FM_V3_DIAGNOSTIC_DTO_H */

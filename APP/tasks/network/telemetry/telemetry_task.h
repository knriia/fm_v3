#ifndef FM_V3_TELEMETRY_TASK_H
#define FM_V3_TELEMETRY_TASK_H

#include <stdint.h>

#define TELEMETRY_NETWORK_TASK_PORT 2328
#define TELEMETRY_NETWORK_TASK_INTERVAL_MS 100
#define TELEMETRY_NETWORK_SEND_TIMEOUT_MS 500
#define TELEMETRY_TASK_STACK_SIZE_BYTES (1024U * 4U)

typedef struct {
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
    int32_t last_error;
} TelemetryTaskDiagnostics;

void TelemetryTask(void *argument);
void telemetry_task_get_diagnostics(TelemetryTaskDiagnostics *diagnostics);

#endif /* FM_V3_TELEMETRY_TASK_H */

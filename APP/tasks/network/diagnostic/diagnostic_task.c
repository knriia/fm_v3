//
// Created by umnik on 23.09.2026.
//

#include "diagnostic_task.h"
#include "diagnostic_dto.h"
#include "network_events.h"
#include "task_context.h"

#include "cmsis_os.h"
#include "ethernetif.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "task.h"

#include <stddef.h>

typedef struct {
    uint32_t stack_min_free_bytes;
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
} DiagnosticTaskRuntime;

static void diagnostic_counter_increment(uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void diagnostic_counter_add(uint32_t *counter, size_t value) {
    if (value > UINT32_MAX || UINT32_MAX - *counter < (uint32_t)value) {
        *counter = UINT32_MAX;
    } else {
        *counter += (uint32_t)value;
    }
}

static void diagnostic_record_error(DiagnosticTaskRuntime *runtime, err_t error) {
    runtime->last_error = (int32_t)error;
}

static void diagnostic_task_fill_stats(DiagnosticPayloadDTO_t *payload, DiagnosticTaskRuntime *runtime) {
    const uint32_t stack_free_bytes = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

    if ((runtime->stack_min_free_bytes == 0U) || (stack_free_bytes < runtime->stack_min_free_bytes)) {
        runtime->stack_min_free_bytes = stack_free_bytes;
    }

    payload->task.stack_size_bytes = DIAGNOSTIC_TASK_STACK_SIZE_BYTES;
    payload->task.stack_free_bytes = stack_free_bytes;
    payload->task.stack_min_free_bytes = runtime->stack_min_free_bytes;
    payload->task.priority = (int32_t)osThreadGetPriority(osThreadGetId());
    payload->task.state = (uint32_t)osThreadGetState(osThreadGetId());
    payload->task.port = DIAGNOSTIC_NETWORK_TASK_PORT;
    payload->task.interval_ms = DIAGNOSTIC_NETWORK_TASK_INTERVAL_MS;
    payload->task.send_timeout_ms = DIAGNOSTIC_NETWORK_SEND_TIMEOUT_MS;
    payload->task.connections_accepted = runtime->connections_accepted;
    payload->task.connections_closed = runtime->connections_closed;
    payload->task.active_connection = runtime->active_connection;
    payload->task.send_attempts = runtime->send_attempts;
    payload->task.send_successes = runtime->send_successes;
    payload->task.send_errors = runtime->send_errors;
    payload->task.partial_writes = runtime->partial_writes;
    payload->task.bytes_sent = runtime->bytes_sent;
    payload->task.netconn_alloc_errors = runtime->netconn_alloc_errors;
    payload->task.bind_errors = runtime->bind_errors;
    payload->task.listen_errors = runtime->listen_errors;
    payload->task.accept_errors = runtime->accept_errors;
    payload->task.snapshot_errors = runtime->snapshot_errors;
    payload->task.last_error = runtime->last_error;
}

void DiagnosticTask(void *argument) {
    NetworkTaskContext *context = argument;
    EthernetRxDiagnostics ethernet_rx_diagnostics = {0};
    EthernetPortDiagnostics ethernet_port_diagnostics = {0};
    SystemDiagnostics system_diagnostics = {0};
    LwipDiagnostics lwip_diagnostics = {0};
    DiagnosticTaskRuntime runtime = {0};

    if ((context == NULL) || (context->lwip_flags == NULL)) {
        Error_Handler();
    }

    osEventFlagsId_t lwip_flags = context->lwip_flags;
    if (osEventFlagsWait(lwip_flags, LWIP_READY_FLAG, osFlagsWaitAll, osWaitForever) & osFlagsError) {
        Error_Handler();
    }

    DiagnosticFrameDTO_t diagnostic_frame = {
        .header =
            {
                .magic = NETWORK_PROTOCOL_MAGIC,
                .version = NETWORK_PROTOCOL_VERSION,
                .message_type = NETWORK_MESSAGE_TYPE_DIAGNOSTIC_SNAPSHOT,
                .header_length = sizeof(NetworkFrameHeaderDTO_t),
                .payload_length = sizeof(DiagnosticPayloadDTO_t),
            },
    };
    uint32_t sequence = 0;
    struct netconn *client = NULL;

    for (;;) {
        struct netconn *connection = netconn_new(NETCONN_TCP);
        if (connection == NULL) {
            diagnostic_counter_increment(&runtime.netconn_alloc_errors);
            diagnostic_record_error(&runtime, ERR_MEM);
            osDelay(10);
            continue;
        }

        const err_t bind_error = netconn_bind(connection, IP_ADDR_ANY, DIAGNOSTIC_NETWORK_TASK_PORT);
        if (bind_error != ERR_OK) {
            diagnostic_counter_increment(&runtime.bind_errors);
            diagnostic_record_error(&runtime, bind_error);
            netconn_delete(connection);
            osDelay(10);
            continue;
        }

        const err_t listen_error = netconn_listen(connection);
        if (listen_error != ERR_OK) {
            diagnostic_counter_increment(&runtime.listen_errors);
            diagnostic_record_error(&runtime, listen_error);
            netconn_delete(connection);
            osDelay(10);
            continue;
        }

        for (;;) {
            const err_t accept_error = netconn_accept(connection, &client);
            if (accept_error != ERR_OK) {
                diagnostic_counter_increment(&runtime.accept_errors);
                diagnostic_record_error(&runtime, accept_error);
                break;
            }

            diagnostic_counter_increment(&runtime.connections_accepted);
            runtime.active_connection = 1U;
            netconn_set_sendtimeout(client, DIAGNOSTIC_NETWORK_SEND_TIMEOUT_MS);

            for (;;) {
                ++sequence;
                diagnostic_frame.payload.sequence = sequence;
                system_diagnostics_collect(&system_diagnostics);
                diagnostic_frame.payload.system = system_diagnostics;
                ethernetif_get_rx_diagnostics(&ethernet_rx_diagnostics);
                diagnostic_frame.payload.ethernet_rx = ethernet_rx_diagnostics;
                if (EthernetPort_GetDiagnostics(&ethernet_port_diagnostics) != HAL_OK) {
                    diagnostic_counter_increment(&runtime.snapshot_errors);
                    ethernet_port_diagnostics = (EthernetPortDiagnostics){0};
                }
                diagnostic_frame.payload.ethernet_port = ethernet_port_diagnostics;
                lwip_diagnostics_collect(&lwip_diagnostics);
                diagnostic_frame.payload.lwip = lwip_diagnostics;
                diagnostic_counter_increment(&runtime.send_attempts);
                diagnostic_task_fill_stats(&diagnostic_frame.payload, &runtime);

                size_t bytes_written = 0U;
                const err_t write_error = netconn_write_partly(
                    client,
                    &diagnostic_frame,
                    sizeof(diagnostic_frame),
                    NETCONN_COPY,
                    &bytes_written
                );
                diagnostic_counter_add(&runtime.bytes_sent, bytes_written);

                if (write_error != ERR_OK || bytes_written != sizeof(diagnostic_frame)) {
                    diagnostic_counter_increment(&runtime.send_errors);
                    if (bytes_written != sizeof(diagnostic_frame)) {
                        diagnostic_counter_increment(&runtime.partial_writes);
                    }
                    diagnostic_record_error(&runtime, write_error == ERR_OK ? ERR_BUF : write_error);
                    break;
                }

                diagnostic_counter_increment(&runtime.send_successes);
                osDelay(DIAGNOSTIC_NETWORK_TASK_INTERVAL_MS);
            }

            netconn_close(client);
            netconn_delete(client);
            runtime.active_connection = 0U;
            diagnostic_counter_increment(&runtime.connections_closed);
        }

        netconn_close(connection);
        netconn_delete(connection);
    }
}

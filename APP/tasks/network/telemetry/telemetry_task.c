#include "telemetry_task.h"
#include "network_events.h"
#include "task_context.h"
#include "telemetry_dto.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/opt.h"

#include <stddef.h>
#include <stdint.h>

static volatile TelemetryTaskDiagnostics telemetry_task_diagnostics;

static void telemetry_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void telemetry_counter_add(volatile uint32_t *counter, size_t value) {
    if (value > UINT32_MAX || UINT32_MAX - *counter < (uint32_t)value) {
        *counter = UINT32_MAX;
    } else {
        *counter += (uint32_t)value;
    }
}

static void telemetry_record_error(err_t error) { telemetry_task_diagnostics.last_error = (int32_t)error; }

void telemetry_task_get_diagnostics(TelemetryTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->port = TELEMETRY_NETWORK_TASK_PORT;
    diagnostics->interval_ms = TELEMETRY_NETWORK_TASK_INTERVAL_MS;
    diagnostics->send_timeout_ms = TELEMETRY_NETWORK_SEND_TIMEOUT_MS;
    diagnostics->connections_accepted = telemetry_task_diagnostics.connections_accepted;
    diagnostics->connections_closed = telemetry_task_diagnostics.connections_closed;
    diagnostics->active_connection = telemetry_task_diagnostics.active_connection;
    diagnostics->send_attempts = telemetry_task_diagnostics.send_attempts;
    diagnostics->send_successes = telemetry_task_diagnostics.send_successes;
    diagnostics->send_errors = telemetry_task_diagnostics.send_errors;
    diagnostics->partial_writes = telemetry_task_diagnostics.partial_writes;
    diagnostics->bytes_sent = telemetry_task_diagnostics.bytes_sent;
    diagnostics->netconn_alloc_errors = telemetry_task_diagnostics.netconn_alloc_errors;
    diagnostics->bind_errors = telemetry_task_diagnostics.bind_errors;
    diagnostics->listen_errors = telemetry_task_diagnostics.listen_errors;
    diagnostics->accept_errors = telemetry_task_diagnostics.accept_errors;
    diagnostics->last_error = telemetry_task_diagnostics.last_error;
}

void TelemetryTask(void *argument) {
    NetworkTaskContext *context = argument;
    if ((context == NULL) || (context->lwip_flags == NULL)) {
        Error_Handler();
        return;
    }

    if (osEventFlagsWait(context->lwip_flags, LWIP_READY_FLAG, osFlagsWaitAll, osWaitForever) & osFlagsError) {
        Error_Handler();
        return;
    }

    uint32_t sequence = 0U;
    for (;;) {
        struct netconn *connection = netconn_new(NETCONN_TCP);
        if (connection == NULL) {
            telemetry_counter_increment(&telemetry_task_diagnostics.netconn_alloc_errors);
            telemetry_record_error(ERR_MEM);
            osDelay(10U);
            continue;
        }

        const err_t bind_error = netconn_bind(connection, IP_ADDR_ANY, TELEMETRY_NETWORK_TASK_PORT);
        if (bind_error != ERR_OK) {
            telemetry_counter_increment(&telemetry_task_diagnostics.bind_errors);
            telemetry_record_error(bind_error);
            netconn_delete(connection);
            osDelay(10U);
            continue;
        }

        const err_t listen_error = netconn_listen(connection);
        if (listen_error != ERR_OK) {
            telemetry_counter_increment(&telemetry_task_diagnostics.listen_errors);
            telemetry_record_error(listen_error);
            netconn_close(connection);
            netconn_delete(connection);
            osDelay(10U);
            continue;
        }

        for (;;) {
            struct netconn *client = NULL;
            const err_t accept_error = netconn_accept(connection, &client);
            if (accept_error != ERR_OK) {
                telemetry_counter_increment(&telemetry_task_diagnostics.accept_errors);
                telemetry_record_error(accept_error);
                break;
            }

            telemetry_counter_increment(&telemetry_task_diagnostics.connections_accepted);
            telemetry_task_diagnostics.active_connection = 1U;
            netconn_set_sendtimeout(client, TELEMETRY_NETWORK_SEND_TIMEOUT_MS);
            for (;;) {
                TelemetryFrameDTO_t frame;
                size_t bytes_written = 0U;
                telemetry_build_test_frame(&frame, ++sequence);
                telemetry_counter_increment(&telemetry_task_diagnostics.send_attempts);

                const err_t write_error =
                    netconn_write_partly(client, &frame, sizeof(frame), NETCONN_COPY, &bytes_written);
                telemetry_counter_add(&telemetry_task_diagnostics.bytes_sent, bytes_written);
                if (write_error != ERR_OK || bytes_written != sizeof(frame)) {
                    telemetry_counter_increment(&telemetry_task_diagnostics.send_errors);
                    if (bytes_written != sizeof(frame)) {
                        telemetry_counter_increment(&telemetry_task_diagnostics.partial_writes);
                    }
                    telemetry_record_error(write_error == ERR_OK ? ERR_BUF : write_error);
                    break;
                }

                telemetry_counter_increment(&telemetry_task_diagnostics.send_successes);
                osDelay(TELEMETRY_NETWORK_TASK_INTERVAL_MS);
            }

            netconn_close(client);
            netconn_delete(client);
            telemetry_task_diagnostics.active_connection = 0U;
            telemetry_counter_increment(&telemetry_task_diagnostics.connections_closed);
        }

        netconn_close(connection);
        netconn_delete(connection);
    }
}

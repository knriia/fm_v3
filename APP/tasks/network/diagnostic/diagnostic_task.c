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
#include "lwip/opt.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>

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

typedef struct {
    uint32_t eth_if_min_free_bytes;
    uint32_t eth_link_min_free_bytes;
    uint32_t tcpip_thread_min_free_bytes;
} NetworkTasksRuntime;

typedef struct {
    TaskHandle_t task_handle;
    uint32_t previous_runtime_ticks;
} TaskRuntimeSample;

typedef struct {
    uint32_t previous_total_runtime_ticks;
    uint8_t initialized;
    TaskRuntimeSample diagnostic_task;
    TaskRuntimeSample eth_if;
    TaskRuntimeSample eth_link;
    TaskRuntimeSample tcpip_thread;
} RuntimePercentTracker;

static RuntimePercentTracker runtime_percent_tracker;

static uint32_t diagnostic_task_calculate_runtime_percent(
    TaskRuntimeSample *sample,
    TaskHandle_t task_handle,
    uint32_t runtime_ticks,
    uint32_t total_runtime_ticks
) {
    if (task_handle == NULL) {
        sample->task_handle = NULL;
        sample->previous_runtime_ticks = 0U;
        return 0U;
    }

    if ((runtime_percent_tracker.initialized == 0U) || (sample->task_handle != task_handle)) {
        sample->task_handle = task_handle;
        sample->previous_runtime_ticks = runtime_ticks;
        return 0U;
    }

    const uint32_t total_runtime_delta = total_runtime_ticks - runtime_percent_tracker.previous_total_runtime_ticks;
    const uint32_t task_runtime_delta = runtime_ticks - sample->previous_runtime_ticks;
    sample->previous_runtime_ticks = runtime_ticks;

    if (total_runtime_delta == 0U) {
        return 0U;
    }

    const uint32_t runtime_percent = (uint32_t)(((uint64_t)task_runtime_delta * 100U) / total_runtime_delta);
    return runtime_percent > 100U ? 100U : runtime_percent;
}

static void diagnostic_task_commit_runtime_sample(uint32_t total_runtime_ticks) {
    runtime_percent_tracker.previous_total_runtime_ticks = total_runtime_ticks;
    runtime_percent_tracker.initialized = 1U;
}

static void diagnostic_network_task_fill_stats(
    TaskDiagnosticsDTO_t *diagnostics,
    TaskHandle_t task_handle,
    TaskRuntimeSample *runtime_sample,
    uint32_t stack_size_bytes,
    uint32_t *minimum_free_bytes,
    uint32_t total_runtime_ticks
) {
    TaskStatus_t task_status = {0};
    diagnostics->stack_size_bytes = stack_size_bytes;

    if (task_handle == NULL) {
        diagnostics->priority = (int32_t)osPriorityError;
        diagnostics->base_priority = (int32_t)osPriorityError;
        diagnostics->state = (uint32_t)osThreadError;
        diagnostics->stack_free_bytes = 0U;
        diagnostics->stack_min_free_bytes = 0U;
        diagnostics->stack_base_address = 0U;
        diagnostics->runtime_ticks = 0U;
        diagnostics->runtime_percent =
            diagnostic_task_calculate_runtime_percent(runtime_sample, NULL, 0U, total_runtime_ticks);
        return;
    }

    vTaskGetInfo(task_handle, &task_status, pdTRUE, eInvalid);
    const uint32_t stack_free_bytes = (uint32_t)task_status.usStackHighWaterMark * sizeof(StackType_t);
    if (*minimum_free_bytes == UINT32_MAX || stack_free_bytes < *minimum_free_bytes) {
        *minimum_free_bytes = stack_free_bytes;
    }

    diagnostics->stack_free_bytes = stack_free_bytes;
    diagnostics->stack_min_free_bytes = *minimum_free_bytes;
    diagnostics->stack_base_address = (uint32_t)(uintptr_t)task_status.pxStackBase;
    diagnostics->priority = (int32_t)task_status.uxCurrentPriority;
    diagnostics->base_priority = (int32_t)task_status.uxBasePriority;
    diagnostics->state = (uint32_t)task_status.eCurrentState;
    diagnostics->runtime_ticks = task_status.ulRunTimeCounter;
    diagnostics->runtime_percent = diagnostic_task_calculate_runtime_percent(
        runtime_sample,
        task_handle,
        task_status.ulRunTimeCounter,
        total_runtime_ticks
    );
}

static void diagnostic_collect_network_tasks(
    DiagnosticPayloadDTO_t *payload,
    NetworkTasksRuntime *runtime,
    uint32_t total_runtime_ticks
) {
    diagnostic_network_task_fill_stats(
        &payload->eth_if.runtime,
        xTaskGetHandle("EthIf"),
        &runtime_percent_tracker.eth_if,
        ETHERNETIF_INPUT_THREAD_STACK_SIZE_BYTES,
        &runtime->eth_if_min_free_bytes,
        total_runtime_ticks
    );
    diagnostic_network_task_fill_stats(
        &payload->eth_link.runtime,
        xTaskGetHandle("EthLink"),
        &runtime_percent_tracker.eth_link,
        ETHERNETIF_LINK_THREAD_STACK_SIZE_BYTES,
        &runtime->eth_link_min_free_bytes,
        total_runtime_ticks
    );
    diagnostic_network_task_fill_stats(
        &payload->tcpip_thread.runtime,
        xTaskGetHandle(TCPIP_THREAD_NAME),
        &runtime_percent_tracker.tcpip_thread,
        TCPIP_THREAD_STACKSIZE,
        &runtime->tcpip_thread_min_free_bytes,
        total_runtime_ticks
    );
}

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
    TaskStatus_t task_status = {0};
    vTaskGetInfo(NULL, &task_status, pdTRUE, eInvalid);
    const uint32_t stack_free_bytes = (uint32_t)task_status.usStackHighWaterMark * sizeof(StackType_t);

    if ((runtime->stack_min_free_bytes == 0U) || (stack_free_bytes < runtime->stack_min_free_bytes)) {
        runtime->stack_min_free_bytes = stack_free_bytes;
    }

    payload->diagnostic_task.runtime.stack_size_bytes = DIAGNOSTIC_TASK_STACK_SIZE_BYTES;
    payload->diagnostic_task.runtime.stack_free_bytes = stack_free_bytes;
    payload->diagnostic_task.runtime.stack_min_free_bytes = runtime->stack_min_free_bytes;
    payload->diagnostic_task.runtime.stack_base_address = (uint32_t)(uintptr_t)task_status.pxStackBase;
    payload->diagnostic_task.runtime.priority = (int32_t)task_status.uxCurrentPriority;
    payload->diagnostic_task.runtime.base_priority = (int32_t)task_status.uxBasePriority;
    payload->diagnostic_task.runtime.state = (uint32_t)task_status.eCurrentState;
    payload->diagnostic_task.runtime.runtime_ticks = task_status.ulRunTimeCounter;
    payload->diagnostic_task.runtime.runtime_percent = diagnostic_task_calculate_runtime_percent(
        &runtime_percent_tracker.diagnostic_task,
        xTaskGetCurrentTaskHandle(),
        task_status.ulRunTimeCounter,
        payload->system.freertos.total_runtime_ticks
    );
    diagnostic_task_commit_runtime_sample(payload->system.freertos.total_runtime_ticks);
    payload->diagnostic_task.port = DIAGNOSTIC_NETWORK_TASK_PORT;
    payload->diagnostic_task.interval_ms = DIAGNOSTIC_NETWORK_TASK_INTERVAL_MS;
    payload->diagnostic_task.send_timeout_ms = DIAGNOSTIC_NETWORK_SEND_TIMEOUT_MS;
    payload->diagnostic_task.connections_accepted = runtime->connections_accepted;
    payload->diagnostic_task.connections_closed = runtime->connections_closed;
    payload->diagnostic_task.active_connection = runtime->active_connection;
    payload->diagnostic_task.send_attempts = runtime->send_attempts;
    payload->diagnostic_task.send_successes = runtime->send_successes;
    payload->diagnostic_task.send_errors = runtime->send_errors;
    payload->diagnostic_task.partial_writes = runtime->partial_writes;
    payload->diagnostic_task.bytes_sent = runtime->bytes_sent;
    payload->diagnostic_task.netconn_alloc_errors = runtime->netconn_alloc_errors;
    payload->diagnostic_task.bind_errors = runtime->bind_errors;
    payload->diagnostic_task.listen_errors = runtime->listen_errors;
    payload->diagnostic_task.accept_errors = runtime->accept_errors;
    payload->diagnostic_task.snapshot_errors = runtime->snapshot_errors;
    payload->diagnostic_task.last_error = runtime->last_error;
}

void DiagnosticTask(void *argument) {
    NetworkTaskContext *context = argument;
    DiagnosticTaskRuntime runtime = {0};
    NetworkTasksRuntime network_tasks_runtime = {
        .eth_if_min_free_bytes = UINT32_MAX,
        .eth_link_min_free_bytes = UINT32_MAX,
        .tcpip_thread_min_free_bytes = UINT32_MAX,
    };

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
                system_diagnostics_collect(&diagnostic_frame.payload.system);
                ethernetif_get_rx_diagnostics(&diagnostic_frame.payload.eth_if.ethernet_rx);
                if (EthernetPort_GetDiagnostics(&diagnostic_frame.payload.eth_link.ethernet_port) != HAL_OK) {
                    diagnostic_counter_increment(&runtime.snapshot_errors);
                    diagnostic_frame.payload.eth_link.ethernet_port = (EthernetPortDiagnostics){0};
                }
                lwip_diagnostics_collect(&diagnostic_frame.payload.tcpip_thread.lwip);
                diagnostic_collect_network_tasks(
                    &diagnostic_frame.payload,
                    &network_tasks_runtime,
                    diagnostic_frame.payload.system.freertos.total_runtime_ticks
                );
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

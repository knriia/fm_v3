//
// Created by umnik on 23.09.2026.
//

#include "diagnostic_task.h"
#include "command/command_queue.h"
#include "command_decoder.h"
#include "command_task.h"
#include "diagnostic_dto.h"
#include "motion/execution_buffer.h"
#include "motion/plan_buffer.h"
#include "network_events.h"
#include "task_context.h"
#include "task_names.h"

#include "cmsis_os.h"
#include "ethernet_port.h"
#include "ethernetif.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    TaskHandle_t task_handle;
    uint32_t previous_runtime_ticks;
} TaskRuntimeSample;

/* CPU share of each task is measured between two consecutive snapshots. */
typedef struct {
    uint32_t previous_total_runtime_ticks;
    uint8_t initialized;
    TaskRuntimeSample tasks[DIAGNOSTIC_TASK_SLOT_COUNT];
} RuntimePercentTracker;

static const char *const diagnostic_task_names[DIAGNOSTIC_TASK_SLOT_COUNT] = {
    [DIAGNOSTIC_TASK_SLOT_STARTUP] = STARTUP_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_COMMAND] = COMMAND_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_MOTION] = MOTION_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_STEP_GENERATOR] = STEP_GENERATOR_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_TELEMETRY] = TELEMETRY_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_DIAGNOSTIC] = DIAGNOSTIC_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_ETH_IF] = ETHERNET_IF_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_ETH_LINK] = ETHERNET_LINK_TASK_NAME,
    [DIAGNOSTIC_TASK_SLOT_TCPIP] = TCPIP_TASK_NAME,
};

static uint32_t diagnostic_task_calculate_runtime_percent(
    const RuntimePercentTracker *tracker,
    TaskRuntimeSample *sample,
    TaskHandle_t task_handle,
    uint32_t runtime_ticks,
    uint32_t total_runtime_ticks
) {
    /* The first sample, or a task created anew under the same name, has nothing to compare against. */
    if (tracker->initialized == 0U || sample->task_handle != task_handle) {
        sample->task_handle = task_handle;
        sample->previous_runtime_ticks = runtime_ticks;
        return 0U;
    }

    const uint32_t total_runtime_delta = total_runtime_ticks - tracker->previous_total_runtime_ticks;
    const uint32_t task_runtime_delta = runtime_ticks - sample->previous_runtime_ticks;
    sample->previous_runtime_ticks = runtime_ticks;
    if (total_runtime_delta == 0U) {
        return 0U;
    }

    const uint32_t runtime_percent = (uint32_t)(((uint64_t)task_runtime_delta * 100U) / total_runtime_delta);
    return runtime_percent > 100U ? 100U : runtime_percent;
}

static void diagnostic_task_fill_task(
    TaskDiagnosticsDTO_t *diagnostics,
    const RuntimePercentTracker *tracker,
    TaskRuntimeSample *sample,
    TaskHandle_t task_handle,
    uint32_t total_runtime_ticks
) {
    if (task_handle == NULL) {
        *diagnostics = (TaskDiagnosticsDTO_t){
            .priority = (int32_t)osPriorityError,
            .base_priority = (int32_t)osPriorityError,
            .state = (uint32_t)osThreadError,
        };
        *sample = (TaskRuntimeSample){0};
        return;
    }

    TaskStatus_t task_status = {0};
    vTaskGetInfo(task_handle, &task_status, pdTRUE, eInvalid);
    *diagnostics = (TaskDiagnosticsDTO_t){
        .stack_min_free_bytes = (uint32_t)task_status.usStackHighWaterMark * sizeof(StackType_t),
        .priority = (int32_t)task_status.uxCurrentPriority,
        .base_priority = (int32_t)task_status.uxBasePriority,
        .state = (uint32_t)task_status.eCurrentState,
        .runtime_percent = diagnostic_task_calculate_runtime_percent(
            tracker,
            sample,
            task_handle,
            task_status.ulRunTimeCounter,
            total_runtime_ticks
        ),
    };
}

static void diagnostic_task_collect_tasks(
    TaskDiagnosticsDTO_t tasks[DIAGNOSTIC_TASK_SLOT_COUNT],
    RuntimePercentTracker *tracker,
    uint32_t total_runtime_ticks
) {
    for (uint32_t slot = 0U; slot < DIAGNOSTIC_TASK_SLOT_COUNT; ++slot) {
        diagnostic_task_fill_task(
            &tasks[slot],
            tracker,
            &tracker->tasks[slot],
            xTaskGetHandle(diagnostic_task_names[slot]),
            total_runtime_ticks
        );
    }
    tracker->previous_total_runtime_ticks = total_runtime_ticks;
    tracker->initialized = 1U;
}

static void diagnostic_task_collect_command_channel(CommandChannelDiagnosticsDTO_t *diagnostics) {
    CommandTaskDiagnostics command = {0};
    CommandDecoderDiagnostics decoder = {0};
    command_task_get_diagnostics(&command);
    command_decoder_get_diagnostics(&decoder);
    *diagnostics = (CommandChannelDiagnosticsDTO_t){
        .connections_accepted = command.connections_accepted,
        .connections_closed = command.connections_closed,
        .active_connection = command.active_connection,
        .frames_received = command.complete_frames_received,
        .invalid_frames = command.invalid_frames,
        .rx_buffer_max_bytes = command.buffer_max_bytes,
        .recv_errors = command.recv_errors,
        .send_errors = command.response_send_errors,
        .decoder_errors = command.decoder_errors,
        .acks_deferred = command.acks_deferred,
        .last_error = command.last_error,
        .commands_received = decoder.commands_received,
        .commands_rejected = decoder.commands_rejected,
        .decoder_last_error = decoder.last_error,
    };
}

static void diagnostic_task_collect_buffers(DiagnosticPayloadDTO_t *payload) {
    command_queue_get_diagnostics(&payload->command_queue);
    (void)plan_buffer_get_diagnostics(&payload->plan_buffer);

    payload->execution_buffer = (ExecutionBufferDiagnosticsDTO_t){
        .level = execution_buffer_count(),
        .underruns = motion_task_get_stream_underrun_count(),
    };
}

/* A busy Ethernet port skips one snapshot; the previous port values stay until the next one succeeds. */
static void diagnostic_task_collect_ethernet(EthernetDiagnosticsDTO_t *diagnostics) {
    EthernetPortDiagnostics port = {0};
    if (EthernetPort_GetDiagnostics(&port) == HAL_OK) {
        diagnostics->link_up = port.link_up;
        diagnostics->speed = port.speed;
        diagnostics->duplex = port.duplex;
        diagnostics->link_up_events = port.link_up_events;
        diagnostics->link_down_events = port.link_down_events;
        diagnostics->link_state_read_errors = port.link_state_read_errors;
        diagnostics->dma_error = port.dma_error;
        diagnostics->tx_errors = port.tx_errors;
        diagnostics->tx_descriptor_overflows = port.tx_descriptor_overflows;
        diagnostics->mutex_timeouts = port.mutex_timeouts;
    }

    EthernetRxDiagnostics rx = {0};
    ethernetif_get_rx_diagnostics(&rx);
    diagnostics->rx_pool_max_used = rx.rx_pool_max_used;
    diagnostics->rx_pool_exhaustions = rx.rx_pool_exhaustions;
    diagnostics->rx_dma_buffer_unavailable = rx.dma_receive_buffer_unavailable;
    diagnostics->rx_dropped_packets = rx.dropped_packets;
    diagnostics->rx_read_errors = rx.hal_read_data_errors;
}

static void diagnostic_task_collect(DiagnosticPayloadDTO_t *payload, RuntimePercentTracker *tracker) {
    const uint32_t total_runtime_ticks = system_diagnostics_collect(&payload->system);
    diagnostic_task_collect_tasks(payload->tasks, tracker, total_runtime_ticks);
    diagnostic_task_collect_command_channel(&payload->command_channel);
    motion_task_get_diagnostics(&payload->motion);
    step_generator_task_get_diagnostics(&payload->step_generator);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &payload->x_axis);
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, &payload->y_axis);
    motion_task_get_axis_diagnostics(MOTION_AXIS_Z, &payload->z_axis);
    diagnostic_task_collect_buffers(payload);
    diagnostic_task_collect_ethernet(&payload->ethernet);
    lwip_diagnostics_collect(&payload->lwip);
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        motion_axis_hardware_get_timing((MotionAxis_t)axis, &payload->step_timing[axis]);
    }
}

void DiagnosticTask(void *argument) {
    NetworkTaskContext *context = argument;
    RuntimePercentTracker runtime_tracker = {0};

    configASSERT(context != NULL);
    configASSERT(context->lwip_flags != NULL);

    const uint32_t wait_result =
        osEventFlagsWait(context->lwip_flags, LWIP_READY_FLAG, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
    configASSERT((wait_result & osFlagsError) == 0U);

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
            osDelay(10);
            continue;
        }
        if (netconn_bind(connection, IP_ADDR_ANY, DIAGNOSTIC_NETWORK_TASK_PORT) != ERR_OK ||
            netconn_listen(connection) != ERR_OK) {
            netconn_delete(connection);
            osDelay(10);
            continue;
        }

        while (netconn_accept(connection, &client) == ERR_OK) {
            netconn_set_sendtimeout(client, DIAGNOSTIC_NETWORK_SEND_TIMEOUT_MS);

            for (;;) {
                ++sequence;
                diagnostic_frame.payload.sequence = sequence;
                diagnostic_task_collect(&diagnostic_frame.payload, &runtime_tracker);

                size_t bytes_written = 0U;
                const err_t write_error = netconn_write_partly(
                    client,
                    &diagnostic_frame,
                    sizeof(diagnostic_frame),
                    NETCONN_COPY,
                    &bytes_written
                );
                if (write_error != ERR_OK || bytes_written != sizeof(diagnostic_frame)) {
                    break;
                }
                osDelay(DIAGNOSTIC_NETWORK_TASK_INTERVAL_MS);
            }

            netconn_close(client);
            netconn_delete(client);
        }

        netconn_close(connection);
        netconn_delete(connection);
    }
}

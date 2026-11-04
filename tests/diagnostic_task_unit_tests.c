#include "command/command_queue.h"
#include "command_decoder.h"
#include "command_task.h"
#include "diagnostic_dto.h"
#include "ethernet_port.h"
#include "ethernetif.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "motion/plan_buffer.h"
#include "task_context.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;
static TaskStatus_t test_task_status;
/* Slots whose task does not exist: xTaskGetHandle returns NULL for their names. */
static const char *test_missing_task_name;
static uint32_t test_error_handler_calls;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint32_t test_stop_on_delay;
static uint32_t test_stop_on_second_new;
static uint32_t test_stop_on_second_accept;
static uint32_t test_succeed_after_first_write;
static uint32_t test_netconn_new_calls;
static uint32_t test_netconn_bind_calls;
static uint32_t test_netconn_listen_calls;
static uint32_t test_netconn_accept_calls;
static uint32_t test_netconn_write_calls;
static uint32_t test_netconn_close_calls;
static uint32_t test_netconn_delete_calls;
static uint32_t test_event_flags_result;
static uint32_t test_new_returns_null;
static int test_ethernet_diagnostics_result;
static err_t test_bind_result;
static err_t test_listen_result;
static err_t test_accept_result;
static err_t test_write_result;
static size_t test_bytes_written;
static size_t test_captured_write_size;
static DiagnosticFrameDTO_t test_captured_frame;
static uint32_t test_total_runtime_ticks;
static EthernetPortDiagnostics test_ethernet_port_diagnostics;
static EthernetRxDiagnostics test_ethernet_rx_diagnostics;
static CommandTaskDiagnostics test_command_diagnostics;
static CommandDecoderDiagnostics test_command_decoder_diagnostics;
static MotionTaskDiagnostics test_motion_task_diagnostics;
static StepGeneratorTaskDiagnostics test_step_generator_diagnostics;
static CommandQueueDiagnostics_t test_command_queue_diagnostics;
static PlanBufferDiagnostics_t test_plan_buffer_diagnostics;
static uint32_t test_execution_buffer_level;
static uint32_t test_underruns;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

TaskHandle_t xTaskGetHandle(const char *name) {
    if (test_missing_task_name != NULL && strcmp(name, test_missing_task_name) == 0) {
        return NULL;
    }
    return (TaskHandle_t)(uintptr_t)1U;
}

void vTaskGetInfo(TaskHandle_t task_handle, TaskStatus_t *task_status, int get_free_stack, int state) {
    (void)task_handle;
    (void)get_free_stack;
    (void)state;
    *task_status = test_task_status;
}

void Error_Handler(void) {
    ++test_error_handler_calls;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
}

void ethernetif_get_rx_diagnostics(EthernetRxDiagnostics *diagnostics) { *diagnostics = test_ethernet_rx_diagnostics; }

int EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics) {
    *diagnostics = test_ethernet_port_diagnostics;
    return test_ethernet_diagnostics_result;
}

uint32_t system_diagnostics_collect(SystemDiagnostics *diagnostics) {
    *diagnostics = (SystemDiagnostics){.tick_count = 5007U, .reset_flags = 12U};
    return test_total_runtime_ticks;
}

void lwip_diagnostics_collect(LwipDiagnostics *diagnostics) {
    *diagnostics = (LwipDiagnostics){.mailbox_errors = 3U, .if_out_discards = 4U};
}

void command_task_get_diagnostics(CommandTaskDiagnostics *diagnostics) { *diagnostics = test_command_diagnostics; }

void command_decoder_get_diagnostics(CommandDecoderDiagnostics *diagnostics) {
    *diagnostics = test_command_decoder_diagnostics;
}

void motion_task_get_diagnostics(MotionTaskDiagnostics *diagnostics) { *diagnostics = test_motion_task_diagnostics; }

void motion_task_get_axis_diagnostics(MotionAxis_t axis, MotionAxisDiagnostics *diagnostics) {
    static const MotionAxisDiagnostics axes[MOTION_AXIS_HARDWARE_COUNT] = {
        [MOTION_AXIS_X] = {.travel_um = 301000U, .scale_position_um = -5},
        [MOTION_AXIS_Y] = {.travel_um = 402000U, .scale_position_um = 7},
        [MOTION_AXIS_Z] = {.travel_um = 50300U, .scale_position_um = -9},
    };
    *diagnostics = axes[axis];
}

void motion_axis_hardware_get_timing(MotionAxis_t axis, MotionAxisHardwareTiming_t *timing) {
    *timing = (MotionAxisHardwareTiming_t){
        .status = MOTION_AXIS_HARDWARE_STATUS_RUNNING,
        .timing_faults = 10U + (uint32_t)axis,
        .isr_max_ns = 1000U + (uint32_t)axis,
        .min_slack_ns = 5000U + (uint32_t)axis,
    };
}

void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics) {
    *diagnostics = test_step_generator_diagnostics;
}

void command_queue_get_diagnostics(CommandQueueDiagnostics_t *diagnostics) {
    *diagnostics = test_command_queue_diagnostics;
}

PlanBufferStatus_t plan_buffer_get_diagnostics(PlanBufferDiagnostics_t *diagnostics) {
    *diagnostics = test_plan_buffer_diagnostics;
    return PLAN_BUFFER_STATUS_OK;
}

uint32_t execution_buffer_count(void) { return test_execution_buffer_level; }

uint32_t step_executor_get_underrun_count(void) { return test_underruns; }

uint32_t osEventFlagsWait(osEventFlagsId_t event_flags_id, uint32_t flags, uint32_t options, uint32_t timeout) {
    (void)event_flags_id;
    (void)flags;
    (void)options;
    (void)timeout;
    return test_event_flags_result;
}

uint32_t osDelay(uint32_t milliseconds) {
    (void)milliseconds;
    if (test_jump_active != 0U && test_stop_on_delay != 0U) {
        longjmp(test_jump_buffer, 1);
    }
    return 0U;
}

struct netconn {
    uint32_t unused;
};

static struct netconn test_listener;
static struct netconn test_client;

struct netconn *netconn_new(int type) {
    (void)type;
    ++test_netconn_new_calls;
    if (test_jump_active != 0U && test_stop_on_second_new != 0U && test_netconn_new_calls > 1U) {
        longjmp(test_jump_buffer, 1);
    }
    return test_new_returns_null != 0U ? NULL : &test_listener;
}

err_t netconn_bind(struct netconn *connection, void *address, uint16_t port) {
    (void)connection;
    (void)address;
    (void)port;
    ++test_netconn_bind_calls;
    return test_bind_result;
}

err_t netconn_listen(struct netconn *connection) {
    (void)connection;
    ++test_netconn_listen_calls;
    return test_listen_result;
}

err_t netconn_accept(struct netconn *connection, struct netconn **new_connection) {
    (void)connection;
    ++test_netconn_accept_calls;
    if (test_jump_active != 0U && test_stop_on_second_accept != 0U && test_netconn_accept_calls > 1U) {
        longjmp(test_jump_buffer, 1);
    }
    if (test_accept_result == ERR_OK) {
        *new_connection = &test_client;
    }
    return test_accept_result;
}

void netconn_set_sendtimeout(struct netconn *connection, int timeout) {
    (void)connection;
    (void)timeout;
}

err_t netconn_write_partly(
    struct netconn *connection,
    const void *data,
    size_t size,
    int apiflags,
    size_t *bytes_written
) {
    (void)connection;
    (void)apiflags;
    ++test_netconn_write_calls;
    test_captured_write_size = size;
    if (size == sizeof(test_captured_frame)) {
        (void)memcpy(&test_captured_frame, data, sizeof(test_captured_frame));
    }
    if (test_succeed_after_first_write != 0U && test_netconn_write_calls > 1U) {
        *bytes_written = size;
        return ERR_OK;
    }
    *bytes_written = test_bytes_written;
    return test_write_result;
}

err_t netconn_close(struct netconn *connection) {
    (void)connection;
    ++test_netconn_close_calls;
    return ERR_OK;
}

void netconn_delete(struct netconn *connection) {
    (void)connection;
    ++test_netconn_delete_calls;
}

#define static
#define DiagnosticTask DiagnosticTask_test_unused
#include "../APP/tasks/network/diagnostic/diagnostic_task.c"
#undef DiagnosticTask
#undef static

static void set_task_status(uint32_t high_water_mark_words, uint32_t runtime_ticks) {
    test_task_status = (TaskStatus_t){
        .uxCurrentPriority = 24U,
        .uxBasePriority = 23U,
        .eCurrentState = 2U,
        .usStackHighWaterMark = high_water_mark_words,
        .ulRunTimeCounter = runtime_ticks,
    };
}

static void test_runtime_percent_calculation(void) {
    RuntimePercentTracker tracker = {0};
    TaskRuntimeSample sample = {0};
    TaskHandle_t task_handle = (TaskHandle_t)(uintptr_t)2U;

    expect_u32(
        diagnostic_task_calculate_runtime_percent(&tracker, &sample, task_handle, 100U, 1000U),
        0U,
        "first sample has no runtime share"
    );

    tracker = (RuntimePercentTracker){.previous_total_runtime_ticks = 1000U, .initialized = 1U};
    expect_u32(
        diagnostic_task_calculate_runtime_percent(&tracker, &sample, task_handle, 150U, 1100U),
        50U,
        "task runtime share"
    );

    tracker.previous_total_runtime_ticks = 1100U;
    expect_u32(
        diagnostic_task_calculate_runtime_percent(&tracker, &sample, task_handle, 400U, 1200U),
        100U,
        "task runtime share is capped"
    );

    tracker.previous_total_runtime_ticks = 1200U;
    expect_u32(
        diagnostic_task_calculate_runtime_percent(&tracker, &sample, task_handle, 450U, 1200U),
        0U,
        "no run time passed"
    );

    expect_u32(
        diagnostic_task_calculate_runtime_percent(&tracker, &sample, (TaskHandle_t)(uintptr_t)3U, 10U, 1300U),
        0U,
        "a recreated task starts a new sample"
    );
    expect_u32((uint32_t)(uintptr_t)sample.task_handle, 3U, "sample follows the new task");
}

static void test_task_slots(void) {
    TaskDiagnosticsDTO_t tasks[DIAGNOSTIC_TASK_SLOT_COUNT] = {0};
    RuntimePercentTracker tracker = {0};

    set_task_status(100U, 500U);
    test_missing_task_name = TELEMETRY_TASK_NAME;
    diagnostic_task_collect_tasks(tasks, &tracker, 1000U);

    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_MOTION].stack_min_free_bytes, 400U, "stack high-water mark in bytes");
    expect_u32((uint32_t)tasks[DIAGNOSTIC_TASK_SLOT_MOTION].priority, 24U, "task priority");
    expect_u32((uint32_t)tasks[DIAGNOSTIC_TASK_SLOT_MOTION].base_priority, 23U, "task base priority");
    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_MOTION].state, 2U, "task state");
    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_MOTION].runtime_percent, 0U, "first snapshot has no CPU share");
    expect_u32(tracker.initialized, 1U, "first snapshot starts the CPU share tracking");
    expect_u32(
        (uint32_t)tasks[DIAGNOSTIC_TASK_SLOT_TELEMETRY].priority,
        (uint32_t)osPriorityError,
        "missing task reports no priority"
    );
    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_TELEMETRY].state, (uint32_t)osThreadError, "missing task reports no state");
    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_TELEMETRY].stack_min_free_bytes, 0U, "missing task reports no stack");

    set_task_status(90U, 750U);
    diagnostic_task_collect_tasks(tasks, &tracker, 2000U);
    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_TCPIP].runtime_percent, 25U, "CPU share between snapshots");
    expect_u32(tasks[DIAGNOSTIC_TASK_SLOT_TCPIP].stack_min_free_bytes, 360U, "lower high-water mark is reported");
    expect_u32(tracker.previous_total_runtime_ticks, 2000U, "run-time total is kept for the next snapshot");
    test_missing_task_name = NULL;
}

static void test_command_channel_mapping(void) {
    CommandChannelDiagnosticsDTO_t channel = {0};

    test_command_diagnostics = (CommandTaskDiagnostics){
        .connections_accepted = 31U,
        .connections_closed = 32U,
        .active_connection = 1U,
        .complete_frames_received = 40U,
        .invalid_frames = 126U,
        .buffer_max_bytes = 45U,
        .response_send_errors = 36U,
        .recv_errors = 50U,
        .decoder_errors = 51U,
        .acks_deferred = 52U,
        .last_error = -37,
    };
    test_command_decoder_diagnostics = (CommandDecoderDiagnostics){
        .commands_received = 69U,
        .commands_rejected = 70U,
        .last_error = -83,
    };
    diagnostic_task_collect_command_channel(&channel);
    expect_u32(channel.connections_accepted, 31U, "accepted connections");
    expect_u32(channel.connections_closed, 32U, "closed connections");
    expect_u32(channel.active_connection, 1U, "active connection");
    expect_u32(channel.frames_received, 40U, "frames received");
    expect_u32(channel.invalid_frames, 126U, "invalid frames");
    expect_u32(channel.rx_buffer_max_bytes, 45U, "receive buffer maximum");
    expect_u32(channel.recv_errors, 50U, "receive errors");
    expect_u32(channel.send_errors, 36U, "send errors");
    expect_u32(channel.decoder_errors, 51U, "decoder errors");
    expect_u32(channel.acks_deferred, 52U, "held ACKs");
    expect_u32((uint32_t)channel.last_error, (uint32_t)-37, "connection last error");
    expect_u32(channel.commands_received, 69U, "commands received");
    expect_u32(channel.commands_rejected, 70U, "commands rejected");
    expect_u32((uint32_t)channel.decoder_last_error, (uint32_t)-83, "decoder last error");
}

static void test_ethernet_mapping(void) {
    EthernetDiagnosticsDTO_t ethernet = {0};

    test_ethernet_diagnostics_result = HAL_OK;
    test_ethernet_port_diagnostics = (EthernetPortDiagnostics){
        .link_up = 1U,
        .speed = 100U,
        .duplex = 1U,
        .dma_error = 2U,
        .link_up_events = 3U,
        .link_down_events = 4U,
        .link_state_read_errors = 5U,
        .tx_errors = 6U,
        .tx_descriptor_overflows = 7U,
        .mutex_timeouts = 8U,
    };
    test_ethernet_rx_diagnostics = (EthernetRxDiagnostics){
        .hal_read_data_errors = 9U,
        .dma_receive_buffer_unavailable = 10U,
        .dropped_packets = 11U,
        .rx_pool_exhaustions = 12U,
        .rx_pool_max_used = 13U,
    };
    diagnostic_task_collect_ethernet(&ethernet);
    expect_u32(ethernet.link_up, 1U, "link state");
    expect_u32(ethernet.speed, 100U, "link speed");
    expect_u32(ethernet.duplex, 1U, "duplex");
    expect_u32(ethernet.link_up_events, 3U, "link up events");
    expect_u32(ethernet.link_down_events, 4U, "link down events");
    expect_u32(ethernet.link_state_read_errors, 5U, "PHY read errors");
    expect_u32(ethernet.dma_error, 2U, "DMA error");
    expect_u32(ethernet.tx_errors, 6U, "transmit errors");
    expect_u32(ethernet.tx_descriptor_overflows, 7U, "transmit descriptor overflows");
    expect_u32(ethernet.mutex_timeouts, 8U, "port mutex timeouts");
    expect_u32(ethernet.rx_pool_max_used, 13U, "receive pool maximum");
    expect_u32(ethernet.rx_pool_exhaustions, 12U, "receive pool exhaustions");
    expect_u32(ethernet.rx_dma_buffer_unavailable, 10U, "receive DMA buffer unavailable");
    expect_u32(ethernet.rx_dropped_packets, 11U, "dropped packets");
    expect_u32(ethernet.rx_read_errors, 9U, "receive read errors");

    /* A busy port keeps the previous port values; receive values are still refreshed. */
    test_ethernet_diagnostics_result = -1;
    test_ethernet_port_diagnostics = (EthernetPortDiagnostics){0};
    test_ethernet_rx_diagnostics.dropped_packets = 21U;
    diagnostic_task_collect_ethernet(&ethernet);
    expect_u32(ethernet.link_up, 1U, "busy port keeps the link state");
    expect_u32(ethernet.mutex_timeouts, 8U, "busy port keeps its counters");
    expect_u32(ethernet.rx_dropped_packets, 21U, "receive values are refreshed anyway");
    test_ethernet_diagnostics_result = HAL_OK;
}

static void reset_network_fixture(void) {
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_stop_on_delay = 0U;
    test_stop_on_second_new = 0U;
    test_stop_on_second_accept = 0U;
    test_succeed_after_first_write = 0U;
    test_netconn_new_calls = 0U;
    test_netconn_bind_calls = 0U;
    test_netconn_listen_calls = 0U;
    test_netconn_accept_calls = 0U;
    test_netconn_write_calls = 0U;
    test_netconn_close_calls = 0U;
    test_netconn_delete_calls = 0U;
    test_event_flags_result = 0U;
    test_new_returns_null = 0U;
    test_ethernet_diagnostics_result = HAL_OK;
    test_bind_result = ERR_OK;
    test_listen_result = ERR_OK;
    test_accept_result = ERR_OK;
    test_write_result = ERR_OK;
    test_bytes_written = sizeof(DiagnosticFrameDTO_t);
    test_captured_write_size = 0U;
    (void)memset(&test_captured_frame, 0, sizeof(test_captured_frame));
    test_total_runtime_ticks = 0U;
    test_missing_task_name = NULL;
    test_ethernet_port_diagnostics = (EthernetPortDiagnostics){0};
    test_ethernet_rx_diagnostics = (EthernetRxDiagnostics){0};
    test_command_diagnostics = (CommandTaskDiagnostics){0};
    test_command_decoder_diagnostics = (CommandDecoderDiagnostics){0};
    test_motion_task_diagnostics = (MotionTaskDiagnostics){0};
    test_step_generator_diagnostics = (StepGeneratorTaskDiagnostics){0};
    test_command_queue_diagnostics = (CommandQueueDiagnostics_t){0};
    test_plan_buffer_diagnostics = (PlanBufferDiagnostics_t){0};
    test_execution_buffer_level = 0U;
    test_underruns = 0U;
}

static int run_diagnostic_task_until_jump(NetworkTaskContext *context) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        DiagnosticTask_test_unused(context);
    }
    test_jump_active = 0U;
    return jumped;
}

static void test_diagnostic_frame_contents(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_network_fixture();
    test_stop_on_delay = 1U;
    set_task_status(100U, 500U);
    test_total_runtime_ticks = 1000U;
    test_command_diagnostics = (CommandTaskDiagnostics){.acks_deferred = 7U};
    test_command_queue_diagnostics = (CommandQueueDiagnostics_t){
        .queued = 73U,
        .max_queued = 91U,
        .enqueued_total = 123U,
        .full_rejections = 4U,
        .enqueue_errors = 5U,
    };
    test_plan_buffer_diagnostics = (PlanBufferDiagnostics_t){
        .queued = 37U,
        .max_queued = 80U,
        .enqueued_total = 1234U,
        .dequeued_total = 1197U,
        .full_rejections = 2U,
        .rejected_operations = 1U,
        .cancelled_blocks = 5U,
    };
    test_motion_task_diagnostics = (MotionTaskDiagnostics){.commands_dequeued = 14U, .last_sequence = 104U};
    test_step_generator_diagnostics = (StepGeneratorTaskDiagnostics){.blocks_started = 8U, .buffer_full_waits = 9U};
    test_execution_buffer_level = 300U;
    test_underruns = 2U;
    test_ethernet_port_diagnostics = (EthernetPortDiagnostics){.link_up = 1U};

    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "successful diagnostic frame path");
    expect_u32(test_netconn_write_calls, 1U, "diagnostic frame write");
    expect_u32(test_captured_write_size, sizeof(DiagnosticFrameDTO_t), "diagnostic frame write size");
    expect_u32(test_captured_frame.header.magic, NETWORK_PROTOCOL_MAGIC, "diagnostic frame magic");
    expect_u32(test_captured_frame.header.version, NETWORK_PROTOCOL_VERSION, "diagnostic frame version");
    expect_u32(test_captured_frame.header.message_type, NETWORK_MESSAGE_TYPE_DIAGNOSTIC_SNAPSHOT, "frame type");
    expect_u32(test_captured_frame.header.header_length, sizeof(NetworkFrameHeaderDTO_t), "header length");
    expect_u32(test_captured_frame.header.payload_length, sizeof(DiagnosticPayloadDTO_t), "payload length");

    const DiagnosticPayloadDTO_t *payload = &test_captured_frame.payload;
    expect_u32(payload->sequence, 1U, "first diagnostic frame sequence");
    expect_u32(payload->system.tick_count, 5007U, "system snapshot");
    expect_u32(payload->tasks[DIAGNOSTIC_TASK_SLOT_DIAGNOSTIC].stack_min_free_bytes, 400U, "own task is reported");
    expect_u32(payload->tasks[DIAGNOSTIC_TASK_SLOT_STEP_GENERATOR].state, 2U, "step generator task is reported");
    expect_u32(payload->command_channel.acks_deferred, 7U, "command channel");
    expect_u32(payload->command_queue.queued, 73U, "command queue occupancy");
    expect_u32(payload->command_queue.max_queued, 91U, "command queue maximum");
    expect_u32(payload->command_queue.enqueued_total, 123U, "command queue accepted total");
    expect_u32(payload->command_queue.full_rejections, 4U, "command queue full rejections");
    expect_u32(payload->command_queue.enqueue_errors, 5U, "command queue errors");
    expect_u32(payload->motion.commands_dequeued, 14U, "MotionTask counters");
    expect_u32(payload->motion.last_sequence, 104U, "MotionTask last sequence");
    expect_u32(payload->plan_buffer.queued, 37U, "plan buffer occupancy");
    expect_u32(payload->plan_buffer.max_queued, 80U, "plan buffer maximum");
    expect_u32(payload->plan_buffer.enqueued_total, 1234U, "plan buffer accepted total");
    expect_u32(payload->plan_buffer.dequeued_total, 1197U, "plan buffer consumed total");
    expect_u32(payload->plan_buffer.full_rejections, 2U, "plan buffer full rejections");
    expect_u32(payload->plan_buffer.rejected_operations, 1U, "plan buffer rejected operations");
    expect_u32(payload->plan_buffer.cancelled_blocks, 5U, "plan buffer cancelled blocks");
    expect_u32(payload->step_generator.blocks_started, 8U, "step generator counters");
    expect_u32(payload->step_generator.buffer_full_waits, 9U, "step generator waits");
    expect_u32(payload->execution_buffer.level, 300U, "execution buffer level");
    expect_u32(payload->execution_buffer.underruns, 2U, "execution buffer underruns");
    expect_u32(payload->x_axis.travel_um, 301000U, "X axis");
    expect_u32((uint32_t)payload->x_axis.scale_position_um, (uint32_t)-5, "X axis scale");
    expect_u32(payload->y_axis.travel_um, 402000U, "Y axis");
    expect_u32((uint32_t)payload->y_axis.scale_position_um, 7U, "Y axis scale");
    expect_u32(payload->z_axis.travel_um, 50300U, "Z axis");
    expect_u32((uint32_t)payload->z_axis.scale_position_um, (uint32_t)-9, "Z axis scale");
    expect_u32(payload->ethernet.link_up, 1U, "Ethernet");
    expect_u32(payload->lwip.mailbox_errors, 3U, "LwIP");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(payload->step_timing[axis].status, MOTION_AXIS_HARDWARE_STATUS_RUNNING, "step timing status");
        expect_u32(payload->step_timing[axis].timing_faults, 10U + axis, "step timing faults follow the axis order");
        expect_u32(payload->step_timing[axis].isr_max_ns, 1000U + axis, "step timing ISR maximum");
        expect_u32(payload->step_timing[axis].min_slack_ns, 5000U + axis, "step timing slack");
    }
}

static void test_diagnostic_task_network_paths(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    NetworkTaskContext context_without_lwip_flags = {.lwip_flags = NULL};

    reset_network_fixture();
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(NULL), 1U, "null task context must stop the task");
    expect_u32(test_error_handler_calls, 1U, "null task context error");

    reset_network_fixture();
    expect_u32(
        (uint32_t)run_diagnostic_task_until_jump(&context_without_lwip_flags),
        1U,
        "null LwIP event flags must stop the task"
    );
    expect_u32(test_error_handler_calls, 1U, "null LwIP event flags error");
    expect_u32(test_netconn_new_calls, 0U, "no socket without LwIP event flags");

    reset_network_fixture();
    test_event_flags_result = osFlagsErrorParameter;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "LwIP wait error must stop the task");
    expect_u32(test_error_handler_calls, 1U, "LwIP wait error handling");
    expect_u32(test_netconn_new_calls, 0U, "no socket after LwIP wait error");

    reset_network_fixture();
    test_new_returns_null = 1U;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "socket allocation error path");
    expect_u32(test_netconn_new_calls, 1U, "socket allocation attempt");
    expect_u32(test_netconn_bind_calls, 0U, "no bind after allocation error");

    reset_network_fixture();
    test_bind_result = ERR_MEM;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "bind error path");
    expect_u32(test_netconn_bind_calls, 1U, "bind attempt");
    expect_u32(test_netconn_listen_calls, 0U, "no listen after bind error");
    expect_u32(test_netconn_delete_calls, 1U, "listener deleted after bind error");

    reset_network_fixture();
    test_listen_result = ERR_MEM;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "listen error path");
    expect_u32(test_netconn_listen_calls, 1U, "listen attempt");
    expect_u32(test_netconn_accept_calls, 0U, "no accept after listen error");
    expect_u32(test_netconn_delete_calls, 1U, "listener deleted after listen error");

    reset_network_fixture();
    test_accept_result = ERR_MEM;
    test_stop_on_second_new = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "accept error path");
    expect_u32(test_netconn_accept_calls, 1U, "accept attempt");
    expect_u32(test_netconn_close_calls, 1U, "listener closed after accept error");

    reset_network_fixture();
    test_write_result = ERR_BUF;
    test_bytes_written = 3U;
    test_stop_on_second_accept = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "partial diagnostic frame path");
    expect_u32(test_netconn_write_calls, 1U, "partial frame write");
    expect_u32(test_netconn_close_calls, 1U, "client closed after partial frame");
    expect_u32(test_netconn_delete_calls, 1U, "client deleted after partial frame");
    expect_u32(test_netconn_accept_calls, 2U, "accept retried after partial frame");

    reset_network_fixture();
    test_write_result = ERR_BUF;
    test_succeed_after_first_write = 1U;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "write error path");
    expect_u32(test_netconn_write_calls, 2U, "client is accepted again after a write error");
    expect_u32(test_captured_frame.payload.sequence, 2U, "sequence keeps counting across clients");

    reset_network_fixture();
    test_bytes_written = 3U;
    test_succeed_after_first_write = 1U;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "short successful write path");
    expect_u32(test_netconn_write_calls, 2U, "a short write closes the client");
}

int main(void) {
    test_runtime_percent_calculation();
    test_task_slots();
    test_command_channel_mapping();
    test_ethernet_mapping();
    test_diagnostic_frame_contents();
    test_diagnostic_task_network_paths();

    expect_u32(test_error_handler_calls, 0U, "diagnostic task helper tests must not call Error_Handler");

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Diagnostic task unit tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Diagnostic task unit tests passed.\n");
    return 0;
}

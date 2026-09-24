#include "diagnostic_dto.h"
#include "ethernetif.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "task_context.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;
static TaskStatus_t test_task_status;
static TaskHandle_t test_current_task = (TaskHandle_t)(uintptr_t)1U;
static uint32_t test_error_handler_calls;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint32_t test_stop_on_delay;
static uint32_t test_stop_on_second_new;
static uint32_t test_stop_on_second_accept;
static uint32_t test_netconn_new_calls;
static uint32_t test_netconn_bind_calls;
static uint32_t test_netconn_listen_calls;
static uint32_t test_netconn_accept_calls;
static uint32_t test_netconn_write_calls;
static uint32_t test_netconn_close_calls;
static uint32_t test_netconn_delete_calls;
static uint32_t test_event_flags_result;
static uint32_t test_new_returns_null;
static err_t test_bind_result;
static err_t test_listen_result;
static err_t test_accept_result;
static err_t test_write_result;
static size_t test_bytes_written;
static size_t test_captured_write_size;
static DiagnosticFrameDTO_t test_captured_frame;

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

TaskHandle_t xTaskGetCurrentTaskHandle(void) { return test_current_task; }

TaskHandle_t xTaskGetHandle(const char *name) {
    (void)name;
    return test_current_task;
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

void ethernetif_get_rx_diagnostics(EthernetRxDiagnostics *diagnostics) {
    (void)memset(diagnostics, 0, sizeof(*diagnostics));
}

int EthernetPort_GetDiagnostics(EthernetPortDiagnostics *diagnostics) {
    (void)memset(diagnostics, 0, sizeof(*diagnostics));
    return HAL_OK;
}

void system_diagnostics_collect(SystemDiagnostics *diagnostics) { (void)memset(diagnostics, 0, sizeof(*diagnostics)); }

void lwip_diagnostics_collect(LwipDiagnostics *diagnostics) { (void)memset(diagnostics, 0, sizeof(*diagnostics)); }

void startup_task_get_diagnostics(StartupTaskDiagnostics *diagnostics) {
    (void)memset(diagnostics, 0, sizeof(*diagnostics));
}

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

static void test_counter_helpers(void) {
    uint32_t counter = 0U;

    diagnostic_counter_increment(&counter);
    expect_u32(counter, 1U, "diagnostic counter increment");

    counter = UINT32_MAX;
    diagnostic_counter_increment(&counter);
    expect_u32(counter, UINT32_MAX, "diagnostic counter saturation");

    counter = 10U;
    diagnostic_counter_add(&counter, 5U);
    expect_u32(counter, 15U, "diagnostic counter add");

    counter = UINT32_MAX - 2U;
    diagnostic_counter_add(&counter, 5U);
    expect_u32(counter, UINT32_MAX, "diagnostic counter add saturation");

    counter = 0U;
    diagnostic_counter_add(&counter, (size_t)UINT32_MAX + 1U);
    expect_u32(counter, UINT32_MAX, "diagnostic counter size overflow saturation");
}

static void test_runtime_percent_calculation(void) {
    TaskRuntimeSample sample = {0};
    TaskHandle_t task_handle = (TaskHandle_t)(uintptr_t)2U;

    runtime_percent_tracker = (RuntimePercentTracker){0};
    expect_u32(
        diagnostic_task_calculate_runtime_percent(&sample, task_handle, 100U, 1000U),
        0U,
        "initial task runtime percentage"
    );

    diagnostic_task_commit_runtime_sample(1000U);
    expect_u32(
        diagnostic_task_calculate_runtime_percent(&sample, task_handle, 150U, 1100U),
        50U,
        "task runtime percentage"
    );

    diagnostic_task_commit_runtime_sample(1100U);
    expect_u32(
        diagnostic_task_calculate_runtime_percent(&sample, task_handle, 400U, 1200U),
        100U,
        "task runtime percentage upper bound"
    );

    expect_u32(
        diagnostic_task_calculate_runtime_percent(&sample, NULL, 0U, 1200U),
        0U,
        "runtime percentage without task"
    );
}

static void test_task_diagnostics_mapping(void) {
    TaskDiagnosticsDTO_t diagnostics = {0};
    TaskRuntimeSample sample = {0};
    uint32_t minimum_free_bytes = UINT32_MAX;

    test_task_status.pxStackBase = (StackType_t *)(uintptr_t)0x12340000U;
    test_task_status.uxCurrentPriority = 24U;
    test_task_status.uxBasePriority = 23U;
    test_task_status.eCurrentState = 2U;
    test_task_status.usStackHighWaterMark = 100U;
    test_task_status.ulRunTimeCounter = 500U;

    diagnostic_network_task_fill_stats(&diagnostics, test_current_task, &sample, 1024U, &minimum_free_bytes, 1000U);

    expect_u32(diagnostics.stack_size_bytes, 1024U, "task stack size");
    expect_u32(diagnostics.stack_free_bytes, 400U, "task free stack");
    expect_u32(diagnostics.stack_min_free_bytes, 400U, "task minimum free stack");
    expect_u32(diagnostics.stack_base_address, 0x12340000U, "task stack base");
    expect_u32((uint32_t)diagnostics.priority, 24U, "task priority");
    expect_u32((uint32_t)diagnostics.base_priority, 23U, "task base priority");
    expect_u32(diagnostics.state, 2U, "task state");
    expect_u32(diagnostics.runtime_ticks, 500U, "task runtime ticks");

    diagnostic_network_task_fill_stats(&diagnostics, NULL, &sample, 1024U, &minimum_free_bytes, 1000U);
    expect_u32((uint32_t)diagnostics.priority, (uint32_t)osPriorityError, "missing task priority");
    expect_u32(diagnostics.state, (uint32_t)osThreadError, "missing task state");
    expect_u32(diagnostics.stack_free_bytes, 0U, "missing task free stack");
}

static void test_diagnostic_task_stats_mapping(void) {
    DiagnosticPayloadDTO_t payload = {0};
    DiagnosticTaskRuntime runtime = {
        .stack_min_free_bytes = 0U,
        .connections_accepted = 1U,
        .connections_closed = 2U,
        .active_connection = 1U,
        .send_attempts = 3U,
        .send_successes = 4U,
        .send_errors = 5U,
        .partial_writes = 6U,
        .bytes_sent = 7U,
        .netconn_alloc_errors = 8U,
        .bind_errors = 9U,
        .listen_errors = 10U,
        .accept_errors = 11U,
        .snapshot_errors = 12U,
        .last_error = -4,
    };

    runtime_percent_tracker = (RuntimePercentTracker){0};
    test_task_status.pxStackBase = (StackType_t *)(uintptr_t)0x12340000U;
    test_task_status.uxCurrentPriority = 24U;
    test_task_status.uxBasePriority = 23U;
    test_task_status.eCurrentState = 2U;
    test_task_status.usStackHighWaterMark = 100U;
    test_task_status.ulRunTimeCounter = 500U;
    payload.system.freertos.total_runtime_ticks = 1000U;

    diagnostic_task_fill_stats(&payload, &runtime);

    expect_u32(
        payload.diagnostic_task.runtime.stack_size_bytes,
        DIAGNOSTIC_TASK_STACK_SIZE_BYTES,
        "diagnostic task stack size"
    );
    expect_u32(payload.diagnostic_task.runtime.stack_free_bytes, 400U, "diagnostic task free stack");
    expect_u32(payload.diagnostic_task.runtime.stack_min_free_bytes, 400U, "diagnostic task minimum free stack");
    expect_u32(payload.diagnostic_task.runtime.stack_base_address, 0x12340000U, "diagnostic task stack base");
    expect_u32((uint32_t)payload.diagnostic_task.runtime.priority, 24U, "diagnostic task priority");
    expect_u32(payload.diagnostic_task.runtime.runtime_ticks, 500U, "diagnostic task runtime ticks");
    expect_u32(payload.diagnostic_task.port, DIAGNOSTIC_NETWORK_TASK_PORT, "diagnostic task port");
    expect_u32(payload.diagnostic_task.interval_ms, DIAGNOSTIC_NETWORK_TASK_INTERVAL_MS, "diagnostic task interval");
    expect_u32(payload.diagnostic_task.send_timeout_ms, DIAGNOSTIC_NETWORK_SEND_TIMEOUT_MS, "diagnostic task timeout");
    expect_u32(payload.diagnostic_task.connections_accepted, 1U, "diagnostic connections accepted");
    expect_u32(payload.diagnostic_task.connections_closed, 2U, "diagnostic connections closed");
    expect_u32(payload.diagnostic_task.active_connection, 1U, "diagnostic active connection");
    expect_u32(payload.diagnostic_task.send_attempts, 3U, "diagnostic send attempts");
    expect_u32(payload.diagnostic_task.send_successes, 4U, "diagnostic send successes");
    expect_u32(payload.diagnostic_task.send_errors, 5U, "diagnostic send errors");
    expect_u32(payload.diagnostic_task.partial_writes, 6U, "diagnostic partial writes");
    expect_u32(payload.diagnostic_task.bytes_sent, 7U, "diagnostic bytes sent");
    expect_u32(payload.diagnostic_task.netconn_alloc_errors, 8U, "diagnostic netconn allocation errors");
    expect_u32(payload.diagnostic_task.bind_errors, 9U, "diagnostic bind errors");
    expect_u32(payload.diagnostic_task.listen_errors, 10U, "diagnostic listen errors");
    expect_u32(payload.diagnostic_task.accept_errors, 11U, "diagnostic accept errors");
    expect_u32(payload.diagnostic_task.snapshot_errors, 12U, "diagnostic snapshot errors");
    expect_u32((uint32_t)payload.diagnostic_task.last_error, (uint32_t)-4, "diagnostic last error");

    diagnostic_record_error(&runtime, ERR_BUF);
    expect_u32((uint32_t)runtime.last_error, (uint32_t)ERR_BUF, "diagnostic error recording");
}

static void reset_network_fixture(void) {
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_stop_on_delay = 0U;
    test_stop_on_second_new = 0U;
    test_stop_on_second_accept = 0U;
    test_netconn_new_calls = 0U;
    test_netconn_bind_calls = 0U;
    test_netconn_listen_calls = 0U;
    test_netconn_accept_calls = 0U;
    test_netconn_write_calls = 0U;
    test_netconn_close_calls = 0U;
    test_netconn_delete_calls = 0U;
    test_event_flags_result = 0U;
    test_new_returns_null = 0U;
    test_bind_result = ERR_OK;
    test_listen_result = ERR_OK;
    test_accept_result = ERR_OK;
    test_write_result = ERR_OK;
    test_bytes_written = sizeof(DiagnosticFrameDTO_t);
    test_captured_write_size = 0U;
    (void)memset(&test_captured_frame, 0, sizeof(test_captured_frame));
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

static void test_diagnostic_task_network_paths(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_network_fixture();
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(NULL), 1U, "null task context must stop the task");
    expect_u32(test_error_handler_calls, 1U, "null task context error");

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
    expect_u32(test_netconn_new_calls, 1U, "socket allocation before bind error");
    expect_u32(test_netconn_bind_calls, 1U, "bind attempt");
    expect_u32(test_netconn_listen_calls, 0U, "no listen after bind error");
    expect_u32(test_netconn_delete_calls, 1U, "listener deleted after bind error");

    reset_network_fixture();
    test_listen_result = ERR_MEM;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "listen error path");
    expect_u32(test_netconn_listen_calls, 1U, "listen attempt");
    expect_u32(test_netconn_accept_calls, 0U, "no accept after listen error");

    reset_network_fixture();
    test_accept_result = ERR_MEM;
    test_stop_on_second_new = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "accept error path");
    expect_u32(test_netconn_accept_calls, 1U, "accept attempt");
    expect_u32(test_netconn_close_calls, 1U, "listener closed after accept error");

    reset_network_fixture();
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "successful diagnostic frame path");
    expect_u32(test_netconn_write_calls, 1U, "diagnostic frame write");
    expect_u32(test_captured_write_size, sizeof(DiagnosticFrameDTO_t), "diagnostic frame write size");
    expect_u32(test_captured_frame.header.magic, NETWORK_PROTOCOL_MAGIC, "diagnostic frame magic");
    expect_u32(test_captured_frame.header.version, NETWORK_PROTOCOL_VERSION, "diagnostic frame version");
    expect_u32(
        test_captured_frame.header.message_type,
        NETWORK_MESSAGE_TYPE_DIAGNOSTIC_SNAPSHOT,
        "diagnostic frame type"
    );
    expect_u32(
        test_captured_frame.header.header_length,
        sizeof(NetworkFrameHeaderDTO_t),
        "diagnostic frame header length"
    );
    expect_u32(
        test_captured_frame.header.payload_length,
        sizeof(DiagnosticPayloadDTO_t),
        "diagnostic frame payload length"
    );
    expect_u32(test_captured_frame.payload.sequence, 1U, "first diagnostic frame sequence");

    reset_network_fixture();
    test_write_result = ERR_BUF;
    test_bytes_written = 3U;
    test_stop_on_second_accept = 1U;
    expect_u32((uint32_t)run_diagnostic_task_until_jump(&context), 1U, "partial diagnostic frame path");
    expect_u32(test_netconn_write_calls, 1U, "partial frame write");
    expect_u32(test_captured_write_size, sizeof(DiagnosticFrameDTO_t), "partial frame attempted size");
    expect_u32(test_netconn_close_calls, 1U, "client closed after partial frame");
    expect_u32(test_netconn_delete_calls, 1U, "client deleted after partial frame");
    expect_u32(test_netconn_accept_calls, 2U, "accept retried after partial frame");
}

int main(void) {
    test_counter_helpers();
    test_runtime_percent_calculation();
    test_task_diagnostics_mapping();
    test_diagnostic_task_stats_mapping();
    test_diagnostic_task_network_paths();

    expect_u32(test_error_handler_calls, 0U, "diagnostic task helper tests must not call Error_Handler");

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Diagnostic task unit tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Diagnostic task unit tests passed.\n");
    return 0;
}

#include "cmsis_os.h"
#include "encoder.h"
#include "task_context.h"
#include "telemetry_dto.h"
#include "telemetry_task.h"
#include "telemetry_test_hooks.h"

#include "lwip/api.h"
#include "lwip/err.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct netconn {
    uint32_t id;
};

static uint32_t test_failures;
static uint32_t test_event_flags_wait_calls;
static uint32_t test_netconn_new_calls;
static uint32_t test_netconn_bind_calls;
static uint32_t test_netconn_listen_calls;
static uint32_t test_netconn_accept_calls;
static uint32_t test_netconn_sendtimeout_calls;
static uint32_t test_netconn_write_calls;
static uint32_t test_netconn_close_calls;
static uint32_t test_netconn_delete_calls;
static uint32_t test_error_handler_calls;
static uint32_t test_event_flags_wait_result;
static uint32_t test_netconn_new_fail_on_call;
static uint32_t test_netconn_accept_fail_on_call;
static uint32_t test_longjmp_on_accept_call;
static err_t test_netconn_accept_error;
static err_t test_netconn_bind_result;
static err_t test_netconn_listen_result;
static uint8_t test_write_override;
static err_t test_write_override_result;
static size_t test_write_override_bytes;
static uint8_t test_jump_active;
static jmp_buf test_jump_buffer;
static TelemetryFrameDTO_t test_first_frame;
static TelemetryFrameDTO_t test_success_frame;
static struct netconn test_listener = {.id = 1U};
static struct netconn test_client = {.id = 2U};
static const int32_t test_encoder_ox_coordinate = -100;
static const int32_t test_encoder_oy_coordinate = 200;
static const int32_t test_encoder_oz_coordinate = -300;

int32_t encoder_ox_get_coordinate(void) { return test_encoder_ox_coordinate; }

int32_t encoder_oy_get_coordinate(void) { return test_encoder_oy_coordinate; }

int32_t encoder_oz_get_coordinate(void) { return test_encoder_oz_coordinate; }

uint32_t osEventFlagsWait(osEventFlagsId_t event_flags_id, uint32_t flags, uint32_t options, uint32_t timeout) {
    (void)event_flags_id;
    (void)flags;
    (void)options;
    (void)timeout;
    ++test_event_flags_wait_calls;
    return test_event_flags_wait_result;
}

uint32_t osDelay(uint32_t milliseconds) {
    (void)milliseconds;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
    return 0U;
}

void Error_Handler(void) {
    ++test_error_handler_calls;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
}

struct netconn *netconn_new(int type) {
    (void)type;
    ++test_netconn_new_calls;
    if (test_netconn_new_calls == test_netconn_new_fail_on_call) {
        return NULL;
    }
    return &test_listener;
}

err_t netconn_bind(struct netconn *connection, void *address, uint16_t port) {
    (void)connection;
    (void)address;
    (void)port;
    ++test_netconn_bind_calls;
    return test_netconn_bind_result;
}

err_t netconn_listen(struct netconn *connection) {
    (void)connection;
    ++test_netconn_listen_calls;
    return test_netconn_listen_result;
}

err_t netconn_accept(struct netconn *connection, struct netconn **new_connection) {
    (void)connection;
    ++test_netconn_accept_calls;
    if ((test_netconn_accept_calls == test_longjmp_on_accept_call) && (test_jump_active != 0U)) {
        longjmp(test_jump_buffer, 1);
    }
    if (test_netconn_accept_calls == test_netconn_accept_fail_on_call) {
        return test_netconn_accept_error;
    }
    if (test_netconn_accept_calls > 2U) {
        return ERR_BUF;
    }

    *new_connection = &test_client;
    return ERR_OK;
}

void netconn_set_sendtimeout(struct netconn *connection, int timeout) {
    (void)connection;
    (void)timeout;
    ++test_netconn_sendtimeout_calls;
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
    if (test_write_override != 0U) {
        (void)memcpy(&test_success_frame, data, sizeof(test_success_frame));
        *bytes_written = test_write_override_bytes;
        return test_write_override_result;
    }
    if (test_netconn_write_calls == 1U) {
        (void)memcpy(&test_first_frame, data, sizeof(test_first_frame));
        *bytes_written = 0U;
        return ERR_BUF;
    }

    (void)memcpy(&test_success_frame, data, sizeof(test_success_frame));
    *bytes_written = size;
    return ERR_OK;
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

static void expect_i32(int32_t actual, int32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %ld, got %ld)\n", message, (long)expected, (long)actual);
    }
}

static void expect_frame(const TelemetryFrameDTO_t *frame, uint32_t sequence, const char *prefix) {
    expect_u32(frame->header.message_type, NETWORK_MESSAGE_TYPE_TELEMETRY, prefix);
    expect_u32(frame->header.payload_length, sizeof(TelemetryPayloadDTO_t), "telemetry task payload length");
    expect_u32(frame->payload.sequence, sequence, "telemetry task sequence");
    expect_i32(frame->payload.x, test_encoder_ox_coordinate, "telemetry task x");
    expect_i32(frame->payload.y, test_encoder_oy_coordinate, "telemetry task y");
    expect_i32(frame->payload.z, test_encoder_oz_coordinate, "telemetry task z");
    expect_u32(frame->payload.is_test_data, 0U, "telemetry task test data flag");
}

static void run_until_delay(NetworkTaskContext *context) {
    test_jump_active = 1U;
    if (setjmp(test_jump_buffer) == 0) {
        TelemetryTask(context);
    }
    test_jump_active = 0U;
}

static int run_reconnect_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_jump_active = 1U;
    if (setjmp(test_jump_buffer) == 0) {
        TelemetryTask(&context);
    }
    test_jump_active = 0U;

    expect_u32(test_event_flags_wait_calls, 1U, "telemetry task event wait calls");
    expect_u32(test_netconn_new_calls, 1U, "telemetry listener creation calls");
    expect_u32(test_netconn_bind_calls, 1U, "telemetry bind calls");
    expect_u32(test_netconn_listen_calls, 1U, "telemetry listen calls");
    expect_u32(test_netconn_accept_calls, 2U, "telemetry accept calls");
    expect_u32(test_netconn_sendtimeout_calls, 2U, "telemetry send timeout calls");
    expect_u32(test_netconn_write_calls, 2U, "telemetry write calls");
    expect_u32(test_netconn_close_calls, 1U, "telemetry client close calls");
    expect_u32(test_netconn_delete_calls, 1U, "telemetry client delete calls");
    expect_u32(test_error_handler_calls, 0U, "telemetry Error_Handler calls");
    expect_frame(&test_first_frame, 1U, "telemetry first frame type");
    expect_frame(&test_success_frame, 2U, "telemetry recovered frame type");
    telemetry_task_get_diagnostics(NULL);
    telemetry_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.port, TELEMETRY_NETWORK_TASK_PORT, "telemetry diagnostics port");
    expect_u32(diagnostics.interval_ms, TELEMETRY_NETWORK_TASK_INTERVAL_MS, "telemetry diagnostics interval");
    expect_u32(diagnostics.send_timeout_ms, TELEMETRY_NETWORK_SEND_TIMEOUT_MS, "telemetry diagnostics send timeout");
    expect_u32(diagnostics.connections_accepted, 2U, "telemetry diagnostics accepted connections");
    expect_u32(diagnostics.connections_closed, 1U, "telemetry diagnostics closed connections");
    expect_u32(diagnostics.active_connection, 1U, "telemetry diagnostics active connection");
    expect_u32(diagnostics.send_attempts, 2U, "telemetry diagnostics send attempts");
    expect_u32(diagnostics.send_successes, 1U, "telemetry diagnostics send successes");
    expect_u32(diagnostics.send_errors, 1U, "telemetry diagnostics send errors");
    expect_u32(diagnostics.partial_writes, 1U, "telemetry diagnostics partial writes");
    expect_u32(diagnostics.bytes_sent, sizeof(TelemetryFrameDTO_t), "telemetry diagnostics bytes sent");
    expect_u32(diagnostics.netconn_alloc_errors, 0U, "telemetry diagnostics allocation errors");
    expect_u32(diagnostics.bind_errors, 0U, "telemetry diagnostics bind errors");
    expect_u32(diagnostics.listen_errors, 0U, "telemetry diagnostics listen errors");
    expect_u32(diagnostics.accept_errors, 0U, "telemetry diagnostics accept errors");
    expect_i32(diagnostics.last_error, ERR_BUF, "telemetry diagnostics last error");
    return test_failures == 0U ? 0 : 1;
}

static int run_wait_error_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    test_event_flags_wait_result = osFlagsErrorParameter;

    TelemetryTask(&context);

    expect_u32(test_event_flags_wait_calls, 1U, "telemetry wait error event calls");
    expect_u32(test_netconn_new_calls, 0U, "telemetry wait error listener creation calls");
    expect_u32(test_error_handler_calls, 1U, "telemetry wait error handler calls");
    return test_failures == 0U ? 0 : 1;
}

static int run_null_context_case(void) {
    TelemetryTask(NULL);

    expect_u32(test_event_flags_wait_calls, 0U, "telemetry null context event calls");
    expect_u32(test_netconn_new_calls, 0U, "telemetry null context listener creation calls");
    expect_u32(test_error_handler_calls, 1U, "telemetry null context handler calls");
    return test_failures == 0U ? 0 : 1;
}

static int run_null_flag_handle_case(void) {
    NetworkTaskContext context = {0};

    TelemetryTask(&context);

    expect_u32(test_event_flags_wait_calls, 0U, "null flags event wait calls");
    expect_u32(test_netconn_new_calls, 0U, "null flags listener creation calls");
    expect_u32(test_error_handler_calls, 1U, "null flags Error_Handler calls");
    return test_failures == 0U ? 0 : 1;
}

static int run_allocation_error_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_netconn_new_fail_on_call = 1U;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(test_event_flags_wait_calls, 1U, "allocation error event wait calls");
    expect_u32(test_netconn_new_calls, 1U, "allocation error listener creation calls");
    expect_u32(test_netconn_bind_calls, 0U, "allocation error bind calls");
    expect_u32(diagnostics.netconn_alloc_errors, 1U, "telemetry allocation error count");
    expect_i32(diagnostics.last_error, ERR_MEM, "telemetry allocation error last error");
    return test_failures == 0U ? 0 : 1;
}

static int run_bind_error_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_netconn_bind_result = ERR_CONN;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(test_netconn_bind_calls, 1U, "bind error bind calls");
    expect_u32(test_netconn_listen_calls, 0U, "bind error listen calls");
    expect_u32(test_netconn_close_calls, 0U, "bind error close calls");
    expect_u32(test_netconn_delete_calls, 1U, "bind error delete calls");
    expect_u32(diagnostics.bind_errors, 1U, "telemetry bind error count");
    expect_i32(diagnostics.last_error, ERR_CONN, "telemetry bind last error");
    return test_failures == 0U ? 0 : 1;
}

static int run_listen_error_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_netconn_listen_result = ERR_TIMEOUT;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(test_netconn_bind_calls, 1U, "listen error bind calls");
    expect_u32(test_netconn_listen_calls, 1U, "listen error listen calls");
    expect_u32(test_netconn_close_calls, 1U, "listen error close calls");
    expect_u32(test_netconn_delete_calls, 1U, "listen error delete calls");
    expect_u32(diagnostics.listen_errors, 1U, "telemetry listen error count");
    expect_i32(diagnostics.last_error, ERR_TIMEOUT, "telemetry listen last error");
    return test_failures == 0U ? 0 : 1;
}

static int run_accept_error_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_netconn_accept_fail_on_call = 1U;
    test_netconn_accept_error = ERR_TIMEOUT;
    test_netconn_new_fail_on_call = 2U;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(test_netconn_accept_calls, 1U, "accept error accept calls");
    expect_u32(test_netconn_close_calls, 1U, "accept error listener close calls");
    expect_u32(test_netconn_delete_calls, 1U, "accept error listener delete calls");
    expect_u32(diagnostics.accept_errors, 1U, "telemetry accept error count");
    expect_u32(diagnostics.netconn_alloc_errors, 1U, "allocation retry after accept error");
    expect_i32(diagnostics.last_error, ERR_MEM, "last error after allocation retry");
    return test_failures == 0U ? 0 : 1;
}

static int run_short_write_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_write_override = 1U;
    test_write_override_result = ERR_OK;
    test_write_override_bytes = sizeof(TelemetryFrameDTO_t) - 1U;
    test_longjmp_on_accept_call = 2U;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(diagnostics.send_errors, 1U, "short write error count");
    expect_u32(diagnostics.partial_writes, 1U, "short write partial count");
    expect_u32(diagnostics.bytes_sent, sizeof(TelemetryFrameDTO_t) - 1U, "short write bytes sent");
    expect_i32(diagnostics.last_error, ERR_BUF, "short successful write last error");
    expect_u32(diagnostics.active_connection, 0U, "short write clears active connection");
    expect_u32(diagnostics.connections_closed, 1U, "short write closed connection count");
    expect_u32(test_netconn_close_calls, 1U, "short write client close");
    expect_u32(test_netconn_delete_calls, 1U, "short write client delete");
    return test_failures == 0U ? 0 : 1;
}

static int run_full_write_error_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_write_override = 1U;
    test_write_override_result = ERR_CONN;
    test_write_override_bytes = sizeof(TelemetryFrameDTO_t);
    test_longjmp_on_accept_call = 2U;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(diagnostics.send_errors, 1U, "full write transport error count");
    expect_u32(diagnostics.partial_writes, 0U, "full byte count is not a partial write");
    expect_u32(diagnostics.bytes_sent, sizeof(TelemetryFrameDTO_t), "full write bytes sent before error");
    expect_i32(diagnostics.last_error, ERR_CONN, "full write transport last error");
    expect_u32(diagnostics.active_connection, 0U, "full write error clears active connection");
    expect_u32(diagnostics.connections_closed, 1U, "full write error closed connection count");
    expect_u32(test_netconn_close_calls, 1U, "full write error client close");
    expect_u32(test_netconn_delete_calls, 1U, "full write error client delete");
    return test_failures == 0U ? 0 : 1;
}

static int run_counter_saturation_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    telemetry_task_test_seed_counters(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX - 10U);
    test_write_override = 1U;
    test_write_override_result = ERR_OK;
    test_write_override_bytes = sizeof(TelemetryFrameDTO_t);
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(diagnostics.connections_accepted, UINT32_MAX, "accepted connection counter saturation");
    expect_u32(diagnostics.send_attempts, UINT32_MAX, "send attempt counter saturation");
    expect_u32(diagnostics.send_successes, UINT32_MAX, "send success counter saturation");
    expect_u32(diagnostics.bytes_sent, UINT32_MAX, "bytes sent counter saturation");
    return test_failures == 0U ? 0 : 1;
}

static int run_oversized_write_count_case(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    TelemetryTaskDiagnostics diagnostics = {0};

    test_write_override = 1U;
    test_write_override_result = ERR_OK;
    /* Inject an impossible LwIP byte count to exercise the defensive size_t overflow guard. */
#if SIZE_MAX > UINT32_MAX
    test_write_override_bytes = (size_t)UINT32_MAX + 1U;
#else
    test_write_override_bytes = SIZE_MAX;
#endif
    test_longjmp_on_accept_call = 2U;
    run_until_delay(&context);
    telemetry_task_get_diagnostics(&diagnostics);

    expect_u32(diagnostics.bytes_sent, UINT32_MAX, "oversized write count saturates bytes counter");
    expect_u32(diagnostics.send_errors, 1U, "oversized write count send error");
    expect_u32(diagnostics.partial_writes, 1U, "oversized write count is not a full frame");
    expect_i32(diagnostics.last_error, ERR_BUF, "oversized successful write last error");
    expect_u32(diagnostics.active_connection, 0U, "oversized write count clears active connection");
    expect_u32(diagnostics.connections_closed, 1U, "oversized write count closed connection");
    return test_failures == 0U ? 0 : 1;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        (void)fprintf(stderr, "Expected one test case argument\n");
        return 2;
    }

    if (strcmp(argv[1], "reconnect") == 0) {
        return run_reconnect_case();
    }
    if (strcmp(argv[1], "wait_error") == 0) {
        return run_wait_error_case();
    }
    if (strcmp(argv[1], "null_context") == 0) {
        return run_null_context_case();
    }
    if (strcmp(argv[1], "null_flag_handle") == 0) {
        return run_null_flag_handle_case();
    }
    if (strcmp(argv[1], "allocation_error") == 0) {
        return run_allocation_error_case();
    }
    if (strcmp(argv[1], "bind_error") == 0) {
        return run_bind_error_case();
    }
    if (strcmp(argv[1], "listen_error") == 0) {
        return run_listen_error_case();
    }
    if (strcmp(argv[1], "accept_error") == 0) {
        return run_accept_error_case();
    }
    if (strcmp(argv[1], "short_write") == 0) {
        return run_short_write_case();
    }
    if (strcmp(argv[1], "full_write_error") == 0) {
        return run_full_write_error_case();
    }
    if (strcmp(argv[1], "counter_saturation") == 0) {
        return run_counter_saturation_case();
    }
    if (strcmp(argv[1], "oversized_write_count") == 0) {
        return run_oversized_write_count_case();
    }

    (void)fprintf(stderr, "Unknown test case: %s\n", argv[1]);
    return 2;
}

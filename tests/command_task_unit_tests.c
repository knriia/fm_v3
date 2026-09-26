#include "cmsis_os.h"
#include "command_protocol.h"
#include "command_task.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netbuf.h"
#include "task_context.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;
static uint32_t test_error_handler_calls;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint8_t test_stop_on_delay;
static uint8_t test_stop_on_second_new;
static uint8_t test_stop_on_second_accept;
static uint32_t test_netconn_new_calls;
static uint32_t test_netconn_bind_calls;
static uint32_t test_netconn_listen_calls;
static uint32_t test_netconn_accept_calls;
static uint32_t test_netconn_recv_calls;
static uint32_t test_netconn_write_calls;
static uint32_t test_netbuf_delete_calls;
static uint32_t test_netconn_close_calls;
static uint32_t test_netconn_delete_calls;
static uint32_t test_event_flags_result;
static uint8_t test_new_returns_null;
static err_t test_bind_result;
static err_t test_listen_result;
static err_t test_accept_result;
static err_t test_recv_result;
static err_t test_write_result;
static size_t test_bytes_written;
static uint8_t test_write_full_size;
static const uint8_t *test_receive_data;
static u16_t test_receive_length;
static uint8_t test_captured_response[COMMAND_MAX_FRAME_SIZE];
static size_t test_captured_response_length;

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

struct netconn {
    uint32_t unused;
};

struct netbuf {
    const uint8_t *data;
    u16_t length;
    int8_t current;
};

static struct netconn test_listener;
static struct netconn test_client;
static struct netbuf test_netbuf;


void Error_Handler(void) {
    ++test_error_handler_calls;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
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

err_t netconn_recv(struct netconn *connection, struct netbuf **buffer) {
    (void)connection;
    ++test_netconn_recv_calls;
    if (test_receive_data != NULL && test_netconn_recv_calls == 1U) {
        test_netbuf.data = test_receive_data;
        test_netbuf.length = test_receive_length;
        test_netbuf.current = 0;
        *buffer = &test_netbuf;
        return ERR_OK;
    }
    *buffer = NULL;
    return test_recv_result;
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
    test_captured_response_length = size;
    if (size <= sizeof(test_captured_response)) {
        memcpy(test_captured_response, data, size);
    }
    *bytes_written = test_write_full_size != 0U ? size : test_bytes_written;
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

void netbuf_first(struct netbuf *buffer) { buffer->current = 0; }

int8_t netbuf_next(struct netbuf *buffer) {
    buffer->current = -1;
    return -1;
}

err_t netbuf_data(struct netbuf *buffer, void **data, u16_t *length) {
    if (buffer->current < 0) {
        return ERR_BUF;
    }
    *data = (void *)buffer->data;
    *length = buffer->length;
    return ERR_OK;
}

void netbuf_delete(struct netbuf *buffer) {
    (void)buffer;
    ++test_netbuf_delete_calls;
}

#define static
#include "../APP/tasks/network/command/command_task.c"
#undef static

static void reset_fixture(void) {
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_stop_on_delay = 0U;
    test_stop_on_second_new = 0U;
    test_stop_on_second_accept = 0U;
    test_netconn_new_calls = 0U;
    test_netconn_bind_calls = 0U;
    test_netconn_listen_calls = 0U;
    test_netconn_accept_calls = 0U;
    test_netconn_recv_calls = 0U;
    test_netconn_write_calls = 0U;
    test_netbuf_delete_calls = 0U;
    test_netconn_close_calls = 0U;
    test_netconn_delete_calls = 0U;
    test_event_flags_result = 0U;
    test_new_returns_null = 0U;
    test_bind_result = ERR_OK;
    test_listen_result = ERR_OK;
    test_accept_result = ERR_OK;
    test_recv_result = ERR_CONN;
    test_write_result = ERR_OK;
    test_bytes_written = 0U;
    test_write_full_size = 1U;
    test_receive_data = NULL;
    test_receive_length = 0U;
    test_captured_response_length = 0U;
    memset(test_captured_response, 0, sizeof(test_captured_response));
    command_task_diagnostics = (CommandTaskDiagnostics){0};
}

static int run_command_task_until_jump(NetworkTaskContext *context) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        CommandTask(context);
    }
    test_jump_active = 0U;
    return jumped;
}

static void test_wait_and_listener_errors(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    test_event_flags_result = osFlagsErrorParameter;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "LwIP wait error stops command task");
    expect_u32(test_error_handler_calls, 1U, "LwIP wait error calls Error_Handler");
    expect_u32(test_netconn_new_calls, 0U, "no listener after LwIP wait error");

    reset_fixture();
    test_new_returns_null = 1U;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "listener allocation error path");
    expect_u32(test_netconn_new_calls, 1U, "listener allocation attempted");
    expect_u32(command_task_diagnostics.netconn_alloc_errors, 1U, "allocation error counter");

    reset_fixture();
    test_bind_result = ERR_MEM;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "bind error path");
    expect_u32(test_netconn_bind_calls, 1U, "bind attempted");
    expect_u32(test_netconn_listen_calls, 0U, "listen skipped after bind error");
    expect_u32(command_task_diagnostics.bind_errors, 1U, "bind error counter");

    reset_fixture();
    test_listen_result = ERR_MEM;
    test_stop_on_delay = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "listen error path");
    expect_u32(test_netconn_listen_calls, 1U, "listen attempted");
    expect_u32(command_task_diagnostics.listen_errors, 1U, "listen error counter");

    reset_fixture();
    test_accept_result = ERR_MEM;
    test_stop_on_second_new = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "accept error path");
    expect_u32(test_netconn_accept_calls, 1U, "accept attempted");
    expect_u32(command_task_diagnostics.accept_errors, 1U, "accept error counter");
}

static void prepare_receive(const uint8_t *frame, size_t frame_length) {
    test_receive_data = frame;
    test_receive_length = (u16_t)frame_length;
    test_stop_on_second_accept = 1U;
}

static void test_ping_receive_and_response(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 41U, NULL, 0U);

    reset_fixture();
    prepare_receive(frame, frame_length);
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "PING path exits through test control");
    expect_u32(test_netconn_bind_calls, 1U, "command bind");
    expect_u32(test_netconn_listen_calls, 1U, "command listen");
    expect_u32(test_netconn_accept_calls, 2U, "command accepts next client after disconnect");
    expect_u32(test_netconn_recv_calls, 2U, "command receives frame and disconnect");
    expect_u32(test_netconn_write_calls, 1U, "PONG response sent");
    expect_u32(test_netbuf_delete_calls, 1U, "received netbuf deleted");
    expect_u32((uint32_t)test_captured_response_length, 14U, "PONG response size");

    CommandFrameHeaderDTO_t response_header;
    expect_u32(
        command_protocol_validate_frame(test_captured_response, test_captured_response_length, &response_header),
        COMMAND_FRAME_VALID,
        "PONG response frame is valid"
    );
    expect_u32(response_header.type, COMMAND_MESSAGE_TYPE_PONG, "PONG response type");
    expect_u32(response_header.sequence, 41U, "PONG response sequence");
    expect_u32(command_task_diagnostics.connections_accepted, 1U, "accepted connection counter");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "closed connection counter");
    expect_u32(command_task_diagnostics.active_connection, 0U, "active connection reset");
    expect_u32(command_task_diagnostics.bytes_received, (uint32_t)frame_length, "received bytes counter");
    expect_u32(command_task_diagnostics.bytes_sent, 14U, "sent bytes counter");
    expect_u32(command_task_diagnostics.frames_received, 1U, "valid frame counter");
    expect_u32(command_task_diagnostics.pings_received, 1U, "PING counter");
    expect_u32(command_task_diagnostics.pongs_sent, 1U, "PONG counter");
    expect_u32(command_task_diagnostics.response_attempts, 1U, "response attempts counter");
    expect_u32(command_task_diagnostics.response_successes, 1U, "response successes counter");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "response errors counter");
}

static void test_rejected_command(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0xFEU};
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_COMMAND, 42U, payload, sizeof(payload));

    reset_fixture();
    prepare_receive(frame, frame_length);
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "rejected command path");
    expect_u32(test_netconn_write_calls, 1U, "ERROR response sent");
    expect_u32((uint32_t)test_captured_response_length, 16U, "ERROR response size");
    CommandFrameHeaderDTO_t response_header;
    expect_u32(
        command_protocol_validate_frame(test_captured_response, test_captured_response_length, &response_header),
        COMMAND_FRAME_VALID,
        "ERROR response frame is valid"
    );
    expect_u32(response_header.type, COMMAND_MESSAGE_TYPE_ERROR, "ERROR response type");
    expect_u32(test_captured_response[COMMAND_FRAME_HEADER_SIZE], COMMAND_ERROR_UNKNOWN_COMMAND, "ERROR code");
    expect_u32(command_task_diagnostics.commands_received, 1U, "received command counter");
    expect_u32(command_task_diagnostics.commands_rejected, 1U, "rejected command counter");
    expect_u32(command_task_diagnostics.errors_sent, 1U, "sent errors counter");
}

static void test_response_error(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 43U, NULL, 0U);

    reset_fixture();
    prepare_receive(frame, frame_length);
    test_write_result = ERR_BUF;
    test_write_full_size = 0U;
    test_bytes_written = 3U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "response error path");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "response send error counter");
    expect_u32(command_task_diagnostics.partial_writes, 1U, "partial response counter");
    expect_u32(command_task_diagnostics.pongs_sent, 0U, "failed PONG is not counted as sent");
}

int main(void) {
    test_wait_and_listener_errors();
    test_ping_receive_and_response();
    test_rejected_command();
    test_response_error();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command task unit tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Command task unit tests passed.\n");
    return 0;
}
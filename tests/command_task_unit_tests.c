#include "cmsis_os.h"
#include "command_decoder_task.h"
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
static uint8_t test_stop_on_third_accept;
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
static uint8_t test_write_partial_then_ok;
static const uint8_t *test_receive_data;
static u16_t test_receive_length;
static const uint8_t *test_receive_data_second;
static u16_t test_receive_length_second;
static const uint8_t *test_receive_data_third;
static u16_t test_receive_length_third;
static uint8_t test_captured_response[COMMAND_MAX_FRAME_SIZE];
static size_t test_captured_response_length;
static uint8_t test_fail_request_queue;
static uint8_t test_force_response_queue_overflow;
static uint8_t test_force_stale_response_queue_overflow;
static uint32_t test_decoder_failure_connection_id;
static CommandDecoderTaskDiagnostics test_decoder_diagnostics;
static CommandTaskResponseDTO_t test_decoder_responses[16U];
static uint32_t test_decoder_response_read_index;
static uint32_t test_decoder_response_count;

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
    if (test_jump_active != 0U && test_stop_on_third_accept != 0U && test_netconn_accept_calls > 2U) {
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
    if (test_receive_data_second != NULL && test_netconn_recv_calls == 2U) {
        test_netbuf.data = test_receive_data_second;
        test_netbuf.length = test_receive_length_second;
        test_netbuf.current = 0;
        *buffer = &test_netbuf;
        return ERR_OK;
    }
    if (test_receive_data_third != NULL && test_netconn_recv_calls == 3U) {
        test_netbuf.data = test_receive_data_third;
        test_netbuf.length = test_receive_length_third;
        test_netbuf.current = 0;
        *buffer = &test_netbuf;
        return ERR_OK;
    }
    *buffer = NULL;
    return test_recv_result;
}

void netconn_set_recvtimeout(struct netconn *connection, int timeout) {
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
    size_t reported_bytes = test_write_full_size != 0U ? size : test_bytes_written;
    err_t result = test_write_result;
    if (test_write_partial_then_ok != 0U) {
        reported_bytes = test_netconn_write_calls == 1U && size > 3U ? 3U : size;
        result = ERR_OK;
    }
    if (reported_bytes > size) {
        reported_bytes = size;
    }
    if (reported_bytes <= sizeof(test_captured_response) - test_captured_response_length) {
        memcpy(&test_captured_response[test_captured_response_length], data, reported_bytes);
        test_captured_response_length += reported_bytes;
    }
    *bytes_written = reported_bytes;
    return result;
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

uint8_t command_decoder_submit_request(const CommandTaskRequestDTO_t *request) {
    if (request == NULL || test_fail_request_queue != 0U) {
        return 0U;
    }

    if (test_force_response_queue_overflow != 0U || test_force_stale_response_queue_overflow != 0U) {
        test_decoder_diagnostics.response_queue_overflows = 1U;
        test_decoder_failure_connection_id = test_force_stale_response_queue_overflow != 0U
                                                 ? request->connection_id - 1U
                                                 : request->connection_id;
        return 1U;
    }

    const CommandFrameHeaderDTO_t *header = &request->request.header;
    const uint16_t error_code = command_protocol_validate_request_dto(&request->request);
    if (test_decoder_response_count >= sizeof(test_decoder_responses) / sizeof(test_decoder_responses[0])) {
        return 0U;
    }
    CommandTaskResponseDTO_t *response =
        &test_decoder_responses[(test_decoder_response_read_index + test_decoder_response_count) %
                                (sizeof(test_decoder_responses) / sizeof(test_decoder_responses[0]))];
    response->connection_id = request->connection_id;
    response->response_type = COMMAND_MESSAGE_TYPE_ERROR;
    if (error_code == 0U && header->type == COMMAND_MESSAGE_TYPE_PING) {
        response->response_type = COMMAND_MESSAGE_TYPE_PONG;
        response->length = (uint16_t)command_protocol_build_pong_response(
            response->frame,
            sizeof(response->frame),
            header->sequence
        );
    } else {
        response->length = (uint16_t)command_protocol_build_error_response(
            response->frame,
            sizeof(response->frame),
            header->sequence,
            error_code == 0U ? COMMAND_ERROR_INVALID_STATE : error_code
        );
    }
    if (response->length != 0U) {
        ++test_decoder_response_count;
    }
    return 1U;
}

static void expect_true(uint8_t condition, const char *message) {
    if (condition == 0U) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
    }
}

uint8_t command_decoder_receive_response(CommandTaskResponseDTO_t *response, uint32_t timeout_ms) {
    (void)timeout_ms;
    if (response == NULL || test_decoder_response_count == 0U) {
        return 0U;
    }
    *response = test_decoder_responses[test_decoder_response_read_index];
    test_decoder_response_read_index =
        (test_decoder_response_read_index + 1U) % (sizeof(test_decoder_responses) / sizeof(test_decoder_responses[0]));
    --test_decoder_response_count;
    return 1U;
}

void command_decoder_get_diagnostics(CommandDecoderTaskDiagnostics *diagnostics) {
    if (diagnostics != NULL) {
        *diagnostics = test_decoder_diagnostics;
    }
}

void command_decoder_get_response_queue_failure(uint32_t *generation, uint32_t *connection_id) {
    if (generation != NULL) {
        *generation = test_decoder_diagnostics.response_queue_overflows;
    }
    if (connection_id != NULL) {
        *connection_id = test_decoder_failure_connection_id;
    }
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
    test_stop_on_third_accept = 0U;
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
    test_write_partial_then_ok = 0U;
    test_receive_data = NULL;
    test_receive_length = 0U;
    test_receive_data_second = NULL;
    test_receive_length_second = 0U;
    test_receive_data_third = NULL;
    test_receive_length_third = 0U;
    test_captured_response_length = 0U;
    test_fail_request_queue = 0U;
    test_force_response_queue_overflow = 0U;
    test_force_stale_response_queue_overflow = 0U;
    test_decoder_failure_connection_id = 0U;
    test_decoder_diagnostics = (CommandDecoderTaskDiagnostics){0};
    test_decoder_response_read_index = 0U;
    test_decoder_response_count = 0U;
    memset(test_decoder_responses, 0, sizeof(test_decoder_responses));
    memset(test_captured_response, 0, sizeof(test_captured_response));
    command_task_diagnostics = (CommandTaskDiagnostics){0};
    command_task_next_connection_id = 0U;
    command_task_active_connection_id = 0U;
    command_task_seen_response_queue_overflows = 0U;
    command_frame_parser_init(&command_task_frame_parser);
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
    test_receive_data_second = NULL;
    test_receive_length_second = 0U;
    test_stop_on_second_accept = 1U;
}

static void prepare_fragmented_receive(
    const uint8_t *first_fragment,
    size_t first_length,
    const uint8_t *second_fragment,
    size_t second_length
) {
    test_receive_data = first_fragment;
    test_receive_length = (u16_t)first_length;
    test_receive_data_second = second_fragment;
    test_receive_length_second = (u16_t)second_length;
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
    expect_u32(command_task_diagnostics.complete_frames_received, 1U, "complete frame counter");
    expect_u32(command_task_diagnostics.bytes_sent, 14U, "sent bytes counter");
    expect_u32(command_task_diagnostics.response_attempts, 1U, "response attempts counter");
    expect_u32(command_task_diagnostics.response_successes, 1U, "response successes counter");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "response errors counter");
}

static void test_fragmented_ping_receive_and_response(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 46U, NULL, 0U);
    const size_t first_length = 3U;

    reset_fixture();
    prepare_fragmented_receive(frame, first_length, &frame[first_length], frame_length - first_length);
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "fragmented PING path exits through test control");
    expect_u32(test_netconn_recv_calls, 3U, "command receives both fragments and disconnect");
    expect_u32(test_netconn_write_calls, 1U, "fragmented frame produces one PONG response");
    expect_u32((uint32_t)test_captured_response_length, 14U, "fragmented PONG response size");

    CommandFrameHeaderDTO_t response_header;
    expect_u32(
        command_protocol_validate_frame(test_captured_response, test_captured_response_length, &response_header),
        COMMAND_FRAME_VALID,
        "fragmented PONG response frame is valid"
    );
    expect_u32(response_header.type, COMMAND_MESSAGE_TYPE_PONG, "fragmented PONG response type");
    expect_u32(response_header.sequence, 46U, "fragmented PONG response sequence");
    expect_u32(command_task_diagnostics.bytes_received, (uint32_t)frame_length, "all fragmented bytes are counted");
    expect_u32(command_task_diagnostics.complete_frames_received, 1U, "fragmented frame counted once");
    expect_u32(command_task_diagnostics.response_successes, 1U, "fragmented response is sent successfully");
}

static void test_multiple_frames_in_one_receive(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t stream[2U * COMMAND_MAX_FRAME_SIZE];
    CommandFrameHeaderDTO_t response_header;
    const size_t first_length = command_protocol_build_frame(
        stream,
        sizeof(stream),
        COMMAND_MESSAGE_TYPE_PING,
        47U,
        NULL,
        0U
    );
    const size_t second_length = command_protocol_build_frame(
        &stream[first_length],
        sizeof(stream) - first_length,
        COMMAND_MESSAGE_TYPE_PING,
        48U,
        NULL,
        0U
    );

    reset_fixture();
    prepare_receive(stream, first_length + second_length);
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "coalesced frames path");
    expect_u32(command_task_diagnostics.complete_frames_received, 2U, "both coalesced frames are parsed");
    expect_u32(test_netconn_write_calls, 2U, "both coalesced requests receive a response");
    expect_u32((uint32_t)test_captured_response_length, 28U, "both PONG frames are sent");
    expect_u32(
        command_protocol_validate_frame(test_captured_response, 14U, &response_header),
        COMMAND_FRAME_VALID,
        "first coalesced PONG is valid"
    );
    expect_u32(response_header.sequence, 47U, "first PONG keeps sequence");
    expect_u32(
        command_protocol_validate_frame(&test_captured_response[14U], 14U, &response_header),
        COMMAND_FRAME_VALID,
        "second coalesced PONG is valid"
    );
    expect_u32(response_header.sequence, 48U, "second PONG keeps sequence");
}

static void test_transport_errors_are_counted_and_resynchronized(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t stream[4U * COMMAND_MAX_FRAME_SIZE];
    uint8_t bad_crc_frame[COMMAND_MAX_FRAME_SIZE];
    uint8_t valid_frame[COMMAND_MAX_FRAME_SIZE];
    const size_t short_frame_size = COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE;
    size_t stream_length = 0U;
    const size_t bad_crc_length = command_protocol_build_frame(
        bad_crc_frame,
        sizeof(bad_crc_frame),
        COMMAND_MESSAGE_TYPE_PING,
        49U,
        NULL,
        0U
    );
    bad_crc_frame[bad_crc_length - 1U] ^= 0x01U;
    const size_t valid_length = command_protocol_build_frame(
        valid_frame,
        sizeof(valid_frame),
        COMMAND_MESSAGE_TYPE_PING,
        50U,
        NULL,
        0U
    );

    memset(stream, 0, short_frame_size);
    stream_length += short_frame_size;
    stream[stream_length] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    stream[stream_length + 1U] = (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U);
    stream[stream_length + 4U] = 0x01U;
    stream[stream_length + 5U] = 0x01U;
    stream_length += short_frame_size;
    memcpy(&stream[stream_length], bad_crc_frame, bad_crc_length);
    stream_length += bad_crc_length;
    memcpy(&stream[stream_length], valid_frame, valid_length);
    stream_length += valid_length;

    reset_fixture();
    prepare_receive(stream, stream_length);
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "transport resynchronization path");
    expect_true(command_task_diagnostics.invalid_magic > 0U, "invalid magic is counted");
    expect_true(command_task_diagnostics.invalid_frame_length > 0U, "invalid frame length is counted");
    expect_true(command_task_diagnostics.invalid_crc > 0U, "invalid CRC is counted");
    expect_u32(command_task_diagnostics.complete_frames_received, 1U, "valid frame is recovered after errors");
    expect_u32(command_task_diagnostics.buffer_current_bytes, 0U, "parser buffer is cleared after disconnect");
    expect_true(
        command_task_diagnostics.buffer_max_bytes >= short_frame_size,
        "parser high-water buffer occupancy is recorded"
    );
    expect_u32(test_netconn_write_calls, 1U, "only recovered valid frame produces a response");
}

static void test_request_queue_overflow_closes_connection(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 51U, NULL, 0U);

    reset_fixture();
    prepare_receive(frame, frame_length);
    test_fail_request_queue = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "request queue overflow path");
    expect_u32(command_task_diagnostics.decoder_queue_errors, 1U, "request queue overflow is counted by transport");
    expect_u32(test_netconn_close_calls, 1U, "connection closes after request cannot be queued");
    expect_u32(test_netconn_write_calls, 0U, "no response is fabricated after queue failure");
    expect_u32(command_task_diagnostics.buffer_current_bytes, 0U, "parser state is reset after queue failure");
}

static void test_response_queue_overflow_closes_connection(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 52U, NULL, 0U);

    reset_fixture();
    prepare_receive(frame, frame_length);
    test_force_response_queue_overflow = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "response queue overflow path");
    expect_u32(test_netconn_close_calls, 1U, "connection closes after response queue overflow");
    expect_u32(test_netconn_write_calls, 0U, "missing response is not written");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_MEM, "queue overflow error is recorded");
}

static void test_stale_response_queue_overflow_does_not_match_new_connection(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 56U, NULL, 0U);

    reset_fixture();
    prepare_receive(frame, frame_length);
    test_force_stale_response_queue_overflow = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "stale response overflow path");
    expect_u32(test_netconn_recv_calls, 2U, "new connection remains active until peer closes it");
    expect_u32(
        (uint32_t)command_task_diagnostics.last_error,
        (uint32_t)ERR_CONN,
        "older connection overflow is not attributed to the active connection"
    );
}

static void test_partial_successful_response_write(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 53U, NULL, 0U);

    reset_fixture();
    prepare_receive(frame, frame_length);
    test_write_partial_then_ok = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "partial successful response path");
    expect_u32(test_netconn_write_calls, 2U, "remaining response bytes are written");
    expect_u32((uint32_t)test_captured_response_length, 14U, "partial calls produce one complete frame");
    expect_u32(command_task_diagnostics.bytes_sent, 14U, "all response bytes are counted");
    expect_u32(command_task_diagnostics.partial_writes, 1U, "partial response is counted once");
    expect_u32(command_task_diagnostics.response_successes, 1U, "completed partial response is successful");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "partial progress is not treated as an error");
}

static void test_partial_frame_is_discarded_on_reconnect(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t incomplete_frame[COMMAND_MAX_FRAME_SIZE];
    uint8_t next_connection_frame[COMMAND_MAX_FRAME_SIZE];
    const size_t incomplete_length = command_protocol_build_frame(
        incomplete_frame,
        sizeof(incomplete_frame),
        COMMAND_MESSAGE_TYPE_PING,
        54U,
        NULL,
        0U
    );
    const size_t next_length = command_protocol_build_frame(
        next_connection_frame,
        sizeof(next_connection_frame),
        COMMAND_MESSAGE_TYPE_PING,
        55U,
        NULL,
        0U
    );

    expect_true(incomplete_length > 3U, "test frame is longer than its first fragment");
    reset_fixture();
    test_receive_data = incomplete_frame;
    test_receive_length = 3U;
    test_receive_data_third = next_connection_frame;
    test_receive_length_third = (u16_t)next_length;
    test_stop_on_third_accept = 1U;
    expect_u32((uint32_t)run_command_task_until_jump(&context), 1U, "reconnect after incomplete frame path");
    expect_u32(command_task_diagnostics.connections_accepted, 2U, "both client connections are accepted");
    expect_u32(command_task_diagnostics.connections_closed, 2U, "both client connections are closed");
    expect_u32(command_task_diagnostics.complete_frames_received, 1U, "old partial frame is not completed on reconnect");
    expect_u32(test_netconn_write_calls, 1U, "only the new connection receives a response");
    expect_u32((uint32_t)test_captured_response_length, 14U, "new connection response is complete");
    CommandFrameHeaderDTO_t response_header;
    expect_u32(
        command_protocol_validate_frame(test_captured_response, test_captured_response_length, &response_header),
        COMMAND_FRAME_VALID,
        "reconnected response is a valid frame"
    );
    expect_u32(response_header.sequence, 55U, "new connection response has its own request sequence");
    expect_u32(command_task_diagnostics.buffer_current_bytes, 0U, "parser state is empty after reconnect");
    expect_true(command_task_diagnostics.buffer_max_bytes >= 14U, "buffer high-water mark survives reconnect");
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
    expect_u32(test_netconn_close_calls, 1U, "connection closes after incomplete response write");
}

int main(void) {
    test_wait_and_listener_errors();
    test_ping_receive_and_response();
    test_fragmented_ping_receive_and_response();
    test_multiple_frames_in_one_receive();
    test_transport_errors_are_counted_and_resynchronized();
    test_request_queue_overflow_closes_connection();
    test_response_queue_overflow_closes_connection();
    test_stale_response_queue_overflow_does_not_match_new_connection();
    test_partial_successful_response_write();
    test_partial_frame_is_discarded_on_reconnect();
    test_rejected_command();
    test_response_error();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command task unit tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Command task unit tests passed.\n");
    return 0;
}

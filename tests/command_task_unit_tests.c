#include "command_decoder.h"
#include "command_protocol.h"
#include "command/command_queue.h"
#include "command_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netbuf.h"
#include "main.h"
#include "network_events.h"
#include "task_context.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;
static uint32_t test_error_handler_calls;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint8_t test_jump_on_delay;
static uint32_t test_jump_after_new_calls;
static uint32_t test_jump_after_accept_calls;
static uint8_t test_new_returns_null;
static uint8_t test_accept_returns_null;
static uint32_t test_event_flags_result;
static err_t test_bind_result;
static err_t test_listen_result;
static err_t test_accept_result;
static err_t test_default_recv_result;
static err_t test_write_result;
static uint32_t test_tick_count;
static uint32_t test_tick_after_receive[8];
static uint8_t test_tick_after_receive_enabled[8];
static uint32_t test_netconn_new_calls;
static uint32_t test_netconn_bind_calls;
static uint32_t test_netconn_listen_calls;
static uint32_t test_netconn_accept_calls;
static uint32_t test_netconn_recv_calls;
static uint32_t test_netconn_write_calls;
static uint32_t test_netconn_close_calls;
static uint32_t test_netconn_delete_calls;
static uint32_t test_netbuf_delete_calls;
static uint32_t test_recv_timeout;
static uint8_t test_write_mode;
static size_t test_captured_response_length;
static uint8_t test_captured_response[COMMAND_MAX_FRAME_SIZE * 4U];
static uint8_t test_force_assembler_next;
static uint8_t test_force_assembler_append_failure;
static uint8_t test_force_assembler_consume_failure;
static uint8_t test_force_decoder_invalid_argument;
static CommandQueuePutResult_t test_command_queue_put_result;
static uint32_t test_command_queue_put_calls;
static CommandDTO_t test_queued_command;

enum {
    TEST_WRITE_FULL = 0U,
    TEST_WRITE_PARTIAL_THEN_FULL,
    TEST_WRITE_FAIL,
    TEST_WRITE_FAIL_AFTER_PARTIAL,
    TEST_WRITE_ERROR_WITH_PARTIAL_FIRST,
    TEST_WRITE_ZERO_PROGRESS,
    TEST_WRITE_OVERREPORT,
    TEST_WRITE_PARTIAL_THEN_OVERREPORT
};

enum {
    TEST_ASSEMBLER_NEXT_REAL = 0U,
    TEST_ASSEMBLER_NEXT_NEED_MORE,
    TEST_ASSEMBLER_NEXT_INVALID_ARGUMENT,
    TEST_ASSEMBLER_NEXT_COMMAND_READY,
    TEST_ASSEMBLER_NEXT_INVALID_FRAME_VALIDATION
};

struct netconn {
    uint32_t unused;
};

struct netbuf {
    const uint8_t *parts[3];
    u16_t lengths[3];
    uint8_t part_count;
    int8_t current;
    err_t data_result;
};

typedef struct {
    err_t result;
    struct netbuf *buffer;
} TestReceive_t;

static struct netconn test_listener;
static struct netconn test_client;
static TestReceive_t test_receives[8];
static uint32_t test_receive_script_count;
static struct netbuf test_buffers[8];

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

static void expect_true(uint8_t condition, const char *message) {
    if (condition == 0U) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
    }
}

TickType_t xTaskGetTickCount(void) { return (TickType_t)test_tick_count; }

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
    if (test_jump_active != 0U && test_jump_on_delay != 0U) {
        longjmp(test_jump_buffer, 1);
    }
    return 0U;
}

struct netconn *netconn_new(int type) {
    (void)type;
    ++test_netconn_new_calls;
    if (test_jump_active != 0U && test_jump_after_new_calls != 0U &&
        test_netconn_new_calls > test_jump_after_new_calls) {
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
    if (test_jump_active != 0U && test_jump_after_accept_calls != 0U &&
        test_netconn_accept_calls > test_jump_after_accept_calls) {
        longjmp(test_jump_buffer, 1);
    }
    if (test_accept_result == ERR_OK) {
        *new_connection = test_accept_returns_null != 0U ? NULL : &test_client;
    }
    return test_accept_result;
}

err_t netconn_recv(struct netconn *connection, struct netbuf **buffer) {
    (void)connection;
    ++test_netconn_recv_calls;
    const uint32_t index = test_netconn_recv_calls - 1U;
    if (index < test_receive_script_count) {
        *buffer = test_receives[index].buffer;
        if (test_tick_after_receive_enabled[index] != 0U) {
            test_tick_count = test_tick_after_receive[index];
        }
        return test_receives[index].result;
    }
    *buffer = NULL;
    return test_default_recv_result;
}

void netconn_set_recvtimeout(struct netconn *connection, int timeout) {
    (void)connection;
    test_recv_timeout = (uint32_t)timeout;
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

    size_t reported = size;
    err_t result = test_write_result;
    switch (test_write_mode) {
    case TEST_WRITE_PARTIAL_THEN_FULL:
        reported = test_netconn_write_calls == 1U && size > 3U ? 3U : size;
        result = ERR_OK;
        break;
    case TEST_WRITE_FAIL:
        reported = 0U;
        result = ERR_BUF;
        break;
    case TEST_WRITE_FAIL_AFTER_PARTIAL:
        reported = test_netconn_write_calls == 1U && size > 3U ? 3U : 0U;
        result = test_netconn_write_calls == 1U ? ERR_OK : ERR_BUF;
        break;
    case TEST_WRITE_ERROR_WITH_PARTIAL_FIRST:
        reported = 3U;
        result = ERR_BUF;
        break;
    case TEST_WRITE_ZERO_PROGRESS:
        reported = 0U;
        break;
    case TEST_WRITE_OVERREPORT:
        reported = size + 1U;
        break;
    case TEST_WRITE_PARTIAL_THEN_OVERREPORT:
        reported = test_netconn_write_calls == 1U && size > 3U ? 3U : size + 1U;
        result = ERR_OK;
        break;
    default:
        break;
    }

    const size_t copy_length = reported > size ? size : reported;
    if (copy_length <= sizeof(test_captured_response) - test_captured_response_length) {
        memcpy(&test_captured_response[test_captured_response_length], data, copy_length);
        test_captured_response_length += copy_length;
    }
    *bytes_written = reported;
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

void netbuf_first(struct netbuf *buffer) { buffer->current = 0; }

int8_t netbuf_next(struct netbuf *buffer) {
    if ((uint8_t)(buffer->current + 1) < buffer->part_count) {
        ++buffer->current;
        return 0;
    }
    buffer->current = -1;
    return -1;
}

u16_t netbuf_len(struct netbuf *buffer) {
    u16_t total = 0U;
    for (uint8_t index = 0U; index < buffer->part_count; ++index) {
        total = (u16_t)(total + buffer->lengths[index]);
    }
    return total;
}

err_t netbuf_data(struct netbuf *buffer, void **data, u16_t *length) {
    if (buffer->data_result != ERR_OK) {
        return buffer->data_result;
    }
    if (buffer->current < 0 || (uint8_t)buffer->current >= buffer->part_count) {
        return ERR_BUF;
    }
    *data = (void *)buffer->parts[(uint8_t)buffer->current];
    *length = buffer->lengths[(uint8_t)buffer->current];
    return ERR_OK;
}

void netbuf_delete(struct netbuf *buffer) {
    (void)buffer;
    ++test_netbuf_delete_calls;
}

static CommandFrameAssemblerEvent_t test_command_frame_assembler_next(const CommandFrameAssembler_t *assembler);
static size_t test_command_frame_assembler_append(
    CommandFrameAssembler_t *assembler,
    const uint8_t *data,
    size_t data_length
);
static uint8_t test_command_frame_assembler_consume(CommandFrameAssembler_t *assembler, size_t length);
static CommandDecoderResult_t test_command_decoder_decode_request(const CommandRequestDTO_t *request);

#define command_frame_assembler_next test_command_frame_assembler_next
#define command_frame_assembler_append test_command_frame_assembler_append
#define command_frame_assembler_consume test_command_frame_assembler_consume
#define command_decoder_decode_request test_command_decoder_decode_request
#define static
#include "../APP/tasks/network/command/command_task.c"
#undef static
#undef command_decoder_decode_request
#undef command_frame_assembler_consume
#undef command_frame_assembler_append
#undef command_frame_assembler_next

CommandQueuePutResult_t command_queue_try_send(const CommandDTO_t *command) {
    ++test_command_queue_put_calls;
    if (command == NULL) {
        return COMMAND_QUEUE_PUT_ERROR;
    }
    test_queued_command = *command;
    return test_command_queue_put_result;
}

static CommandFrameAssemblerEvent_t test_command_frame_assembler_next(const CommandFrameAssembler_t *assembler) {
    if (test_force_assembler_next == TEST_ASSEMBLER_NEXT_NEED_MORE) {
        test_force_assembler_next = TEST_ASSEMBLER_NEXT_REAL;
        return (CommandFrameAssemblerEvent_t){.result = COMMAND_FRAME_ASSEMBLER_NEED_MORE};
    }
    if (test_force_assembler_next == TEST_ASSEMBLER_NEXT_INVALID_ARGUMENT) {
        test_force_assembler_next = TEST_ASSEMBLER_NEXT_REAL;
        return (CommandFrameAssemblerEvent_t){.result = COMMAND_FRAME_ASSEMBLER_INVALID_ARGUMENT};
    }
    if (test_force_assembler_next == TEST_ASSEMBLER_NEXT_COMMAND_READY) {
        test_force_assembler_next = TEST_ASSEMBLER_NEXT_REAL;
        return (CommandFrameAssemblerEvent_t){
            .result = COMMAND_FRAME_ASSEMBLER_COMMAND_READY,
            .consume_length = 1U,
            .command = {
                .header = {
                    .version = COMMAND_PROTOCOL_VERSION,
                    .type = COMMAND_MESSAGE_TYPE_COMMAND,
                    .payload_length = 1U,
                    .sequence = 1U,
                },
                .payload = {COMMAND_CODE_STOP},
            },
        };
    }
    if (test_force_assembler_next == TEST_ASSEMBLER_NEXT_INVALID_FRAME_VALIDATION) {
        test_force_assembler_next = TEST_ASSEMBLER_NEXT_REAL;
        return (CommandFrameAssemblerEvent_t){
            .result = COMMAND_FRAME_ASSEMBLER_INVALID_FRAME,
            .validation = COMMAND_FRAME_VALID,
        };
    }
    return command_frame_assembler_next(assembler);
}

static size_t test_command_frame_assembler_append(
    CommandFrameAssembler_t *assembler,
    const uint8_t *data,
    size_t data_length
) {
    if (test_force_assembler_append_failure != 0U) {
        test_force_assembler_append_failure = 0U;
        return 0U;
    }
    return command_frame_assembler_append(assembler, data, data_length);
}

static uint8_t test_command_frame_assembler_consume(CommandFrameAssembler_t *assembler, size_t length) {
    if (test_force_assembler_consume_failure != 0U) {
        test_force_assembler_consume_failure = 0U;
        return 0U;
    }
    return command_frame_assembler_consume(assembler, length);
}

static CommandDecoderResult_t test_command_decoder_decode_request(const CommandRequestDTO_t *request) {
    if (test_force_decoder_invalid_argument != 0U) {
        test_force_decoder_invalid_argument = 0U;
        return (CommandDecoderResult_t){.type = COMMAND_DECODER_RESULT_INVALID_ARGUMENT};
    }
    return command_decoder_decode_request(request);
}

static void reset_fixture(void) {
    test_error_handler_calls = 0U;
    test_jump_active = 0U;
    test_jump_on_delay = 0U;
    test_jump_after_new_calls = 0U;
    test_jump_after_accept_calls = 0U;
    test_new_returns_null = 0U;
    test_accept_returns_null = 0U;
    test_event_flags_result = 0U;
    test_bind_result = ERR_OK;
    test_listen_result = ERR_OK;
    test_accept_result = ERR_OK;
    test_default_recv_result = ERR_CONN;
    test_write_result = ERR_OK;
    test_tick_count = 0U;
    memset(test_tick_after_receive, 0, sizeof(test_tick_after_receive));
    memset(test_tick_after_receive_enabled, 0, sizeof(test_tick_after_receive_enabled));
    test_netconn_new_calls = 0U;
    test_netconn_bind_calls = 0U;
    test_netconn_listen_calls = 0U;
    test_netconn_accept_calls = 0U;
    test_netconn_recv_calls = 0U;
    test_netconn_write_calls = 0U;
    test_netconn_close_calls = 0U;
    test_netconn_delete_calls = 0U;
    test_netbuf_delete_calls = 0U;
    test_recv_timeout = 0U;
    test_write_mode = TEST_WRITE_FULL;
    test_captured_response_length = 0U;
    test_receive_script_count = 0U;
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_REAL;
    test_force_assembler_append_failure = 0U;
    test_force_assembler_consume_failure = 0U;
    test_force_decoder_invalid_argument = 0U;
    test_command_queue_put_result = COMMAND_QUEUE_PUT_OK;
    test_command_queue_put_calls = 0U;
    test_queued_command = (CommandDTO_t){0};
    memset(test_receives, 0, sizeof(test_receives));
    memset(test_buffers, 0, sizeof(test_buffers));
    memset(test_captured_response, 0, sizeof(test_captured_response));
    command_task_diagnostics = (CommandTaskDiagnostics){0};
    command_frame_assembler_init(&command_task_assembler);
}

static void set_buffer(
    uint32_t receive_index,
    const uint8_t *first,
    u16_t first_length,
    const uint8_t *second,
    u16_t second_length,
    const uint8_t *third,
    u16_t third_length
) {
    struct netbuf *buffer = &test_buffers[receive_index];
    buffer->parts[0] = first;
    buffer->lengths[0] = first_length;
    buffer->part_count = 1U;
    if (second != NULL || second_length != 0U) {
        buffer->parts[1] = second;
        buffer->lengths[1] = second_length;
        buffer->part_count = 2U;
    }
    if (third != NULL || third_length != 0U) {
        buffer->parts[2] = third;
        buffer->lengths[2] = third_length;
        buffer->part_count = 3U;
    }
    buffer->current = 0;
    buffer->data_result = ERR_OK;
    test_receives[receive_index].buffer = buffer;
    if (test_receive_script_count <= receive_index) {
        test_receive_script_count = receive_index + 1U;
    }
}

static void set_receive_result(uint32_t receive_index, err_t result) {
    test_receives[receive_index].result = result;
    if (test_receive_script_count <= receive_index) {
        test_receive_script_count = receive_index + 1U;
    }
}

static void set_tick_after_receive(uint32_t receive_index, uint32_t tick_count) {
    test_tick_after_receive[receive_index] = tick_count;
    test_tick_after_receive_enabled[receive_index] = 1U;
}

static int run_task_until_jump(NetworkTaskContext *context) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        CommandTask(context);
    }
    test_jump_active = 0U;
    return jumped;
}

static size_t build_ping(uint8_t *frame, uint32_t sequence) {
    return command_protocol_build_frame(frame, COMMAND_MAX_FRAME_SIZE, COMMAND_MESSAGE_TYPE_PING, sequence, NULL, 0U);
}

static size_t build_stop_command(uint8_t *frame, uint32_t sequence) {
    const uint8_t payload[] = {COMMAND_CODE_STOP};
    return command_protocol_build_frame(
        frame,
        COMMAND_MAX_FRAME_SIZE,
        COMMAND_MESSAGE_TYPE_COMMAND,
        sequence,
        payload,
        sizeof(payload)
    );
}

static void expect_single_response(uint8_t type, uint32_t sequence, uint16_t error_code) {
    const size_t expected_length = type == COMMAND_MESSAGE_TYPE_ERROR ? 16U : 14U;
    expect_u32((uint32_t)test_captured_response_length, (uint32_t)expected_length, "exactly one response is sent");
    CommandFrameHeaderDTO_t header = {0};
    expect_u32(
        command_protocol_validate_frame(test_captured_response, test_captured_response_length, &header),
        COMMAND_FRAME_VALID,
        "response is a valid protocol frame"
    );
    expect_u32(header.type, type, "response type matches the current protocol");
    expect_u32(header.sequence, sequence, "response preserves request sequence");
    if (type == COMMAND_MESSAGE_TYPE_ERROR) {
        const uint16_t actual_error = (uint16_t)test_captured_response[COMMAND_FRAME_HEADER_SIZE] |
            ((uint16_t)test_captured_response[COMMAND_FRAME_HEADER_SIZE + 1U] << 8U);
        expect_u32(actual_error, error_code, "ERROR response contains the expected code");
    }
}

static void test_context_and_listener_failures(void) {
    NetworkTaskContext valid_context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    CommandTask(NULL);
    expect_u32(test_error_handler_calls, 1U, "null task context reports a fatal error");

    reset_fixture();
    NetworkTaskContext null_flags = {.lwip_flags = NULL};
    CommandTask(&null_flags);
    expect_u32(test_error_handler_calls, 1U, "null event flags report a fatal error");

    reset_fixture();
    test_event_flags_result = osFlagsErrorParameter;
    CommandTask(&valid_context);
    expect_u32(test_error_handler_calls, 1U, "LwIP readiness error reports a fatal error");
    expect_u32(test_netconn_new_calls, 0U, "listener is not created before LwIP is ready");

    reset_fixture();
    test_new_returns_null = 1U;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "listener allocation retry is interruptible in test");
    expect_u32(command_task_diagnostics.netconn_alloc_errors, 1U, "listener allocation failure is counted");

    reset_fixture();
    test_bind_result = ERR_MEM;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "bind retry is interruptible in test");
    expect_u32(test_netconn_bind_calls, 1U, "listener bind is attempted");
    expect_u32(test_netconn_listen_calls, 0U, "listen is skipped after bind failure");
    expect_u32(test_netconn_delete_calls, 1U, "unbound listener is deleted");
    expect_u32(command_task_diagnostics.bind_errors, 1U, "bind failure is counted");

    reset_fixture();
    test_listen_result = ERR_MEM;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "listen retry is interruptible in test");
    expect_u32(test_netconn_close_calls, 1U, "listener is closed after listen failure");
    expect_u32(test_netconn_delete_calls, 1U, "listener is deleted after listen failure");
    expect_u32(command_task_diagnostics.listen_errors, 1U, "listen failure is counted");

    reset_fixture();
    test_accept_result = ERR_MEM;
    test_jump_after_new_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "accept error exits the current listener");
    expect_u32(command_task_diagnostics.accept_errors, 1U, "accept error is counted");
    expect_u32(test_netconn_close_calls, 1U, "listener is closed after accept error");

    reset_fixture();
    test_accept_returns_null = 1U;
    test_jump_after_new_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "null accepted client exits current listener");
    expect_u32(command_task_diagnostics.accept_errors, 1U, "null client is counted as accept error");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_CONN, "null client records connection error");
}

static void run_one_receive(const uint8_t *frame, size_t frame_length) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    set_buffer(0U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "task exits through controlled accept on next client");
}

static void test_ping_ack_and_error_responses(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];

    reset_fixture();
    size_t frame_length = build_ping(frame, 41U);
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_PONG, 41U, 0U);
    expect_u32(test_netconn_write_calls, 1U, "PING receives only one PONG");
    expect_u32(test_command_queue_put_calls, 0U, "PING is not placed in the command queue");
    expect_u32(command_task_diagnostics.complete_frames_received, 1U, "complete PING is counted");

    reset_fixture();
    frame_length = build_stop_command(frame, 42U);
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ACK, 42U, 0U);
    expect_u32(test_netconn_write_calls, 1U, "accepted command receives only one ACK");
    expect_u32(test_command_queue_put_calls, 1U, "decoded command is submitted to the queue once");
    expect_u32(test_queued_command.sequence, 42U, "queued command preserves sequence");
    expect_u32(test_queued_command.code, COMMAND_CODE_STOP, "queued command preserves command code");

    reset_fixture();
    test_command_queue_put_result = COMMAND_QUEUE_PUT_FULL;
    frame_length = build_stop_command(frame, 46U);
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 46U, COMMAND_ERROR_QUEUE_FULL);
    expect_u32(test_netconn_write_calls, 1U, "full queue returns one ERROR response");

    reset_fixture();
    test_command_queue_put_result = COMMAND_QUEUE_PUT_ERROR;
    frame_length = build_stop_command(frame, 47U);
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 47U, COMMAND_ERROR_INVALID_STATE);

    reset_fixture();
    const uint8_t unknown_payload[] = {0xFEU};
    frame_length = command_protocol_build_frame(
        frame,
        sizeof(frame),
        COMMAND_MESSAGE_TYPE_COMMAND,
        43U,
        unknown_payload,
        sizeof(unknown_payload)
    );
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 43U, COMMAND_ERROR_UNKNOWN_COMMAND);

    reset_fixture();
    frame_length = build_ping(frame, 44U);
    frame[frame_length - 1U] ^= 1U;
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 44U, COMMAND_ERROR_INVALID_FRAME_CRC);

    reset_fixture();
    frame_length = build_ping(frame, 45U);
    frame[0] ^= 1U;
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 45U, COMMAND_ERROR_INVALID_FRAME_FORMAT);
}

static void test_fragmented_receive_and_disconnect_cleanup(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 51U);

    reset_fixture();
    set_buffer(0U, frame, 3U, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_buffer(1U, &frame[3], (u16_t)(frame_length - 3U), NULL, 0U, NULL, 0U);
    set_receive_result(1U, ERR_OK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "fragmented frame is handled before disconnect");
    expect_single_response(COMMAND_MESSAGE_TYPE_PONG, 51U, 0U);
    expect_u32(test_netbuf_delete_calls, 2U, "each received buffer is released");
    expect_u32(test_recv_timeout, COMMAND_TASK_RECEIVE_TIMEOUT_MS, "client receive timeout is configured");
    expect_u32(command_task_diagnostics.connections_accepted, 1U, "client acceptance is counted");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "client close is counted");
    expect_u32(command_task_diagnostics.active_connection, 0U, "active connection clears on disconnect");

    reset_fixture();
    set_buffer(0U, frame, 3U, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_CONN);
    set_buffer(2U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(2U, ERR_OK);
    test_jump_after_accept_calls = 2U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "new connection starts with an empty assembler");
    expect_single_response(COMMAND_MESSAGE_TYPE_PONG, 51U, 0U);
    expect_u32(command_task_diagnostics.invalid_frame_length, 1U, "partial frame from prior connection is discarded");
}

static void test_receive_timeout_and_receive_errors(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    (void)build_ping(frame, 61U);

    reset_fixture();
    set_buffer(0U, frame, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_TIMEOUT);
    set_tick_after_receive(1U, COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS - 1U);
    set_receive_result(2U, ERR_CONN);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "nonexpired frame survives receive timeout");
    expect_u32(test_netconn_write_calls, 0U, "no response is sent for a truncated request");
    expect_u32(command_task_diagnostics.invalid_frame_length, 1U, "disconnect records a truncated frame");

    reset_fixture();
    set_buffer(0U, frame, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_TIMEOUT);
    set_tick_after_receive(1U, COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed timeout ERROR closes the client");
    expect_u32(test_netconn_write_calls, 1U, "truncation ERROR is attempted once");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failed timeout response is counted");

    reset_fixture();
    set_receive_result(0U, ERR_MEM);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "receive error closes the client");
    expect_u32(command_task_diagnostics.recv_errors, 1U, "receive error is counted");

    reset_fixture();
    set_receive_result(0U, ERR_OK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "successful receive with no buffer is rejected");
    expect_u32(command_task_diagnostics.recv_errors, 1U, "missing receive buffer is counted");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "missing buffer records ERR_BUF");

    reset_fixture();
    set_buffer(0U, frame, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    test_buffers[0].data_result = ERR_MEM;
    set_receive_result(0U, ERR_OK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "netbuf assembly error closes current connection");
    expect_u32(command_task_diagnostics.recv_errors, 1U, "netbuf assembly error is counted");
    expect_u32(test_netbuf_delete_calls, 1U, "buffer is deleted after assembly error");
}

static void test_assembler_errors_and_boundary_paths(void) {
    reset_fixture();
    struct netbuf buffer = {.parts = {(const uint8_t[]){1U}}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_MEM};
    CommandTaskAssemblyEvent_t event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "netbuf extraction error becomes assembly error");
    expect_u32(command_task_diagnostics.recv_errors, 1U, "netbuf extraction error is counted");

    reset_fixture();
    buffer = (struct netbuf){.parts = {NULL}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "null data with nonzero length is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "null data records ERR_BUF");

    reset_fixture();
    buffer = (struct netbuf){.parts = {NULL}, .lengths = {0U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_NEED_MORE, "empty netbuf part is ignored");

    reset_fixture();
    buffer = (struct netbuf){.parts = {(const uint8_t[]){1U}}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    command_task_assembler.length = COMMAND_MAX_FRAME_SIZE + 1U;
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "invalid assembler state is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "invalid assembler state records ERR_ARG");

    reset_fixture();
    buffer = (struct netbuf){.parts = {(const uint8_t[]){1U}}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_NEED_MORE;
    test_force_assembler_append_failure = 1U;
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "zero-byte append is handled as assembly failure");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "zero-byte append records ERR_BUF");

    reset_fixture();
    buffer = (struct netbuf){.parts = {(const uint8_t[]){1U}}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_INVALID_ARGUMENT;
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "assembler invalid-argument event is propagated");

    reset_fixture();
    buffer = (struct netbuf){.parts = {(const uint8_t[]){1U}}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_INVALID_FRAME_VALIDATION;
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "unmapped frame validation result becomes error");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "unmapped frame result records ERR_ARG");

    reset_fixture();
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    size_t frame_length = build_stop_command(frame, 71U);
    buffer = (struct netbuf){
        .parts = {frame},
        .lengths = {(u16_t)frame_length},
        .part_count = 1U,
        .current = 0,
        .data_result = ERR_OK,
    };
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_COMMAND_READY;
    test_force_assembler_consume_failure = 1U;
    event = command_task_assemble_netbuf(&buffer, &(CommandTaskAssemblerContext_t){0});
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "assembler consume failure is handled");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "consume failure records ERR_BUF");

    reset_fixture();
    command_task_assembler.length = 1U;
    buffer = (struct netbuf){.parts = {(const uint8_t[]){1U}}, .lengths = {1U}, .part_count = 1U, .current = 0, .data_result = ERR_OK};
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_COMMAND_READY;
    CommandTaskAssemblerContext_t disconnect_context = {.disconnect_requested = 1U};
    event = command_task_assemble_netbuf(&buffer, &disconnect_context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "pending disconnect stops assembly after consume");

    reset_fixture();
    frame_length = build_stop_command(frame, 72U);
    const uint8_t trailing[] = {0xA5U};
    buffer = (struct netbuf){
        .parts = {frame, trailing},
        .lengths = {(u16_t)frame_length, sizeof(trailing)},
        .part_count = 2U,
        .current = 0,
        .data_result = ERR_OK,
    };
    event = command_task_assemble_netbuf(&buffer, &(CommandTaskAssemblerContext_t){0});
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "trailing bytes after a complete frame are rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_VAL, "trailing bytes record ERR_VAL");

    reset_fixture();
    frame_length = build_stop_command(frame, 73U);
    const uint8_t next_frame[COMMAND_MAX_FRAME_SIZE] = {0};
    buffer = (struct netbuf){
        .parts = {frame, next_frame},
        .lengths = {(u16_t)frame_length, 1U},
        .part_count = 2U,
        .current = 0,
        .data_result = ERR_OK,
    };
    event = command_task_assemble_netbuf(&buffer, &(CommandTaskAssemblerContext_t){0});
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "unread later netbuf segments are rejected");
}

static void test_frame_errors_timeout_helpers_and_response_writes(void) {
    reset_fixture();
    uint8_t ping[COMMAND_MAX_FRAME_SIZE];
    const size_t ping_length = build_ping(ping, 81U);
    uint32_t sequence = UINT32_MAX;
    expect_u32(command_task_record_frame_error(COMMAND_FRAME_VALID, NULL, 0U, &sequence), 0U, "valid frame has no error code");
    expect_u32(sequence, 0U, "sequence output resets before frame validation");
    expect_u32(
        command_task_record_frame_error(COMMAND_FRAME_INVALID_MAGIC, ping, ping_length, &sequence),
        COMMAND_ERROR_INVALID_FRAME_FORMAT,
        "invalid magic maps to ERROR"
    );
    expect_u32(sequence, 81U, "invalid magic error extracts sequence");
    expect_u32(
        command_task_record_frame_error(COMMAND_FRAME_INVALID_LENGTH, ping, ping_length, NULL),
        COMMAND_ERROR_INVALID_FRAME_FORMAT,
        "invalid length maps to ERROR"
    );
    expect_u32(
        command_task_record_frame_error(COMMAND_FRAME_INVALID_CRC, ping, ping_length, &sequence),
        COMMAND_ERROR_INVALID_FRAME_CRC,
        "invalid CRC maps to ERROR"
    );
    expect_u32(
        command_task_record_frame_error(COMMAND_FRAME_INVALID_ARGUMENT, NULL, 0U, NULL),
        0U,
        "unclassified parser error has no protocol error code"
    );
    expect_u32(
        command_task_record_frame_error(COMMAND_FRAME_INVALID_CRC, NULL, 0U, &sequence),
        COMMAND_ERROR_INVALID_FRAME_CRC,
        "frame error code remains available without frame bytes"
    );
    expect_u32(sequence, 0U, "missing frame bytes leave sequence unset");

    reset_fixture();
    expect_u32(command_task_finalize_incomplete_frame(NULL, 1U), 1U, "null timeout context is ignored");
    CommandTaskAssemblerContext_t timeout_context = {.client = &test_client, .partial_frame_active = 0U};
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 1U, "inactive partial frame is ignored");
    timeout_context.disconnect_requested = 1U;
    timeout_context.partial_frame_active = 1U;
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 1U, "disconnecting context skips finalization");

    reset_fixture();
    uint8_t stop_frame[COMMAND_MAX_FRAME_SIZE];
    (void)build_stop_command(stop_frame, 82U);
    memcpy(command_task_assembler.buffer, stop_frame, COMMAND_FRAME_HEADER_SIZE);
    command_task_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    timeout_context = (CommandTaskAssemblerContext_t){
        .client = &test_client,
        .partial_frame_active = 0U,
        .disconnect_requested = 0U,
    };
    test_tick_count = 100U;
    command_task_update_partial_frame_tracking(&timeout_context);
    expect_u32(timeout_context.partial_frame_active, 1U, "incomplete frame starts tracking");
    expect_u32(timeout_context.partial_frame_started_at, 100U, "partial frame start tick is captured");
    test_tick_count = 200U;
    command_task_update_partial_frame_tracking(&timeout_context);
    expect_u32(timeout_context.partial_frame_started_at, 100U, "active deadline is not restarted");
    expect_u32(command_task_reject_expired_incomplete_frame(NULL), 1U, "null timeout check is ignored");
    CommandTaskAssemblerContext_t inactive_timeout_context = {.partial_frame_active = 0U};
    expect_u32(
        command_task_reject_expired_incomplete_frame(&inactive_timeout_context),
        1U,
        "inactive partial frame is ignored by timeout check"
    );
    expect_u32(command_task_reject_expired_incomplete_frame(&timeout_context), 1U, "timeout before deadline is retained");
    test_tick_count = 100U + COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS;
    expect_u32(command_task_reject_expired_incomplete_frame(&timeout_context), 1U, "expired frame gets a truncation response");
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 82U, COMMAND_ERROR_TRUNCATED_FRAME);
    expect_u32(command_task_assembler.length, 0U, "timeout clears assembler state");

    reset_fixture();
    command_task_assembler.length = 3U;
    memcpy(command_task_assembler.buffer, stop_frame, 3U);
    timeout_context = (CommandTaskAssemblerContext_t){
        .client = &test_client,
        .partial_frame_active = 1U,
    };
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 1U, "short header records truncation without reply");
    expect_u32(test_netconn_write_calls, 0U, "no response is sent until sequence is parseable");

    reset_fixture();
    memcpy(command_task_assembler.buffer, stop_frame, COMMAND_FRAME_HEADER_SIZE);
    command_task_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    timeout_context = (CommandTaskAssemblerContext_t){
        .client = NULL,
        .partial_frame_active = 1U,
    };
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 0U, "failed truncation response closes connection");
    expect_u32(timeout_context.disconnect_requested, 1U, "failed truncation response marks disconnect");

    reset_fixture();
    expect_u32(command_task_send_response(NULL, ping, ping_length), 0U, "null client cannot receive a response");
    expect_u32(command_task_send_response(&test_client, NULL, ping_length), 0U, "null response bytes are rejected");
    expect_u32(command_task_send_response(&test_client, ping, 0U), 0U, "empty response is rejected");
    expect_u32(
        command_task_send_response(&test_client, ping, COMMAND_MAX_FRAME_SIZE + 1U),
        0U,
        "oversized response is rejected"
    );
    expect_u32(command_task_send_protocol_response(&test_client, 0xFEU, 1U, 0U), 0U, "unsupported response type is rejected");
    reset_fixture();
    test_write_mode = TEST_WRITE_PARTIAL_THEN_FULL;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 1U, "partial successful writes finish response");
    expect_u32(test_netconn_write_calls, 2U, "partial response continues from the remaining byte");
    expect_u32(command_task_diagnostics.partial_writes, 1U, "successful partial response is counted");
    expect_u32(command_task_diagnostics.response_successes, 1U, "completed response is counted");

    reset_fixture();
    test_write_mode = TEST_WRITE_OVERREPORT;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "overreported initial write is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "overreport records argument error");

    reset_fixture();
    test_write_mode = TEST_WRITE_PARTIAL_THEN_OVERREPORT;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "overreport after partial progress is rejected");
    expect_u32(command_task_diagnostics.partial_writes, 1U, "partial overreported response increments partial count");

    reset_fixture();
    test_write_result = ERR_OK;
    test_write_mode = TEST_WRITE_FAIL;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "zero-progress write is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "zero progress maps to buffer error");

    reset_fixture();
    test_write_result = ERR_BUF;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "transport write error is propagated");
    expect_u32(command_task_diagnostics.partial_writes, 0U, "failure before progress is not partial");

    reset_fixture();
    test_write_mode = TEST_WRITE_FAIL_AFTER_PARTIAL;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "failure after partial progress is rejected");
    expect_u32(command_task_diagnostics.partial_writes, 1U, "failure after progress is counted partial");

    reset_fixture();
    test_write_mode = TEST_WRITE_ERROR_WITH_PARTIAL_FIRST;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "write error with initial progress is rejected");
    expect_u32(command_task_diagnostics.partial_writes, 1U, "initial progress before error is counted partial");

    reset_fixture();
    test_write_mode = TEST_WRITE_ZERO_PROGRESS;
    expect_u32(command_task_send_response(&test_client, ping, ping_length), 0U, "zero progress without transport error is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "zero progress maps to buffer error");
}

static void test_incomplete_frame_tracking_edges(void) {
    reset_fixture();
    expect_u32(command_task_assembler_has_incomplete_command(NULL), 0U, "null assembler is not incomplete");
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 0U, "empty assembler is not incomplete");

    command_task_assembler.length = 1U;
    command_task_assembler.buffer[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 1U, "magic prefix is incomplete");
    command_task_assembler.buffer[0] ^= 1U;
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 0U, "unrelated byte is not incomplete");

    command_task_assembler.length = 2U;
    command_task_assembler.buffer[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    command_task_assembler.buffer[1] = (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U);
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 1U, "valid magic prefix is incomplete");
    command_task_assembler.buffer[1] ^= 1U;
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 0U, "bad second magic byte is discarded");

    uint8_t ping[COMMAND_MAX_FRAME_SIZE];
    const size_t ping_length = build_ping(ping, 86U);
    command_task_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    memcpy(command_task_assembler.buffer, ping, COMMAND_FRAME_HEADER_SIZE);
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 1U, "valid short header is incomplete");
    command_task_assembler.length = ping_length;
    memcpy(command_task_assembler.buffer, ping, ping_length);
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 0U, "complete frame is not incomplete");
    command_task_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    command_task_assembler.buffer[4] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU);
    command_task_assembler.buffer[5] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U);
    expect_u32(command_task_assembler_has_incomplete_command(&command_task_assembler), 0U, "invalid header length is not incomplete");

    CommandTaskAssemblerContext_t context = {.partial_frame_active = 1U};
    command_frame_assembler_init(&command_task_assembler);
    command_task_update_partial_frame_tracking(&context);
    expect_u32(context.partial_frame_active, 0U, "tracking clears when no partial command remains");
    command_task_update_partial_frame_tracking(NULL);

    command_task_assembler.length = 1U;
    command_task_assembler.buffer[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    context.partial_frame_active = 0U;
    test_tick_count = 87U;
    command_task_update_partial_frame_tracking(&context);
    expect_u32(context.partial_frame_active, 1U, "tracking starts for a magic prefix");
    expect_u32(context.partial_frame_started_at, 87U, "tracking records the first partial tick");
}

static void test_assembly_netbuf_segment_and_trailing_cases(void) {
    reset_fixture();
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 88U);
    struct netbuf buffer = {
        .parts = {frame, &frame[3]},
        .lengths = {3U, (u16_t)(frame_length - 3U)},
        .part_count = 2U,
        .current = 0,
        .data_result = ERR_OK,
    };
    CommandTaskAssemblerContext_t context = {0};
    CommandTaskAssemblyEvent_t event = command_task_assemble_netbuf(&buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_COMMAND_READY, "frame split across netbuf parts is assembled");
    expect_u32(event.command.header.sequence, 88U, "segmented netbuf returns the full request");

    reset_fixture();
    command_task_diagnostics.buffer_max_bytes = 100U;
    const uint8_t partial_byte[] = {0x5AU};
    buffer = (struct netbuf){
        .parts = {partial_byte},
        .lengths = {sizeof(partial_byte)},
        .part_count = 1U,
        .current = 0,
        .data_result = ERR_OK,
    };
    event = command_task_assemble_netbuf(&buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_NEED_MORE, "partial byte stays in assembler");
    expect_u32(command_task_diagnostics.buffer_max_bytes, 100U, "diagnostic high-water mark does not decrease");

    reset_fixture();
    uint8_t with_trailing_data[COMMAND_MAX_FRAME_SIZE + 20U] = {0};
    (void)memcpy(with_trailing_data, frame, frame_length);
    buffer = (struct netbuf){
        .parts = {with_trailing_data},
        .lengths = {(u16_t)sizeof(with_trailing_data)},
        .part_count = 1U,
        .current = 0,
        .data_result = ERR_OK,
    };
    event = command_task_assemble_netbuf(&buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "unappended bytes in a large segment are rejected");

    reset_fixture();
    uint8_t second_frame[COMMAND_MAX_FRAME_SIZE];
    const size_t second_length = build_ping(second_frame, 89U);
    uint8_t combined[COMMAND_MAX_FRAME_SIZE * 2U];
    memcpy(combined, frame, frame_length);
    memcpy(&combined[frame_length], second_frame, second_length);
    buffer = (struct netbuf){
        .parts = {combined},
        .lengths = {(u16_t)(frame_length + second_length)},
        .part_count = 1U,
        .current = 0,
        .data_result = ERR_OK,
    };
    context = (CommandTaskAssemblerContext_t){0};
    event = command_task_assemble_netbuf(&buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "second complete frame in one receive is rejected");
    expect_u32((uint32_t)command_task_assembler.length, (uint32_t)second_length, "assembler retains the unexpected second frame");
}

static void test_diagnostics_and_internal_edges(void) {
    command_task_get_diagnostics(NULL);
    CommandTaskDiagnostics output = {0};
    command_task_diagnostics = (CommandTaskDiagnostics){
        .connections_accepted = 1U,
        .connections_closed = 2U,
        .active_connection = 3U,
        .bytes_received = 4U,
        .complete_frames_received = 5U,
        .invalid_magic = 6U,
        .invalid_frame_length = 7U,
        .invalid_crc = 8U,
        .buffer_current_bytes = 9U,
        .buffer_max_bytes = 10U,
        .bytes_sent = 11U,
        .response_attempts = 12U,
        .response_successes = 13U,
        .response_send_errors = 14U,
        .partial_writes = 15U,
        .netconn_alloc_errors = 16U,
        .bind_errors = 17U,
        .listen_errors = 18U,
        .accept_errors = 19U,
        .recv_errors = 20U,
        .decoder_errors = 21U,
        .last_error = -22,
    };
    command_task_get_diagnostics(&output);
    expect_u32(output.port, COMMAND_NETWORK_TASK_PORT, "diagnostics include command port");
    expect_u32(output.connections_accepted, 1U, "diagnostics copy accepted connections");
    expect_u32(output.connections_closed, 2U, "diagnostics copy closed connections");
    expect_u32(output.active_connection, 3U, "diagnostics copy active flag");
    expect_u32(output.bytes_received, 4U, "diagnostics copy receive bytes");
    expect_u32(output.complete_frames_received, 5U, "diagnostics copy complete frames");
    expect_u32(output.invalid_magic, 6U, "diagnostics copy invalid magic");
    expect_u32(output.invalid_frame_length, 7U, "diagnostics copy invalid lengths");
    expect_u32(output.invalid_crc, 8U, "diagnostics copy invalid CRC");
    expect_u32(output.buffer_current_bytes, 9U, "diagnostics copy current buffer size");
    expect_u32(output.buffer_max_bytes, 10U, "diagnostics copy maximum buffer size");
    expect_u32(output.bytes_sent, 11U, "diagnostics copy sent bytes");
    expect_u32(output.response_attempts, 12U, "diagnostics copy response attempts");
    expect_u32(output.response_successes, 13U, "diagnostics copy successful responses");
    expect_u32(output.response_send_errors, 14U, "diagnostics copy response errors");
    expect_u32(output.partial_writes, 15U, "diagnostics copy partial writes");
    expect_u32(output.netconn_alloc_errors, 16U, "diagnostics copy allocation errors");
    expect_u32(output.bind_errors, 17U, "diagnostics copy bind errors");
    expect_u32(output.listen_errors, 18U, "diagnostics copy listen errors");
    expect_u32(output.accept_errors, 19U, "diagnostics copy accept errors");
    expect_u32(output.recv_errors, 20U, "diagnostics copy receive errors");
    expect_u32(output.decoder_errors, 21U, "diagnostics copy decoder errors");
    expect_u32((uint32_t)output.last_error, (uint32_t)-22, "diagnostics copy last error");

    volatile uint32_t counter = UINT32_MAX;
    command_counter_increment(&counter);
    expect_u32(counter, UINT32_MAX, "diagnostic counter saturates at maximum");
    counter = UINT32_MAX - 1U;
    command_counter_add(&counter, 2U);
    expect_u32(counter, UINT32_MAX, "diagnostic addition saturates on overflow");
    counter = 2U;
    command_counter_add(&counter, 3U);
    expect_u32(counter, 5U, "diagnostic addition succeeds without overflow");
    if (SIZE_MAX > UINT32_MAX) {
        counter = 0U;
        command_counter_add(&counter, (size_t)UINT32_MAX + 1U);
        expect_u32(counter, UINT32_MAX, "oversized size addition saturates");
    }

    CommandTaskAssemblerContext_t context = {.client = &test_client};
    expect_true(command_task_get_client(NULL) == NULL, "null assembler context has no client");
    expect_true(command_task_get_client(&context) == &test_client, "assembler context returns its client");
}

static void test_decoder_failure_and_error_response_send_failure(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    size_t frame_length = build_stop_command(frame, 91U);

    reset_fixture();
    set_buffer(0U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    test_force_decoder_invalid_argument = 1U;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "invalid decoder result closes current client");
    expect_u32(command_task_diagnostics.decoder_errors, 1U, "invalid decoder result is counted");
    expect_u32(test_netconn_write_calls, 0U, "invalid decoder result sends no ACK");

    reset_fixture();
    frame_length = build_stop_command(frame, 92U);
    set_buffer(0U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed ACK closes current connection");
    expect_u32(test_netconn_write_calls, 1U, "ACK write is attempted once");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failed ACK is counted");

    reset_fixture();
    frame_length = build_ping(frame, 93U);
    frame[frame_length - 1U] ^= 1U;
    set_buffer(0U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed malformed-frame ERROR closes connection");
    expect_u32(test_netconn_write_calls, 1U, "malformed-frame ERROR write is attempted once");
}

int main(void) {
    test_context_and_listener_failures();
    test_ping_ack_and_error_responses();
    test_fragmented_receive_and_disconnect_cleanup();
    test_receive_timeout_and_receive_errors();
    test_assembler_errors_and_boundary_paths();
    test_incomplete_frame_tracking_edges();
    test_assembly_netbuf_segment_and_trailing_cases();
    test_frame_errors_timeout_helpers_and_response_writes();
    test_diagnostics_and_internal_edges();
    test_decoder_failure_and_error_response_send_failure();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command task unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command task unit tests passed.\n");
    return 0;
}

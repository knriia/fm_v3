#include "command_decoder.h"
#include "command_protocol.h"
#include "command/command_queue.h"
#include "command/command_result_queue.h"
#include "command_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "main.h"
#include "network_events.h"
#include "task_context.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_SCRIPT_LENGTH 32U
#define TEST_PBUF_SEGMENTS 3U
#define TEST_THREAD_ID ((osThreadId_t)(uintptr_t)0x1234U)

static uint32_t test_failures;
static uint32_t test_error_handler_calls;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint8_t test_jump_on_delay;
static uint32_t test_jump_after_new_calls;
static uint32_t test_jump_after_accept_calls;
static uint8_t test_new_returns_null;
static uint32_t test_event_flags_result;
static err_t test_bind_result;
static err_t test_listen_result;
static err_t test_accept_result;
static err_t test_default_recv_result;
static err_t test_write_result;
static uint32_t test_tick_count;
static uint32_t test_tick_after_receive[TEST_SCRIPT_LENGTH];
static uint8_t test_tick_after_receive_enabled[TEST_SCRIPT_LENGTH];
static uint32_t test_netconn_new_calls;
static uint32_t test_netconn_bind_calls;
static uint32_t test_netconn_listen_calls;
static uint32_t test_netconn_accept_calls;
static uint32_t test_netconn_recv_calls;
static uint32_t test_netconn_write_calls;
static uint32_t test_netconn_close_calls;
static uint32_t test_netconn_delete_calls;
static uint32_t test_pbuf_free_calls;
static uint8_t test_recv_apiflags;
static netconn_callback test_listener_callback;
static uint32_t test_thread_flags_clear_calls;
static uint32_t test_thread_flags_cleared;
static uint32_t test_thread_flags_set_calls;
static uint32_t test_thread_flags_set_value;
static osThreadId_t test_thread_flags_set_target;
static uint32_t test_thread_flags_wait_calls;
static uint32_t test_thread_flags_wait_timeouts[TEST_SCRIPT_LENGTH];
static uint8_t test_write_mode;
static uint8_t test_write_blocked;
static int test_write_apiflags;
static size_t test_captured_response_length;
static uint8_t test_captured_response[COMMAND_MAX_FRAME_SIZE * 4U];
static uint8_t test_force_assembler_next;
static uint8_t test_force_assembler_append_failure;
static uint8_t test_force_assembler_consume_failure;
static uint8_t test_force_decoder_invalid_argument;
static CommandQueuePutResult_t test_command_queue_put_result;
static uint32_t test_command_queue_put_calls;
static CommandDTO_t test_put_commands[8];
static uint32_t test_put_stop_requests_before[8];
static uint32_t test_queue_free_slots[TEST_SCRIPT_LENGTH];
static uint32_t test_queue_free_slots_calls;
static CommandQueueSpaceListener_t test_space_listener;
static CommandFrameAssembler_t test_assembler;
/* The results MotionTask has left in the result queue: delivered in order by try_receive, dropped by discard_all. */
#define TEST_RESULT_SCRIPT_LENGTH 8U
static CommandResult_t test_result_script[TEST_RESULT_SCRIPT_LENGTH];
static uint32_t test_result_script_count;
static uint32_t test_result_script_position;
static uint32_t test_result_receive_calls;
static uint32_t test_result_discard_calls;
/* Whether a discard erases the scripted results; the others are scripted for after the client has connected. */
static uint8_t test_result_discard_erases;
/* Scripted results become visible only once this many flag waits have started; 0 means from the beginning. */
static uint32_t test_result_visible_from_wait;
static CommandResultListener_t test_result_listener;
/* How many results had been taken when each wait for flags began. */
static uint32_t test_results_taken_at_wait[TEST_SCRIPT_LENGTH];

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

typedef struct {
    err_t result;
    struct pbuf *buffer;
} TestReceive_t;

typedef struct {
    uint32_t result;
    uint8_t set_tick;
    uint32_t tick;
    uint8_t unblock_writes;
} TestFlagsWait_t;

static struct netconn test_listener;
static struct netconn test_client;
static TestReceive_t test_receives[TEST_SCRIPT_LENGTH];
static uint32_t test_receive_script_count;
static struct pbuf test_buffers[TEST_SCRIPT_LENGTH][TEST_PBUF_SEGMENTS];
static TestFlagsWait_t test_flags_waits[TEST_SCRIPT_LENGTH];
static uint32_t test_flags_wait_script_count;

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

osThreadId_t osThreadGetId(void) { return TEST_THREAD_ID; }

uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags) {
    ++test_thread_flags_set_calls;
    test_thread_flags_set_target = thread_id;
    test_thread_flags_set_value |= flags;
    return test_thread_flags_set_value;
}

uint32_t osThreadFlagsClear(uint32_t flags) {
    ++test_thread_flags_clear_calls;
    test_thread_flags_cleared = flags;
    return 0U;
}

struct netconn *netconn_new_with_callback(int type, netconn_callback callback) {
    (void)type;
    ++test_netconn_new_calls;
    test_listener_callback = callback;
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
        *new_connection = &test_client;
    }
    return test_accept_result;
}

err_t netconn_recv_tcp_pbuf_flags(struct netconn *connection, struct pbuf **buffer, u8_t apiflags) {
    (void)connection;
    test_recv_apiflags = apiflags;
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

err_t netconn_write_partly(
    struct netconn *connection,
    const void *data,
    size_t size,
    int apiflags,
    size_t *bytes_written
) {
    (void)connection;
    test_write_apiflags = apiflags;
    ++test_netconn_write_calls;
    if (test_write_blocked != 0U) {
        *bytes_written = 0U;
        return ERR_WOULDBLOCK;
    }

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

u8_t pbuf_free(struct pbuf *buffer) {
    (void)buffer;
    ++test_pbuf_free_calls;
    return 1U;
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

/* Scripted wait results; once the script ends the task is woken with RX so it reads the default result. */
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout) {
    (void)flags;
    (void)options;
    const uint32_t index = test_thread_flags_wait_calls;
    ++test_thread_flags_wait_calls;
    if (index < TEST_SCRIPT_LENGTH) {
        test_thread_flags_wait_timeouts[index] = timeout;
        test_results_taken_at_wait[index] = test_result_script_position;
    }
    if (index < test_flags_wait_script_count) {
        if (test_flags_waits[index].set_tick != 0U) {
            test_tick_count = test_flags_waits[index].tick;
        }
        if (test_flags_waits[index].unblock_writes != 0U) {
            test_write_blocked = 0U;
        }
        return test_flags_waits[index].result;
    }
    return COMMAND_TASK_FLAG_RX;
}

/* The order of the calls that stop motion: how many commands had been queued at each request_stop, and the command and
 * the number of request_stop calls before each queued command. */
#define TEST_CALL_LOG_LENGTH 8U
static uint32_t test_motion_stop_requests;
static uint32_t test_stop_request_puts_before[TEST_CALL_LOG_LENGTH];
static uint32_t test_stop_request_deletes_before[TEST_CALL_LOG_LENGTH];

void motion_task_request_stop(void) {
    if (test_motion_stop_requests < TEST_CALL_LOG_LENGTH) {
        test_stop_request_puts_before[test_motion_stop_requests] = test_command_queue_put_calls;
        test_stop_request_deletes_before[test_motion_stop_requests] = test_netconn_delete_calls;
    }
    ++test_motion_stop_requests;
}

CommandQueuePutResult_t command_queue_try_send(const CommandDTO_t *command) {
    if (test_command_queue_put_calls < TEST_CALL_LOG_LENGTH) {
        test_put_commands[test_command_queue_put_calls] = *command;
        test_put_stop_requests_before[test_command_queue_put_calls] = test_motion_stop_requests;
    }
    ++test_command_queue_put_calls;
    return test_command_queue_put_result;
}

/* Free slots after each accepted command; unscripted calls report room so the ACK is sent at once. */
uint32_t command_queue_free_slots(void) {
    const uint32_t index = test_queue_free_slots_calls;
    ++test_queue_free_slots_calls;
    if (index < TEST_SCRIPT_LENGTH && test_queue_free_slots[index] != UINT32_MAX) {
        return test_queue_free_slots[index];
    }
    return 1U;
}

void command_queue_set_space_listener(CommandQueueSpaceListener_t listener) { test_space_listener = listener; }

void command_result_queue_set_listener(CommandResultListener_t listener) { test_result_listener = listener; }

osStatus_t command_result_queue_try_receive(CommandResult_t *result) {
    ++test_result_receive_calls;
    if (test_thread_flags_wait_calls < test_result_visible_from_wait ||
        test_result_script_position >= test_result_script_count) {
        return osErrorResource;
    }
    *result = test_result_script[test_result_script_position++];
    return osOK;
}

void command_result_queue_discard_all(void) {
    ++test_result_discard_calls;
    if (test_result_discard_erases != 0U) {
        test_result_script_position = test_result_script_count;
    }
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
    test_pbuf_free_calls = 0U;
    test_recv_apiflags = 0U;
    test_listener_callback = NULL;
    test_thread_flags_clear_calls = 0U;
    test_thread_flags_cleared = 0U;
    test_thread_flags_set_calls = 0U;
    test_thread_flags_set_value = 0U;
    test_thread_flags_set_target = NULL;
    test_thread_flags_wait_calls = 0U;
    memset(test_thread_flags_wait_timeouts, 0, sizeof(test_thread_flags_wait_timeouts));
    test_write_mode = TEST_WRITE_FULL;
    test_write_blocked = 0U;
    test_write_apiflags = 0;
    test_captured_response_length = 0U;
    test_receive_script_count = 0U;
    test_flags_wait_script_count = 0U;
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_REAL;
    test_force_assembler_append_failure = 0U;
    test_force_assembler_consume_failure = 0U;
    test_force_decoder_invalid_argument = 0U;
    test_command_queue_put_result = COMMAND_QUEUE_PUT_OK;
    test_command_queue_put_calls = 0U;
    test_motion_stop_requests = 0U;
    memset(test_stop_request_puts_before, 0, sizeof(test_stop_request_puts_before));
    memset(test_stop_request_deletes_before, 0, sizeof(test_stop_request_deletes_before));
    memset(test_put_commands, 0, sizeof(test_put_commands));
    memset(test_put_stop_requests_before, 0, sizeof(test_put_stop_requests_before));
    memset(test_queue_free_slots, 0xFF, sizeof(test_queue_free_slots));
    test_queue_free_slots_calls = 0U;
    test_space_listener = NULL;
    memset(test_result_script, 0, sizeof(test_result_script));
    test_result_script_count = 0U;
    test_result_script_position = 0U;
    test_result_receive_calls = 0U;
    test_result_discard_calls = 0U;
    test_result_discard_erases = 0U;
    test_result_visible_from_wait = 0U;
    test_result_listener = NULL;
    memset(test_results_taken_at_wait, 0, sizeof(test_results_taken_at_wait));
    memset(test_receives, 0, sizeof(test_receives));
    memset(test_buffers, 0, sizeof(test_buffers));
    memset(test_flags_waits, 0, sizeof(test_flags_waits));
    memset(test_captured_response, 0, sizeof(test_captured_response));
    command_task_diagnostics = (CommandTaskDiagnostics){0};
    command_task_thread_id = NULL;
    command_frame_assembler_init(&test_assembler);
}

static CommandTaskAssemblerContext_t test_context(void) {
    return (CommandTaskAssemblerContext_t){
        .client = &test_client,
        .assembler = &test_assembler,
    };
}

/* Links up to three segments into one pbuf chain; tot_len of each segment covers the rest of the chain. */
static struct pbuf *test_chain(
    struct pbuf *segments,
    const uint8_t *first,
    u16_t first_length,
    const uint8_t *second,
    u16_t second_length,
    const uint8_t *third,
    u16_t third_length
) {
    const uint8_t *parts[TEST_PBUF_SEGMENTS] = {first, second, third};
    const u16_t lengths[TEST_PBUF_SEGMENTS] = {first_length, second_length, third_length};
    uint8_t count = 1U;
    if (second != NULL || second_length != 0U) {
        count = 2U;
    }
    if (third != NULL || third_length != 0U) {
        count = 3U;
    }

    u16_t remaining = 0U;
    for (uint8_t index = count; index > 0U; --index) {
        struct pbuf *segment = &segments[index - 1U];
        remaining = (u16_t)(remaining + lengths[index - 1U]);
        segment->payload = (void *)(uintptr_t)parts[index - 1U];
        segment->len = lengths[index - 1U];
        segment->tot_len = remaining;
        segment->next = index < count ? &segments[index] : NULL;
    }
    return &segments[0];
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
    test_receives[receive_index].buffer = test_chain(
        test_buffers[receive_index],
        first,
        first_length,
        second,
        second_length,
        third,
        third_length
    );
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

static void set_flags_wait(uint32_t wait_index, uint32_t result) {
    test_flags_waits[wait_index].result = result;
    if (test_flags_wait_script_count <= wait_index) {
        test_flags_wait_script_count = wait_index + 1U;
    }
}

static void set_flags_wait_timeout_at(uint32_t wait_index, uint32_t tick_count) {
    set_flags_wait(wait_index, osFlagsErrorTimeout);
    test_flags_waits[wait_index].set_tick = 1U;
    test_flags_waits[wait_index].tick = tick_count;
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
    const uint8_t has_code = type == COMMAND_MESSAGE_TYPE_ERROR || type == COMMAND_MESSAGE_TYPE_DONE ? 1U : 0U;
    /* Header 10 + payload (ERROR 5, DONE 2, others 0) + CRC 4. */
    const size_t expected_length = type == COMMAND_MESSAGE_TYPE_ERROR ? 19U : (has_code != 0U ? 16U : 14U);
    expect_u32((uint32_t)test_captured_response_length, (uint32_t)expected_length, "exactly one response is sent");
    CommandFrameHeaderDTO_t header = {0};
    expect_u32(
        command_protocol_validate_frame(test_captured_response, test_captured_response_length, &header),
        COMMAND_FRAME_VALID,
        "response is a valid protocol frame"
    );
    expect_u32(header.type, type, "response type matches the current protocol");
    expect_u32(header.sequence, sequence, "response preserves request sequence");
    if (has_code != 0U) {
        const uint16_t actual_error = (uint16_t)test_captured_response[COMMAND_FRAME_HEADER_SIZE] |
            ((uint16_t)test_captured_response[COMMAND_FRAME_HEADER_SIZE + 1U] << 8U);
        expect_u32(actual_error, error_code, "response contains the expected code");
    }
    if (type == COMMAND_MESSAGE_TYPE_ERROR) {
        expect_u32(header.payload_length, 5U, "an ERROR payload is five bytes");
        expect_u32(test_captured_response[COMMAND_FRAME_HEADER_SIZE + 2U], 0x00U, "a refusal has cause 0");
        expect_u32(test_captured_response[COMMAND_FRAME_HEADER_SIZE + 3U], 0xFFU, "a refusal names no axis");
        expect_u32(test_captured_response[COMMAND_FRAME_HEADER_SIZE + 4U], 0x00U, "a refusal has detail 0");
    }
}

static void test_context_and_listener_failures(void) {
    NetworkTaskContext valid_context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    expect_u32((uint32_t)run_task_until_jump(NULL), 1U, "null task context stops the task");
    expect_u32(test_error_handler_calls, 1U, "null task context reports a fatal error");

    reset_fixture();
    NetworkTaskContext null_flags = {.lwip_flags = NULL};
    expect_u32((uint32_t)run_task_until_jump(&null_flags), 1U, "null event flags stop the task");
    expect_u32(test_error_handler_calls, 1U, "null event flags report a fatal error");

    reset_fixture();
    test_event_flags_result = osFlagsErrorParameter;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "LwIP readiness error stops the task");
    expect_u32(test_error_handler_calls, 1U, "LwIP readiness error reports a fatal error");
    expect_u32(test_netconn_new_calls, 0U, "listener is not created before LwIP is ready");

    reset_fixture();
    test_new_returns_null = 1U;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "listener allocation retry is interruptible in test");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_MEM, "allocation failure is recorded");
    expect_true(test_listener_callback == command_task_netconn_event, "listener is created with the event callback");
    expect_true(command_task_thread_id == TEST_THREAD_ID, "task stores its id for the callback");
    expect_true(test_space_listener == command_task_queue_space_available, "task listens for free queue space");
    expect_true(test_result_listener == command_task_result_available, "task listens for new command results");

    reset_fixture();
    test_bind_result = ERR_MEM;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "bind retry is interruptible in test");
    expect_u32(test_netconn_bind_calls, 1U, "listener bind is attempted");
    expect_u32(test_netconn_listen_calls, 0U, "listen is skipped after bind failure");
    expect_u32(test_netconn_delete_calls, 1U, "unbound listener is deleted");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_MEM, "bind failure is recorded");

    reset_fixture();
    test_listen_result = ERR_MEM;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "listen retry is interruptible in test");
    expect_u32(test_netconn_close_calls, 1U, "listener is closed after listen failure");
    expect_u32(test_netconn_delete_calls, 1U, "listener is deleted after listen failure");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_MEM, "listen failure is recorded");

    reset_fixture();
    test_accept_result = ERR_MEM;
    test_jump_after_new_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "accept error exits the current listener");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_MEM, "accept error is recorded");
    expect_u32(test_netconn_close_calls, 1U, "listener is closed after accept error");

    reset_fixture();
    test_accept_result = ERR_ABRT;
    test_jump_on_delay = 1U;
    expect_u32((uint32_t)run_task_until_jump(&valid_context), 1U, "aborted accept waits before retrying");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ABRT, "aborted accept records ERR_ABRT");
    expect_u32(test_netconn_close_calls, 0U, "listener is kept after aborted accept");
    expect_u32(test_netconn_new_calls, 1U, "listener is not recreated after aborted accept");
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
    expect_u32(test_command_queue_put_calls, 1U, "only the STOP of the disconnect is placed in the command queue");
    expect_u32(test_put_commands[0].sequence, COMMAND_SEQUENCE_INTERNAL, "PING itself is not queued: the STOP is internal");
    expect_u32(command_task_diagnostics.complete_frames_received, 1U, "complete PING is counted");

    reset_fixture();
    frame_length = build_stop_command(frame, 42U);
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ACK, 42U, 0U);
    expect_u32(test_netconn_write_calls, 1U, "accepted command receives only one ACK");
    expect_u32(test_command_queue_put_calls, 2U, "the client STOP and the STOP of the disconnect are queued");
    expect_u32(test_motion_stop_requests, 2U, "the client STOP and the disconnect each stop motion once");
    expect_u32(test_stop_request_puts_before[0], 0U, "STOP acts before it is queued");
    expect_u32(test_put_stop_requests_before[0], 1U, "the client STOP is queued after its request_stop");
    expect_u32(test_put_commands[0].sequence, 42U, "queued command preserves sequence");
    expect_u32(test_put_commands[0].code, COMMAND_CODE_STOP, "queued command preserves command code");
    expect_u32(test_stop_request_puts_before[1], 1U, "the disconnect stops motion after the client STOP was queued");
    expect_u32(test_put_stop_requests_before[1], 2U, "the STOP of the disconnect is queued after its request_stop");
    expect_u32(test_put_commands[1].sequence, COMMAND_SEQUENCE_INTERNAL, "the STOP of the disconnect is internal");
    expect_u32(test_put_commands[1].code, COMMAND_CODE_STOP, "the disconnect queues a STOP");

    reset_fixture();
    const uint8_t home_payload[] = {COMMAND_CODE_HOME, 0x01U};
    frame_length = command_protocol_build_frame(
        frame,
        sizeof(frame),
        COMMAND_MESSAGE_TYPE_COMMAND,
        48U,
        home_payload,
        sizeof(home_payload)
    );
    run_one_receive(frame, frame_length);
    expect_single_response(COMMAND_MESSAGE_TYPE_ACK, 48U, 0U);
    expect_u32(test_motion_stop_requests, 1U, "other commands do not stop motion: only the disconnect does");
    expect_u32(test_command_queue_put_calls, 2U, "the HOME and the STOP of the disconnect are queued");
    expect_u32(test_put_commands[0].code, COMMAND_CODE_HOME, "the HOME is queued first");
    expect_u32(test_put_stop_requests_before[0], 0U, "the HOME is queued without a stop before it");
    expect_u32(test_stop_request_puts_before[0], 1U, "the stop of the disconnect comes after the HOME");

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
    expect_u32(test_pbuf_free_calls, 2U, "each received buffer is released");
    expect_u32(test_recv_apiflags, NETCONN_DONTBLOCK, "client data is read without blocking");
    expect_u32(test_thread_flags_wait_calls, 0U, "data already queued is read before waiting for flags");
    expect_u32(test_thread_flags_clear_calls, 1U, "task flags are cleared for the new client");
    expect_u32(test_thread_flags_cleared, COMMAND_TASK_FLAGS_ALL, "all task flags are cleared for the new client");
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
    expect_u32(command_task_diagnostics.invalid_frames, 1U, "partial frame from prior connection is discarded");
}

static void test_flag_waits_and_receive_errors(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 61U);

    reset_fixture();
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_buffer(1U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(1U, ERR_OK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_RX);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "RX flag wakes the task for new data");
    expect_single_response(COMMAND_MESSAGE_TYPE_PONG, 61U, 0U);
    expect_u32(test_thread_flags_wait_timeouts[0], osWaitForever, "task waits forever without a partial frame");

    reset_fixture();
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_TX);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "TX flag alone is ignored");
    expect_u32(test_thread_flags_wait_calls, 2U, "TX flag leads back to waiting");
    expect_u32(test_netconn_recv_calls, 2U, "TX flag does not trigger a read");

    reset_fixture();
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_receive_result(1U, ERR_RST);
    set_flags_wait(0U, COMMAND_TASK_FLAG_ERROR);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "ERROR flag closes the client through the read path");
    expect_u32(command_task_diagnostics.recv_errors, 1U, "connection error is counted as a receive error");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_RST, "connection error code is recorded");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "client is closed after a connection error");

    reset_fixture();
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, osFlagsErrorParameter);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "task flag wait error stops the task");
    expect_u32(test_error_handler_calls, 1U, "task flag wait error is fatal");

    reset_fixture();
    set_buffer(0U, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    test_write_blocked = 1U;
    test_write_mode = TEST_WRITE_FAIL;
    set_flags_wait(0U, COMMAND_TASK_FLAG_TX);
    test_flags_waits[0].unblock_writes = 1U;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed write on the TX flag closes the client");
    expect_u32(test_netconn_write_calls, 2U, "buffered response is retried on the TX flag");
    expect_u32(test_netconn_recv_calls, 2U, "socket is not read after a failed write");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failed buffered write is counted");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "client is closed after a failed buffered write");

    reset_fixture();
    set_buffer(0U, frame, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_flags_wait_timeout_at(0U, COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS - 1U);
    set_flags_wait(1U, COMMAND_TASK_FLAG_RX);
    set_receive_result(2U, ERR_CONN);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "nonexpired frame survives a wait timeout");
    expect_u32(
        test_thread_flags_wait_timeouts[0],
        pdMS_TO_TICKS(COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS),
        "partial frame limits the wait to its deadline"
    );
    expect_u32(test_thread_flags_wait_timeouts[1], 1U, "wait resumes with the remaining deadline");
    expect_u32(test_netconn_write_calls, 0U, "no response is sent for a truncated request");
    expect_u32(command_task_diagnostics.invalid_frames, 1U, "disconnect records a truncated frame");

    reset_fixture();
    set_buffer(0U, frame, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_flags_wait_timeout_at(0U, COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed timeout ERROR closes the client");
    expect_u32(test_netconn_write_calls, 1U, "truncation ERROR is attempted once");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failed timeout response is counted");

    reset_fixture();
    set_buffer(0U, frame, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_tick_after_receive(1U, COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "frame expired before waiting is rejected at once");
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 61U, COMMAND_ERROR_TRUNCATED_FRAME);
    expect_u32(test_thread_flags_wait_timeouts[0], osWaitForever, "task waits forever after rejecting the frame");

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
    set_buffer(0U, NULL, COMMAND_FRAME_HEADER_SIZE, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "pbuf assembly error closes current connection");
    expect_u32(command_task_diagnostics.recv_errors, 1U, "pbuf assembly error is counted");
    expect_u32(test_pbuf_free_calls, 1U, "buffer is released after assembly error");
}

static void test_netconn_event_callback(void) {
    reset_fixture();
    command_task_netconn_event(&test_client, NETCONN_EVT_RCVPLUS, 1U);
    expect_u32(test_thread_flags_set_calls, 0U, "callback is ignored before the task id is known");

    command_task_thread_id = TEST_THREAD_ID;
    command_task_netconn_event(&test_client, NETCONN_EVT_RCVPLUS, 1U);
    expect_u32(test_thread_flags_set_value, COMMAND_TASK_FLAG_RX, "received data sets the RX flag");
    expect_true(test_thread_flags_set_target == TEST_THREAD_ID, "flag goes to the command task");

    test_thread_flags_set_value = 0U;
    command_task_netconn_event(&test_client, NETCONN_EVT_SENDPLUS, 0U);
    expect_u32(test_thread_flags_set_value, COMMAND_TASK_FLAG_TX, "free send space sets the TX flag");

    test_thread_flags_set_value = 0U;
    command_task_netconn_event(&test_client, NETCONN_EVT_ERROR, 0U);
    expect_u32(test_thread_flags_set_value, COMMAND_TASK_FLAG_ERROR, "connection error sets the ERROR flag");

    test_thread_flags_set_calls = 0U;
    command_task_netconn_event(&test_client, NETCONN_EVT_RCVMINUS, 0U);
    command_task_netconn_event(&test_client, NETCONN_EVT_SENDMINUS, 0U);
    expect_u32(test_thread_flags_set_calls, 0U, "consumed-data events set no flags");
}

static void expect_ack_at(size_t offset, uint32_t sequence, const char *message) {
    CommandFrameHeaderDTO_t header = {0};
    const size_t ack_length = 14U;
    const uint8_t present = test_captured_response_length >= offset + ack_length ? 1U : 0U;
    expect_true(present, message);
    if (present == 0U) {
        return;
    }
    expect_u32(
        command_protocol_validate_frame(&test_captured_response[offset], ack_length, &header),
        COMMAND_FRAME_VALID,
        message
    );
    expect_u32(header.type, COMMAND_MESSAGE_TYPE_ACK, message);
    expect_u32(header.sequence, sequence, message);
}

static void test_deferred_ack(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t first[COMMAND_MAX_FRAME_SIZE];
    uint8_t second[COMMAND_MAX_FRAME_SIZE];
    const size_t first_length = build_stop_command(first, 101U);
    const size_t second_length = build_stop_command(second, 102U);

    reset_fixture();
    test_queue_free_slots[0] = 0U;
    set_buffer(0U, first, (u16_t)first_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_QUEUE_SPACE);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "command that fills the queue is accepted");
    expect_u32(test_command_queue_put_calls, 2U, "command filling the queue and the STOP of the disconnect are queued");
    expect_u32(command_task_diagnostics.acks_deferred, 1U, "held ACK is counted");
    expect_single_response(COMMAND_MESSAGE_TYPE_ACK, 101U, 0U);
    expect_u32(test_thread_flags_wait_calls, 2U, "ACK waits for the queue space flag");
    expect_u32(test_netconn_recv_calls, 3U, "queue space flag alone does not read the socket");

    reset_fixture();
    test_queue_free_slots[0] = 0U;
    set_buffer(0U, first, (u16_t)first_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_TX);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "held ACK survives unrelated flags");
    expect_u32((uint32_t)test_captured_response_length, 0U, "ACK is not sent without free queue space");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "client closes with the ACK still held");

    reset_fixture();
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_QUEUE_SPACE);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "queue space without a held ACK is ignored");
    expect_u32(test_netconn_write_calls, 0U, "nothing is sent without a held ACK");

    reset_fixture();
    test_queue_free_slots[0] = 0U;
    test_queue_free_slots[1] = 1U;
    set_buffer(0U, first, (u16_t)first_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_buffer(1U, second, (u16_t)second_length, NULL, 0U, NULL, 0U);
    set_receive_result(1U, ERR_OK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "held ACK is released by the next queued command");
    expect_u32((uint32_t)test_captured_response_length, 28U, "both commands are acknowledged");
    expect_ack_at(0U, 101U, "older held ACK is sent first");
    expect_ack_at(14U, 102U, "newer ACK follows the held one");

    reset_fixture();
    test_queue_free_slots[0] = 0U;
    set_buffer(0U, first, (u16_t)first_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_QUEUE_SPACE);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed held ACK closes the client");
    expect_u32(test_netconn_write_calls, 1U, "held ACK is attempted once");
    expect_u32(test_netconn_recv_calls, 2U, "socket is not read after a failed held ACK");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failed held ACK is counted");

    reset_fixture();
    test_queue_free_slots[0] = 0U;
    test_queue_free_slots[1] = 1U;
    set_buffer(0U, first, (u16_t)first_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_buffer(1U, second, (u16_t)second_length, NULL, 0U, NULL, 0U);
    set_receive_result(1U, ERR_OK);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "failed held ACK before the next ACK closes the client");
    expect_u32(
        test_command_queue_put_calls,
        3U,
        "both commands were queued before the held ACK failed, and the disconnect queued its STOP"
    );
    expect_u32(test_netconn_write_calls, 1U, "only the held ACK is attempted");
    expect_u32(test_queue_free_slots_calls, 1U, "next ACK is not prepared after the held ACK failed");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "client closes after the failed held ACK");

    reset_fixture();
    command_task_queue_space_available();
    expect_u32(test_thread_flags_set_calls, 0U, "queue space is ignored before the task id is known");
    command_task_thread_id = TEST_THREAD_ID;
    command_task_queue_space_available();
    expect_u32(test_thread_flags_set_value, COMMAND_TASK_FLAG_QUEUE_SPACE, "queue space sets its task flag");
}

static void test_flags_wait_timeout_helper(void) {
    reset_fixture();
    CommandTaskAssemblerContext_t context = test_context();
    expect_u32(command_task_flags_wait_timeout(&context), osWaitForever, "no partial frame waits forever");

    context.partial_frame_active = 1U;
    context.partial_frame_started_at = 100U;
    test_tick_count = 400U;
    expect_u32(
        command_task_flags_wait_timeout(&context),
        pdMS_TO_TICKS(COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS) - 300U,
        "partial frame waits for the rest of its deadline"
    );

    test_tick_count = 100U + COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS;
    expect_u32(command_task_flags_wait_timeout(&context), 0U, "expired partial frame does not wait");
}

static void test_assembler_errors_and_boundary_paths(void) {
    struct pbuf segments[TEST_PBUF_SEGMENTS];
    CommandTaskAssemblerContext_t context;

    reset_fixture();
    struct pbuf *buffer = test_chain(segments, (const uint8_t[]){1U}, 1U, NULL, 0U, NULL, 0U);
    CommandTaskAssemblyEvent_t event = command_task_assemble_pbuf(buffer, NULL);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "missing context is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "missing context records ERR_ARG");

    reset_fixture();
    context = (CommandTaskAssemblerContext_t){.client = &test_client};
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "context without an assembler is rejected");

    reset_fixture();
    context = test_context();
    event = command_task_assemble_pbuf(NULL, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "missing pbuf is rejected");

    reset_fixture();
    buffer = test_chain(segments, NULL, 1U, NULL, 0U, NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "null data with nonzero length is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "null data records ERR_BUF");

    reset_fixture();
    buffer = test_chain(segments, NULL, 0U, NULL, 0U, NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_NEED_MORE, "empty pbuf segment is ignored");

    reset_fixture();
    buffer = test_chain(segments, (const uint8_t[]){1U}, 1U, NULL, 0U, NULL, 0U);
    test_assembler.length = COMMAND_MAX_FRAME_SIZE + 1U;
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "invalid assembler state is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "invalid assembler state records ERR_ARG");

    reset_fixture();
    buffer = test_chain(segments, (const uint8_t[]){1U}, 1U, NULL, 0U, NULL, 0U);
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_NEED_MORE;
    test_force_assembler_append_failure = 1U;
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "zero-byte append is handled as assembly failure");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "zero-byte append records ERR_BUF");

    reset_fixture();
    buffer = test_chain(segments, (const uint8_t[]){1U}, 1U, NULL, 0U, NULL, 0U);
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_INVALID_ARGUMENT;
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "assembler invalid-argument event is propagated");

    reset_fixture();
    buffer = test_chain(segments, (const uint8_t[]){1U}, 1U, NULL, 0U, NULL, 0U);
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_INVALID_FRAME_VALIDATION;
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "unmapped frame validation result becomes error");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "unmapped frame result records ERR_ARG");

    reset_fixture();
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    size_t frame_length = build_stop_command(frame, 71U);
    buffer = test_chain(segments, frame, (u16_t)frame_length, NULL, 0U, NULL, 0U);
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_COMMAND_READY;
    test_force_assembler_consume_failure = 1U;
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "assembler consume failure is handled");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "consume failure records ERR_BUF");

    reset_fixture();
    test_assembler.length = 1U;
    buffer = test_chain(segments, (const uint8_t[]){1U}, 1U, NULL, 0U, NULL, 0U);
    test_force_assembler_next = TEST_ASSEMBLER_NEXT_COMMAND_READY;
    context = test_context();
    context.disconnect_requested = 1U;
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "pending disconnect stops assembly after consume");

    reset_fixture();
    frame_length = build_stop_command(frame, 72U);
    const uint8_t trailing[] = {0xA5U};
    buffer = test_chain(segments, frame, (u16_t)frame_length, trailing, sizeof(trailing), NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "trailing bytes after a complete frame are rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_VAL, "trailing bytes record ERR_VAL");

    reset_fixture();
    frame_length = build_stop_command(frame, 73U);
    const uint8_t next_frame[COMMAND_MAX_FRAME_SIZE] = {0};
    buffer = test_chain(segments, frame, (u16_t)frame_length, next_frame, 1U, NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "unread later pbuf segments are rejected");
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
    CommandTaskAssemblerContext_t timeout_context = test_context();
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 1U, "inactive partial frame is ignored");
    timeout_context.disconnect_requested = 1U;
    timeout_context.partial_frame_active = 1U;
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 1U, "disconnecting context skips finalization");

    reset_fixture();
    uint8_t stop_frame[COMMAND_MAX_FRAME_SIZE];
    (void)build_stop_command(stop_frame, 82U);
    memcpy(test_assembler.buffer, stop_frame, COMMAND_FRAME_HEADER_SIZE);
    test_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    timeout_context = test_context();
    test_tick_count = 100U;
    command_task_update_partial_frame_tracking(&timeout_context);
    expect_u32(timeout_context.partial_frame_active, 1U, "incomplete frame starts tracking");
    expect_u32(timeout_context.partial_frame_started_at, 100U, "partial frame start tick is captured");
    test_tick_count = 200U;
    command_task_update_partial_frame_tracking(&timeout_context);
    expect_u32(timeout_context.partial_frame_started_at, 100U, "active deadline is not restarted");
    expect_u32(command_task_reject_expired_incomplete_frame(NULL), 1U, "null timeout check is ignored");
    CommandTaskAssemblerContext_t inactive_timeout_context = test_context();
    expect_u32(
        command_task_reject_expired_incomplete_frame(&inactive_timeout_context),
        1U,
        "inactive partial frame is ignored by timeout check"
    );
    expect_u32(command_task_reject_expired_incomplete_frame(&timeout_context), 1U, "timeout before deadline is retained");
    test_tick_count = 100U + COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS;
    expect_u32(command_task_reject_expired_incomplete_frame(&timeout_context), 1U, "expired frame gets a truncation response");
    expect_single_response(COMMAND_MESSAGE_TYPE_ERROR, 82U, COMMAND_ERROR_TRUNCATED_FRAME);
    expect_u32(test_assembler.length, 0U, "timeout clears assembler state");

    reset_fixture();
    test_assembler.length = 3U;
    memcpy(test_assembler.buffer, stop_frame, 3U);
    timeout_context = test_context();
    timeout_context.partial_frame_active = 1U;
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 1U, "short header records truncation without reply");
    expect_u32(test_netconn_write_calls, 0U, "no response is sent until sequence is parseable");

    reset_fixture();
    memcpy(test_assembler.buffer, stop_frame, COMMAND_FRAME_HEADER_SIZE);
    test_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    timeout_context = test_context();
    timeout_context.client = NULL;
    timeout_context.partial_frame_active = 1U;
    expect_u32(command_task_finalize_incomplete_frame(&timeout_context, 1U), 0U, "failed truncation response closes connection");
    expect_u32(timeout_context.disconnect_requested, 1U, "failed truncation response marks disconnect");

    reset_fixture();
    CommandTaskAssemblerContext_t send_context = test_context();
    expect_u32(command_task_send_response(NULL, ping, ping_length), 0U, "missing session cannot send a response");
    send_context.client = NULL;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "null client cannot receive a response");
    send_context = test_context();
    expect_u32(command_task_send_response(&send_context, NULL, ping_length), 0U, "null response bytes are rejected");
    expect_u32(command_task_send_response(&send_context, ping, 0U), 0U, "empty response is rejected");
    expect_u32(
        command_task_send_response(&send_context, ping, COMMAND_MAX_FRAME_SIZE + 1U),
        0U,
        "oversized response is rejected"
    );
    expect_u32(
        command_task_send_protocol_response(&send_context, 0xFEU, 1U, 0U, NULL),
        0U,
        "unsupported response type is rejected"
    );

    reset_fixture();
    send_context = test_context();
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 1U, "response is written at once");
    expect_u32((uint32_t)test_write_apiflags, NETCONN_COPY | NETCONN_DONTBLOCK, "responses are written without blocking");
    expect_u32((uint32_t)send_context.tx_length, 0U, "fully written response leaves the send buffer empty");
    expect_u32((uint32_t)test_captured_response_length, (uint32_t)ping_length, "whole response reaches LwIP");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_PARTIAL_THEN_FULL;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 1U, "partial write keeps the client");
    expect_u32(test_netconn_write_calls, 1U, "partial write waits for the TX flag");
    expect_u32((uint32_t)send_context.tx_sent, 3U, "written bytes are tracked");
    expect_u32(command_task_flush_responses(&send_context), 1U, "TX flag finishes the partial response");
    expect_u32(test_netconn_write_calls, 2U, "rest of the response is written");
    expect_u32((uint32_t)send_context.tx_length, 0U, "send buffer is empty after the rest is written");
    expect_u32((uint32_t)test_captured_response_length, (uint32_t)ping_length, "whole response reaches LwIP");

    reset_fixture();
    send_context = test_context();
    test_write_blocked = 1U;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 1U, "full LwIP buffer keeps the client");
    expect_u32((uint32_t)send_context.tx_length, (uint32_t)ping_length, "blocked response stays buffered");
    expect_u32((uint32_t)test_captured_response_length, 0U, "blocked response is not written");
    test_write_blocked = 0U;
    expect_u32(command_task_flush_responses(&send_context), 1U, "buffered response is written later");
    expect_u32((uint32_t)test_captured_response_length, (uint32_t)ping_length, "buffered response reaches LwIP");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_OVERREPORT;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "overreported write is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_ARG, "overreport records argument error");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_PARTIAL_THEN_OVERREPORT;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 1U, "first partial write is accepted");
    expect_u32(command_task_flush_responses(&send_context), 0U, "overreport after partial progress is rejected");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_FAIL;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "failed write is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "failed write records its error");

    reset_fixture();
    send_context = test_context();
    test_write_result = ERR_BUF;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "transport write error is propagated");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failure before progress is counted");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_FAIL_AFTER_PARTIAL;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 1U, "partial write before failure keeps the client");
    expect_u32(command_task_flush_responses(&send_context), 0U, "failure after partial progress is rejected");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "failure after partial progress is counted");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_ERROR_WITH_PARTIAL_FIRST;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "write error with progress is rejected");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "write error with progress is counted");

    reset_fixture();
    send_context = test_context();
    test_write_mode = TEST_WRITE_ZERO_PROGRESS;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "zero progress without error is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_BUF, "zero progress maps to buffer error");

    reset_fixture();
    send_context = test_context();
    send_context.tx_length = COMMAND_TASK_TX_BUFFER_SIZE - 1U;
    test_write_blocked = 1U;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 0U, "response that does not fit is rejected");
    expect_u32((uint32_t)command_task_diagnostics.last_error, (uint32_t)ERR_MEM, "send buffer overflow records ERR_MEM");
    expect_u32(test_netconn_write_calls, 0U, "overflowing response is not written");

    reset_fixture();
    send_context = test_context();
    send_context.tx_sent = COMMAND_TASK_TX_BUFFER_SIZE - 10U;
    send_context.tx_length = COMMAND_TASK_TX_BUFFER_SIZE - 4U;
    test_write_blocked = 1U;
    expect_u32(command_task_send_response(&send_context, ping, ping_length), 1U, "response fits after compaction");
    expect_u32((uint32_t)send_context.tx_sent, 0U, "unsent bytes move to the buffer start");
    expect_u32((uint32_t)send_context.tx_length, 6U + (uint32_t)ping_length, "new response follows the unsent bytes");
}

static void test_nonblocking_responses_pause_reading(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    static uint8_t pings[20][COMMAND_MAX_FRAME_SIZE];
    const uint32_t ping_count = 20U;
    const size_t pong_length = 14U;

    reset_fixture();
    for (uint32_t index = 0U; index < ping_count; ++index) {
        const size_t ping_length = build_ping(pings[index], 200U + index);
        set_buffer(index, pings[index], (u16_t)ping_length, NULL, 0U, NULL, 0U);
        set_receive_result(index, ERR_OK);
    }
    test_write_blocked = 1U;
    set_flags_wait(0U, COMMAND_TASK_FLAG_TX);
    set_flags_wait(1U, COMMAND_TASK_FLAG_TX);
    test_flags_waits[1].unblock_writes = 1U;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "client is served while LwIP cannot take responses");
    expect_u32(test_thread_flags_wait_calls, 2U, "reading stays paused while responses cannot drain");
    expect_u32(test_netconn_recv_calls, ping_count + 1U, "reading resumes after the TX flag without a new RX flag");
    expect_u32(
        (uint32_t)test_captured_response_length,
        ping_count * (uint32_t)pong_length,
        "every PING gets its PONG"
    );
    CommandFrameHeaderDTO_t header = {0};
    (void)command_protocol_validate_frame(
        &test_captured_response[(ping_count - 1U) * pong_length],
        pong_length,
        &header
    );
    expect_u32(header.sequence, 200U + ping_count - 1U, "PONGs keep the request order");
}

static void test_incomplete_frame_tracking_edges(void) {
    reset_fixture();
    expect_u32(command_task_assembler_has_incomplete_command(NULL), 0U, "null assembler is not incomplete");
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 0U, "empty assembler is not incomplete");

    test_assembler.length = 1U;
    test_assembler.buffer[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 1U, "magic prefix is incomplete");
    test_assembler.buffer[0] ^= 1U;
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 0U, "unrelated byte is not incomplete");

    test_assembler.length = 2U;
    test_assembler.buffer[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    test_assembler.buffer[1] = (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U);
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 1U, "valid magic prefix is incomplete");
    test_assembler.buffer[1] ^= 1U;
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 0U, "bad second magic byte is discarded");

    uint8_t ping[COMMAND_MAX_FRAME_SIZE];
    const size_t ping_length = build_ping(ping, 86U);
    test_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    memcpy(test_assembler.buffer, ping, COMMAND_FRAME_HEADER_SIZE);
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 1U, "valid short header is incomplete");
    test_assembler.length = ping_length;
    memcpy(test_assembler.buffer, ping, ping_length);
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 0U, "complete frame is not incomplete");
    test_assembler.length = COMMAND_FRAME_HEADER_SIZE;
    test_assembler.buffer[4] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU);
    test_assembler.buffer[5] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U);
    expect_u32(command_task_assembler_has_incomplete_command(&test_assembler), 0U, "invalid header length is not incomplete");

    CommandTaskAssemblerContext_t context = test_context();
    context.partial_frame_active = 1U;
    command_frame_assembler_init(&test_assembler);
    command_task_update_partial_frame_tracking(&context);
    expect_u32(context.partial_frame_active, 0U, "tracking clears when no partial command remains");
    command_task_update_partial_frame_tracking(NULL);

    test_assembler.length = 1U;
    test_assembler.buffer[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    context.partial_frame_active = 0U;
    test_tick_count = 87U;
    command_task_update_partial_frame_tracking(&context);
    expect_u32(context.partial_frame_active, 1U, "tracking starts for a magic prefix");
    expect_u32(context.partial_frame_started_at, 87U, "tracking records the first partial tick");
}

static void test_assembly_pbuf_segment_and_trailing_cases(void) {
    struct pbuf segments[TEST_PBUF_SEGMENTS];

    reset_fixture();
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 88U);
    struct pbuf *buffer = test_chain(segments, frame, 3U, &frame[3], (u16_t)(frame_length - 3U), NULL, 0U);
    CommandTaskAssemblerContext_t context = test_context();
    CommandTaskAssemblyEvent_t event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_COMMAND_READY, "frame split across pbuf segments is assembled");
    expect_u32(event.command.header.sequence, 88U, "segmented pbuf returns the full request");

    reset_fixture();
    command_task_diagnostics.buffer_max_bytes = 100U;
    const uint8_t partial_byte[] = {0x5AU};
    buffer = test_chain(segments, partial_byte, sizeof(partial_byte), NULL, 0U, NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_NEED_MORE, "partial byte stays in assembler");
    expect_u32(command_task_diagnostics.buffer_max_bytes, 100U, "diagnostic high-water mark does not decrease");

    reset_fixture();
    uint8_t with_trailing_data[COMMAND_MAX_FRAME_SIZE + 20U] = {0};
    (void)memcpy(with_trailing_data, frame, frame_length);
    buffer = test_chain(segments, with_trailing_data, (u16_t)sizeof(with_trailing_data), NULL, 0U, NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "unappended bytes in a large segment are rejected");

    reset_fixture();
    uint8_t second_frame[COMMAND_MAX_FRAME_SIZE];
    const size_t second_length = build_ping(second_frame, 89U);
    uint8_t combined[COMMAND_MAX_FRAME_SIZE * 2U];
    memcpy(combined, frame, frame_length);
    memcpy(&combined[frame_length], second_frame, second_length);
    buffer = test_chain(segments, combined, (u16_t)(frame_length + second_length), NULL, 0U, NULL, 0U);
    context = test_context();
    event = command_task_assemble_pbuf(buffer, &context);
    expect_u32((uint32_t)event.result, COMMAND_TASK_ASSEMBLY_ERROR, "second complete frame in one receive is rejected");
    expect_u32((uint32_t)test_assembler.length, (uint32_t)second_length, "assembler retains the unexpected second frame");
}

static void test_diagnostics_and_internal_edges(void) {
    command_task_get_diagnostics(NULL);
    CommandTaskDiagnostics output = {0};
    command_task_diagnostics = (CommandTaskDiagnostics){
        .connections_accepted = 1U,
        .connections_closed = 2U,
        .active_connection = 3U,
        .complete_frames_received = 5U,
        .invalid_frames = 6U,
        .buffer_max_bytes = 10U,
        .response_send_errors = 14U,
        .recv_errors = 20U,
        .decoder_errors = 21U,
        .acks_deferred = 23U,
        .last_error = -22,
    };
    command_task_get_diagnostics(&output);
    expect_u32(output.connections_accepted, 1U, "diagnostics copy accepted connections");
    expect_u32(output.connections_closed, 2U, "diagnostics copy closed connections");
    expect_u32(output.active_connection, 3U, "diagnostics copy active flag");
    expect_u32(output.complete_frames_received, 5U, "diagnostics copy complete frames");
    expect_u32(output.invalid_frames, 6U, "diagnostics copy invalid frames");
    expect_u32(output.buffer_max_bytes, 10U, "diagnostics copy maximum buffer size");
    expect_u32(output.response_send_errors, 14U, "diagnostics copy response errors");
    expect_u32(output.recv_errors, 20U, "diagnostics copy receive errors");
    expect_u32(output.decoder_errors, 21U, "diagnostics copy decoder errors");
    expect_u32(output.acks_deferred, 23U, "diagnostics copy held ACKs");
    expect_u32((uint32_t)output.last_error, (uint32_t)-22, "diagnostics copy last error");

    volatile uint32_t counter = UINT32_MAX;
    command_counter_increment(&counter);
    expect_u32(counter, UINT32_MAX, "diagnostic counter saturates at maximum");
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

/* Checks one response frame in `data`: valid CRC, type, sequence, and for ERROR and DONE the 16-bit code, which is
 * little endian. An ERROR payload has three more bytes, which here must be those of a refusal: 00 FF 00. */
static void expect_frame_in(
    const uint8_t *data,
    size_t data_length,
    size_t offset,
    uint8_t type,
    uint32_t sequence,
    uint16_t error_code,
    const char *message
) {
    const uint8_t has_code = type == COMMAND_MESSAGE_TYPE_ERROR || type == COMMAND_MESSAGE_TYPE_DONE ? 1U : 0U;
    const uint32_t payload_length =
        type == COMMAND_MESSAGE_TYPE_ERROR ? 5U : (has_code != 0U ? 2U : 0U);
    const size_t length = COMMAND_FRAME_HEADER_SIZE + payload_length + COMMAND_FRAME_CRC_SIZE;
    const uint8_t present = data_length >= offset + length ? 1U : 0U;
    expect_true(present, message);
    if (present == 0U) {
        return;
    }
    CommandFrameHeaderDTO_t header = {0};
    expect_u32(command_protocol_validate_frame(&data[offset], length, &header), COMMAND_FRAME_VALID, message);
    expect_u32(header.type, type, message);
    expect_u32(header.sequence, sequence, message);
    expect_u32(header.payload_length, payload_length, message);
    if (has_code != 0U) {
        const uint16_t actual_error =
            (uint16_t)data[offset + COMMAND_FRAME_HEADER_SIZE] |
            (uint16_t)((uint16_t)data[offset + COMMAND_FRAME_HEADER_SIZE + 1U] << 8U);
        expect_u32(actual_error, error_code, message);
    }
    if (type == COMMAND_MESSAGE_TYPE_ERROR) {
        expect_u32(data[offset + COMMAND_FRAME_HEADER_SIZE + 2U], 0x00U, message);
        expect_u32(data[offset + COMMAND_FRAME_HEADER_SIZE + 3U], 0xFFU, message);
        expect_u32(data[offset + COMMAND_FRAME_HEADER_SIZE + 4U], 0x00U, message);
    }
}

static void expect_frame_at(size_t offset, uint8_t type, uint32_t sequence, uint16_t error_code, const char *message) {
    expect_frame_in(test_captured_response, test_captured_response_length, offset, type, sequence, error_code, message);
}

/* A result as command_result_queue_post makes it: cause 0, no axis, detail 0. */
static void script_result(uint32_t index, uint32_t sequence, uint8_t response_type, uint16_t code) {
    test_result_script[index] = (CommandResult_t){
        .sequence = sequence,
        .response_type = response_type,
        .code = code,
        .axis = COMMAND_ERROR_AXIS_NONE,
    };
    if (test_result_script_count <= index) {
        test_result_script_count = index + 1U;
    }
}

static void script_detailed_result(
    uint32_t index,
    uint32_t sequence,
    uint8_t response_type,
    uint16_t code,
    uint8_t cause,
    uint8_t axis,
    uint8_t detail
) {
    script_result(index, sequence, response_type, code);
    test_result_script[index].cause = cause;
    test_result_script[index].axis = axis;
    test_result_script[index].detail = detail;
}

/* A DONE result goes to the client as a DONE frame with the sequence of its command and its result code. */
static void test_result_done_frame(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    script_result(0U, 500U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    test_result_visible_from_wait = 1U;
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_RESULT);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "a result is sent to the connected client");
    expect_u32((uint32_t)test_captured_response_length, 16U, "DONE is a header, a two-byte result code and a CRC");
    expect_frame_at(0U, COMMAND_MESSAGE_TYPE_DONE, 500U, 0U, "an accepted result is sent as DONE with code 0");
    expect_u32(test_captured_response[COMMAND_FRAME_HEADER_SIZE], 0x00U, "the ACCEPTED payload is 00 00 (low byte)");
    expect_u32(test_captured_response[COMMAND_FRAME_HEADER_SIZE + 1U], 0x00U, "the ACCEPTED payload is 00 00 (high byte)");
    expect_u32(test_netconn_write_calls, 1U, "the result is written once");
    expect_u32(test_result_receive_calls, 3U, "one empty read after the socket, then the result and an empty read");
    expect_u32(test_thread_flags_wait_calls, 2U, "the RESULT flag alone does not read the socket");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "a sent result is not an error");
}

/* Several results wake the task once; each is answered in queue order with the frame its response type selects, not
 * its code: a DONE with a nonzero code stays DONE. */
static void test_result_error_frames_and_order(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    script_result(0U, 510U, COMMAND_MESSAGE_TYPE_ERROR, 0x0121U);
    script_result(1U, 511U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(2U, 512U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
    script_result(3U, 513U, COMMAND_MESSAGE_TYPE_ERROR, 0x0126U);
    test_result_visible_from_wait = 1U;
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_RESULT);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "several results are sent in one wake-up");
    expect_u32((uint32_t)test_captured_response_length, 19U + 16U + 16U + 19U, "two 19-byte ERROR and two 16-byte DONE frames");
    expect_frame_at(0U, COMMAND_MESSAGE_TYPE_ERROR, 510U, 0x0121U, "a refused command gets ERROR with its code");
    expect_frame_at(19U, COMMAND_MESSAGE_TYPE_DONE, 511U, 0U, "an accepted command gets DONE with code 0");
    expect_frame_at(35U, COMMAND_MESSAGE_TYPE_DONE, 512U, 1U, "a move to the current point gets DONE with code 1");
    expect_u32(test_captured_response[35U + COMMAND_FRAME_HEADER_SIZE], 0x01U, "ALREADY_AT_TARGET payload is 01 00 (low)");
    expect_u32(test_captured_response[35U + COMMAND_FRAME_HEADER_SIZE + 1U], 0x00U, "ALREADY_AT_TARGET payload is 01 00 (high)");
    expect_frame_at(51U, COMMAND_MESSAGE_TYPE_ERROR, 513U, 0x0126U, "the last refusal gets its own code");
    expect_u32(test_result_receive_calls, 6U, "one empty read after the socket, four results and one empty read");
}

/* Checks one ERROR frame at `offset` of the captured response, down to the three bytes after the code. */
static void expect_error_details_at(
    size_t offset,
    uint32_t sequence,
    uint16_t error_code,
    uint8_t cause,
    uint8_t axis,
    uint8_t detail,
    const char *message
) {
    const size_t length = COMMAND_FRAME_HEADER_SIZE + 5U + COMMAND_FRAME_CRC_SIZE;
    const uint8_t present = test_captured_response_length >= offset + length ? 1U : 0U;
    expect_true(present, message);
    if (present == 0U) {
        return;
    }
    CommandFrameHeaderDTO_t header = {0};
    expect_u32(
        command_protocol_validate_frame(&test_captured_response[offset], length, &header),
        COMMAND_FRAME_VALID,
        message
    );
    expect_u32(header.type, 0xFFU, message);
    expect_u32(header.sequence, sequence, message);
    expect_u32(header.payload_length, 5U, message);
    expect_u32(test_captured_response[offset + COMMAND_FRAME_HEADER_SIZE], (uint32_t)(error_code & 0xFFU), message);
    expect_u32(test_captured_response[offset + COMMAND_FRAME_HEADER_SIZE + 1U], (uint32_t)(error_code >> 8U), message);
    expect_u32(test_captured_response[offset + COMMAND_FRAME_HEADER_SIZE + 2U], cause, message);
    expect_u32(test_captured_response[offset + COMMAND_FRAME_HEADER_SIZE + 3U], axis, message);
    expect_u32(test_captured_response[offset + COMMAND_FRAME_HEADER_SIZE + 4U], detail, message);
}

/* The final results of a move: DONE COMPLETED, and ERROR STREAM_INTERRUPTED with the cause, axis and detail of the
 * stop, which are the three bytes after the code. */
static void test_result_stream_interrupted_and_completed_frames(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    /* A limit switch (cause 2) of Z (axis 2), the end switch (detail 3). */
    script_detailed_result(0U, 520U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 2U, 3U);
    /* A group fault (cause 3) of X: axis 0 is X and must not be sent as "no axis". */
    script_detailed_result(1U, 521U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 3U, 0U, 0x7EU);
    /* A stop that no axis is to blame for. */
    script_detailed_result(2U, 522U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 4U, 0xFFU, 9U);
    script_result(3U, 523U, COMMAND_MESSAGE_TYPE_DONE, 0x0002U);
    /* The stop fields of a DONE result are not part of its frame. */
    script_detailed_result(4U, 524U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED, 2U, 2U, 3U);
    test_result_visible_from_wait = 1U;
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_RESULT);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "the final results of moves are sent");

    expect_u32((uint32_t)test_captured_response_length, 3U * 19U + 2U * 16U, "three 19-byte ERROR and two 16-byte DONE frames");
    expect_error_details_at(0U, 520U, 0x0128U, 2U, 2U, 3U, "the limit stop carries cause, axis and detail");
    expect_error_details_at(19U, 521U, 0x0128U, 3U, 0U, 0x7EU, "axis X is sent as 0");
    expect_error_details_at(38U, 522U, 0x0128U, 4U, 0xFFU, 9U, "no blamed axis is sent as 0xFF");
    expect_frame_at(57U, COMMAND_MESSAGE_TYPE_DONE, 523U, 2U, "the completed move gets DONE with code 2");
    expect_u32(test_captured_response[57U + COMMAND_FRAME_HEADER_SIZE], 0x02U, "COMPLETED payload is 02 00 (low)");
    expect_u32(test_captured_response[57U + COMMAND_FRAME_HEADER_SIZE + 1U], 0x00U, "COMPLETED payload is 02 00 (high)");
    expect_frame_at(73U, COMMAND_MESSAGE_TYPE_DONE, 524U, 0U, "a DONE result keeps its two-byte payload");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "nothing fails");
}

/* A result stays queued while fewer than 38 bytes (two 19-byte ERROR frames) are free and is sent once the TX flag
 * drained the buffer. */
static void test_result_waits_for_send_buffer_space(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    static uint8_t pings[16][COMMAND_MAX_FRAME_SIZE];
    const uint32_t ping_count = 16U;
    const size_t pong_length = 14U;

    reset_fixture();
    /* 16 PONGs fill 224 of 256 bytes while LwIP takes nothing; reading stops with 32 bytes free, below the 38 kept. */
    for (uint32_t index = 0U; index < ping_count; ++index) {
        const size_t ping_length = build_ping(pings[index], 200U + index);
        set_buffer(index, pings[index], (u16_t)ping_length, NULL, 0U, NULL, 0U);
        set_receive_result(index, ERR_OK);
    }
    script_result(0U, 600U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(1U, 601U, COMMAND_MESSAGE_TYPE_ERROR, 0x0122U);
    script_result(2U, 602U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    test_write_blocked = 1U;
    set_flags_wait(0U, COMMAND_TASK_FLAG_RESULT);
    set_flags_wait(1U, COMMAND_TASK_FLAG_TX);
    test_flags_waits[1].unblock_writes = 1U;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "client is served while its send buffer is full");

    expect_u32(test_thread_flags_wait_calls, 2U, "the task waited for the RESULT and the TX flags");
    expect_u32(test_results_taken_at_wait[0], 0U, "no result is taken before the RESULT flag");
    expect_u32(test_results_taken_at_wait[1], 0U, "32 free bytes are below the 38 kept, so no result is taken");
    expect_u32(test_result_script_position, 3U, "the TX flag brought the waiting results");
    expect_u32(test_result_receive_calls, 4U, "three results and one empty read");
    expect_u32(
        (uint32_t)test_captured_response_length,
        ping_count * (uint32_t)pong_length + 16U + 19U + 16U,
        "the PONGs, two 16-byte DONE and one 19-byte ERROR reach the client"
    );
    for (uint32_t index = 0U; index < ping_count; ++index) {
        expect_frame_at(index * pong_length, COMMAND_MESSAGE_TYPE_PONG, 200U + index, 0U, "PONGs keep their order");
    }
    expect_frame_at(224U, COMMAND_MESSAGE_TYPE_DONE, 600U, 0U, "the first result follows the PONGs");
    expect_frame_at(240U, COMMAND_MESSAGE_TYPE_ERROR, 601U, 0x0122U, "the second result follows the first");
    expect_frame_at(259U, COMMAND_MESSAGE_TYPE_DONE, 602U, 0U, "the third result is last");
}

/* The send buffer must keep 38 bytes free (a held ACK plus the reply to a new frame, as the largest response is the
 * 19-byte ERROR) before a result is taken. */
static void test_result_needs_room_for_the_largest_frame(void) {
    CommandTaskAssemblerContext_t context;

    reset_fixture();
    script_result(0U, 700U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(1U, 701U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    context = test_context();
    test_write_blocked = 1U;
    context.tx_length = 219U;
    expect_u32(command_task_send_results(&context), 1U, "a nearly full send buffer keeps the client");
    expect_u32(test_result_receive_calls, 0U, "no result is taken with 37 bytes free");
    expect_u32(test_result_script_position, 0U, "the results stay queued");
    expect_u32(test_netconn_write_calls, 0U, "nothing is written without a result");

    context.tx_length = 218U;
    expect_u32(command_task_send_results(&context), 1U, "38 free bytes take one result");
    expect_u32(test_result_receive_calls, 1U, "exactly one result is taken with 38 free bytes");
    expect_u32(test_result_script_position, 1U, "the second result stays queued behind it");
    expect_u32((uint32_t)context.tx_length, 234U, "the 16-byte DONE frame is appended after the 218 buffered bytes");
    expect_frame_in(
        context.tx_buffer,
        context.tx_length,
        218U,
        COMMAND_MESSAGE_TYPE_DONE,
        700U,
        0U,
        "the buffered frame is the DONE of the first result"
    );
}

/* A result that cannot be sent closes the connection. */
static void test_result_send_failure_closes_the_client(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    script_result(0U, 710U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(1U, 711U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    test_result_visible_from_wait = 1U;
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_RESULT);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "a failed result write ends the client session");
    expect_u32(test_netconn_write_calls, 1U, "the result write is attempted once");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "the failed result write is counted");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "the client is closed after the failed result");
    expect_u32(test_netconn_recv_calls, 1U, "the socket is not read after the failed result");
    expect_u32(test_thread_flags_wait_calls, 1U, "the task does not wait again for a closed client");
    expect_u32(test_result_receive_calls, 2U, "after the empty read only the failing result is taken, not the next one");

    reset_fixture();
    script_result(0U, 712U, COMMAND_MESSAGE_TYPE_ERROR, 0x0124U);
    CommandTaskAssemblerContext_t send_context = test_context();
    test_write_mode = TEST_WRITE_FAIL;
    expect_u32(command_task_send_results(&send_context), 0U, "a failed ERROR write reports the closing of the client");
    expect_u32(send_context.disconnect_requested, 1U, "the failed result marks the disconnect");
}

/* A new client must not receive what was decided for the commands of the previous one. */
static void test_new_client_does_not_get_old_results(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    test_result_discard_erases = 1U;
    script_result(0U, 800U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(1U, 801U, COMMAND_MESSAGE_TYPE_ERROR, 0x0125U);
    set_receive_result(0U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_RESULT);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "the client is served after the old results are dropped");
    expect_u32(test_result_discard_calls, 1U, "accepting a client drops the queued results");
    expect_u32((uint32_t)test_captured_response_length, 0U, "no old result reaches the new client");
    expect_u32(test_netconn_write_calls, 0U, "nothing is written for old results");

    reset_fixture();
    test_result_discard_erases = 1U;
    script_result(0U, 802U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    test_jump_after_accept_calls = 2U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "two clients are served in a row");
    expect_u32(test_result_discard_calls, 2U, "every accepted client drops the queued results");
    expect_u32(command_task_diagnostics.connections_accepted, 2U, "both clients were accepted");
}

/* A result already in the queue goes out right after the incoming frame is read, with no RESULT flag. */
static void test_result_is_sent_after_reading_input(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t ping[COMMAND_MAX_FRAME_SIZE];
    const size_t ping_length = build_ping(ping, 300U);

    reset_fixture();
    script_result(0U, 310U, COMMAND_MESSAGE_TYPE_ERROR, 0x0121U);
    set_buffer(0U, ping, (u16_t)ping_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "the client is served with a result already queued");
    expect_u32(test_results_taken_at_wait[0], 1U, "the result was taken before the first wait for flags");
    expect_u32((uint32_t)test_captured_response_length, 14U + 19U, "the PONG and the 19-byte ERROR reach the client");
    expect_frame_at(0U, COMMAND_MESSAGE_TYPE_PONG, 300U, 0U, "the reply to the frame comes first");
    expect_frame_at(14U, COMMAND_MESSAGE_TYPE_ERROR, 310U, 0x0121U, "the queued result follows the reply");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "sending after reading is not an error");

    /* A send failure at this point closes the connection before the task waits for anything. */
    reset_fixture();
    script_result(0U, 311U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(1U, 312U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    set_receive_result(0U, ERR_WOULDBLOCK);
    test_write_mode = TEST_WRITE_FAIL;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "a failed result write after reading ends the session");
    expect_u32(test_netconn_write_calls, 1U, "the result write is attempted once");
    expect_u32(test_result_receive_calls, 1U, "the next result is not taken after the failure");
    expect_u32(command_task_diagnostics.response_send_errors, 1U, "the failed result write is counted");
    expect_u32(command_task_diagnostics.connections_closed, 1U, "the client is closed after the failed result");
    expect_u32(test_thread_flags_wait_calls, 0U, "the task does not wait again for a closed client");
}

/* While an ACK is held, a result is not sent after reading; QUEUE_SPACE sends the ACK first and the result after it. */
static void test_held_ack_goes_before_results_after_reading(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    uint8_t stop[COMMAND_MAX_FRAME_SIZE];
    const size_t stop_length = build_stop_command(stop, 101U);

    reset_fixture();
    test_queue_free_slots[0] = 0U;
    script_result(0U, 320U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    set_buffer(0U, stop, (u16_t)stop_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    set_receive_result(1U, ERR_WOULDBLOCK);
    set_flags_wait(0U, COMMAND_TASK_FLAG_QUEUE_SPACE | COMMAND_TASK_FLAG_RESULT);
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "the client is served with a held ACK and a result");
    expect_u32(command_task_diagnostics.acks_deferred, 1U, "the ACK of the command that filled the queue is held");
    expect_u32(test_results_taken_at_wait[0], 0U, "no result is taken after reading while the ACK is held");
    expect_u32((uint32_t)test_captured_response_length, 14U + 16U, "the 14-byte ACK and the 16-byte result reach the client");
    expect_ack_at(0U, 101U, "the held ACK is sent before the result");
    expect_frame_at(14U, COMMAND_MESSAGE_TYPE_DONE, 320U, 0U, "the result follows the ACK");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "nothing fails");
}

/* A held ACK must still fit into an almost full send buffer: results leave 38 bytes free and go after the ACK. */
static void test_results_do_not_starve_held_ack(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    static uint8_t pings[16][COMMAND_MAX_FRAME_SIZE];
    const uint32_t ping_count = 16U;
    const size_t pong_length = 14U;
    uint8_t stop[COMMAND_MAX_FRAME_SIZE];
    const size_t stop_length = build_stop_command(stop, 101U);

    reset_fixture();
    /* The STOP fills the command queue and its ACK is held. 16 PONGs then occupy 224 of 256 bytes. */
    test_queue_free_slots[0] = 0U;
    set_buffer(0U, stop, (u16_t)stop_length, NULL, 0U, NULL, 0U);
    set_receive_result(0U, ERR_OK);
    for (uint32_t index = 0U; index < ping_count; ++index) {
        const size_t ping_length = build_ping(pings[index], 200U + index);
        set_buffer(1U + index, pings[index], (u16_t)ping_length, NULL, 0U, NULL, 0U);
        set_receive_result(1U + index, ERR_OK);
    }
    script_result(0U, 600U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    script_result(1U, 601U, COMMAND_MESSAGE_TYPE_ERROR, 0x0122U);
    script_result(2U, 602U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    test_write_blocked = 1U;
    set_flags_wait(0U, COMMAND_TASK_FLAG_QUEUE_SPACE | COMMAND_TASK_FLAG_RESULT);
    set_flags_wait(1U, COMMAND_TASK_FLAG_TX);
    test_flags_waits[1].unblock_writes = 1U;
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "the client is served with a held ACK and a full buffer");

    expect_u32(command_task_diagnostics.acks_deferred, 1U, "the ACK is held");
    expect_u32(test_thread_flags_wait_calls, 2U, "the task waited for QUEUE_SPACE with RESULT and then for TX");
    expect_u32(test_results_taken_at_wait[0], 0U, "no result is taken while the ACK is held");
    expect_u32(test_results_taken_at_wait[1], 0U, "the ACK took the room: 18 bytes are left, below the 38 kept, so no result is taken");
    expect_u32(command_task_diagnostics.response_send_errors, 0U, "the held ACK is not refused for lack of room");
    expect_u32(test_result_script_position, 3U, "the TX flag brought all three results");
    expect_u32(
        (uint32_t)test_captured_response_length,
        ping_count * (uint32_t)pong_length + 14U + 16U + 19U + 16U,
        "the PONGs, the 14-byte ACK, two 16-byte DONE and one 19-byte ERROR reach the client"
    );
    for (uint32_t index = 0U; index < ping_count; ++index) {
        expect_frame_at(index * pong_length, COMMAND_MESSAGE_TYPE_PONG, 200U + index, 0U, "PONGs keep their order");
    }
    expect_ack_at(224U, 101U, "the held ACK follows the PONGs");
    expect_frame_at(238U, COMMAND_MESSAGE_TYPE_DONE, 600U, 0U, "the first result goes after the ACK");
    expect_frame_at(254U, COMMAND_MESSAGE_TYPE_ERROR, 601U, 0x0122U, "the second result follows the first");
    expect_frame_at(273U, COMMAND_MESSAGE_TYPE_DONE, 602U, 0U, "the third result is last");
}

/* Nothing of a client outlives its connection: after the socket is closed and deleted the motion is stopped and a STOP
 * with the internal sequence follows it into the command queue. */
static void test_disconnect_stops_motion_and_queues_an_internal_stop(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};

    reset_fixture();
    test_jump_after_accept_calls = 1U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "one client connects and leaves");
    expect_u32(test_motion_stop_requests, 1U, "the disconnect stops motion exactly once");
    expect_u32(test_command_queue_put_calls, 1U, "the disconnect queues exactly one command");
    expect_u32(test_put_commands[0].code, COMMAND_CODE_STOP, "the queued command is a STOP");
    expect_u32(test_put_commands[0].sequence, 0U, "the STOP has sequence 0: it gets no answer");
    expect_u32(test_stop_request_puts_before[0], 0U, "motion is stopped before the STOP is queued");
    expect_u32(test_put_stop_requests_before[0], 1U, "the STOP is queued after motion was stopped");
    expect_u32(test_stop_request_deletes_before[0], 1U, "motion is stopped after the client socket is deleted");
    expect_u32(test_netconn_close_calls, 1U, "the client socket is closed");
    expect_u32(test_netconn_delete_calls, 1U, "the client socket is deleted");

    reset_fixture();
    test_jump_after_accept_calls = 2U;
    expect_u32((uint32_t)run_task_until_jump(&context), 1U, "two clients connect one after another");
    expect_u32(test_motion_stop_requests, 2U, "each disconnect stops motion once");
    expect_u32(test_command_queue_put_calls, 2U, "each disconnect queues one command");
    for (uint32_t index = 0U; index < 2U; ++index) {
        expect_u32(test_put_commands[index].code, COMMAND_CODE_STOP, "each queued command is a STOP");
        expect_u32(test_put_commands[index].sequence, 0U, "each STOP has sequence 0");
        expect_u32(test_stop_request_puts_before[index], index, "each stop comes after the STOP of the earlier client");
        expect_u32(test_put_stop_requests_before[index], index + 1U, "each STOP is queued after its own stop of motion");
        expect_u32(test_stop_request_deletes_before[index], index + 1U, "each stop follows the deletion of its socket");
    }
}

/* A STOP that does not fit into the command queue, or fails, does not end the accept loop. */
static void test_disconnect_survives_a_refused_stop(void) {
    NetworkTaskContext context = {.lwip_flags = (osEventFlagsId_t)(uintptr_t)1U};
    const CommandQueuePutResult_t refusals[] = {COMMAND_QUEUE_PUT_FULL, COMMAND_QUEUE_PUT_ERROR};

    for (uint32_t index = 0U; index < sizeof(refusals) / sizeof(refusals[0]); ++index) {
        reset_fixture();
        test_command_queue_put_result = refusals[index];
        test_jump_after_accept_calls = 2U;
        expect_u32((uint32_t)run_task_until_jump(&context), 1U, "the accept loop goes on after a refused STOP");
        expect_u32(test_netconn_accept_calls, 3U, "the loop accepted both clients and waited for a third");
        expect_u32(command_task_diagnostics.connections_accepted, 2U, "both clients were accepted");
        expect_u32(command_task_diagnostics.connections_closed, 2U, "both clients were counted as closed");
        expect_u32(command_task_diagnostics.active_connection, 0U, "no connection stays active");
        expect_u32(test_motion_stop_requests, 2U, "motion was stopped for each disconnect");
        expect_u32(test_command_queue_put_calls, 2U, "a STOP was offered for each disconnect");
    }
}

static void test_result_listener_sets_its_flag(void) {
    reset_fixture();
    command_task_result_available();
    expect_u32(test_thread_flags_set_calls, 0U, "a result before the task id is known sets no flag");

    command_task_thread_id = TEST_THREAD_ID;
    command_task_result_available();
    expect_u32(test_thread_flags_set_value, COMMAND_TASK_FLAG_RESULT, "a new result sets the RESULT flag");
    expect_true(test_thread_flags_set_target == TEST_THREAD_ID, "the RESULT flag goes to the command task");
    expect_u32(COMMAND_TASK_FLAGS_ALL & COMMAND_TASK_FLAG_RESULT, COMMAND_TASK_FLAG_RESULT, "the RESULT flag is awaited");
}

int main(void) {
    test_result_done_frame();
    test_result_error_frames_and_order();
    test_result_stream_interrupted_and_completed_frames();
    test_result_waits_for_send_buffer_space();
    test_result_needs_room_for_the_largest_frame();
    test_result_send_failure_closes_the_client();
    test_new_client_does_not_get_old_results();
    test_result_is_sent_after_reading_input();
    test_held_ack_goes_before_results_after_reading();
    test_results_do_not_starve_held_ack();
    test_disconnect_stops_motion_and_queues_an_internal_stop();
    test_disconnect_survives_a_refused_stop();
    test_result_listener_sets_its_flag();
    test_context_and_listener_failures();
    test_ping_ack_and_error_responses();
    test_fragmented_receive_and_disconnect_cleanup();
    test_flag_waits_and_receive_errors();
    test_netconn_event_callback();
    test_deferred_ack();
    test_flags_wait_timeout_helper();
    test_assembler_errors_and_boundary_paths();
    test_incomplete_frame_tracking_edges();
    test_assembly_pbuf_segment_and_trailing_cases();
    test_frame_errors_timeout_helpers_and_response_writes();
    test_nonblocking_responses_pause_reading();
    test_diagnostics_and_internal_edges();
    test_decoder_failure_and_error_response_send_failure();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command task unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command task unit tests passed.\n");
    return 0;
}

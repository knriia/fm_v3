#include "command_decoder_task.h"
#include "command_protocol.h"
#include "FreeRTOS.h"
#include "queue.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    UBaseType_t item_size;
    UBaseType_t length;
    UBaseType_t count;
    UBaseType_t read_index;
    UBaseType_t write_index;
    uint8_t storage[16U][512U];
} TestQueue;

static TestQueue test_input_queue;
static TestQueue test_response_queue;
static uint32_t test_failures;
static uint8_t test_jump_active;
static uint8_t test_return_false_once;
static uint8_t test_fail_queue_create_call;
static uint8_t test_force_receive_failure;
static uint8_t test_force_validation_success;
static uint8_t test_force_error_response_build_failure;
static uint32_t test_queue_create_calls;
static TickType_t test_last_receive_timeout;
static jmp_buf test_jump_buffer;

static uint16_t command_decoder_test_validate_request_dto(const CommandRequestDTO_t *request);
static size_t command_decoder_test_build_error_response(
    uint8_t *frame,
    size_t frame_capacity,
    uint32_t sequence,
    uint16_t error_code
);

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

QueueHandle_t xQueueCreateStatic(
    UBaseType_t queue_length,
    UBaseType_t item_size,
    uint8_t *queue_storage,
    StaticQueue_t *queue_buffer
) {
    (void)queue_storage;
    (void)queue_buffer;
    ++test_queue_create_calls;
    TestQueue *queue = item_size == sizeof(CommandTaskRequestDTO_t) ? &test_input_queue : &test_response_queue;
    (void)memset(queue, 0, sizeof(*queue));
    queue->length = queue_length;
    queue->item_size = item_size;
    if (test_queue_create_calls == test_fail_queue_create_call) {
        return NULL;
    }
    return queue;
}

BaseType_t xQueueSend(QueueHandle_t handle, const void *item, TickType_t timeout) {
    (void)timeout;
    TestQueue *queue = handle;
    if (queue == NULL || item == NULL || queue->count >= queue->length ||
        queue->item_size > sizeof(queue->storage[0])) {
        return pdFALSE;
    }
    (void)memcpy(queue->storage[queue->write_index], item, queue->item_size);
    queue->write_index = (queue->write_index + 1U) % queue->length;
    ++queue->count;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t handle, void *item, TickType_t timeout) {
    test_last_receive_timeout = timeout;
    TestQueue *queue = handle;
    if (queue == NULL || item == NULL || test_force_receive_failure != 0U) {
        return pdFALSE;
    }
    if (queue->count == 0U) {
        if (test_return_false_once != 0U && timeout == portMAX_DELAY) {
            test_return_false_once = 0U;
            return pdFALSE;
        }
        if (test_jump_active != 0U && timeout == portMAX_DELAY) {
            longjmp(test_jump_buffer, 1);
        }
        return pdFALSE;
    }
    (void)memcpy(item, queue->storage[queue->read_index], queue->item_size);
    queue->read_index = (queue->read_index + 1U) % queue->length;
    --queue->count;
    return pdTRUE;
}

static void reset_fixture(void) {
    test_return_false_once = 0U;
    test_fail_queue_create_call = 0U;
    test_force_receive_failure = 0U;
    test_force_validation_success = 0U;
    test_force_error_response_build_failure = 0U;
    test_queue_create_calls = 0U;
    expect_true(command_decoder_initialize() != 0U, "decoder initialization succeeds");
    test_jump_active = 0U;
}

static void run_decoder_until_queues_are_empty(void) {
    test_jump_active = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        CommandDecoderTask(NULL);
    }
    test_jump_active = 0U;
    expect_u32((uint32_t)jumped, 1U, "decoder task stops only at empty input queue in test");
}

static size_t build_ping(uint8_t *frame, uint32_t sequence) {
    return command_protocol_build_frame(frame, COMMAND_MAX_FRAME_SIZE, COMMAND_MESSAGE_TYPE_PING, sequence, NULL, 0U);
}

static CommandTaskRequestDTO_t make_request(uint32_t connection_id, const uint8_t *frame, size_t frame_length) {
    CommandTaskRequestDTO_t request = {.connection_id = connection_id};
    expect_u32(
        command_protocol_decode_request(frame, frame_length, &request.request),
        COMMAND_FRAME_VALID,
        "test frame is decoded before entering decoder task"
    );
    return request;
}

static void test_ping_request(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 41U);
    const CommandTaskRequestDTO_t request = make_request(7U, frame, frame_length);

    reset_fixture();
    expect_u32(command_decoder_submit_request(&request), 1U, "complete request is queued");
    run_decoder_until_queues_are_empty();

    CommandTaskResponseDTO_t response = {0};
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "PONG response is available");
    expect_u32(response.connection_id, 7U, "response keeps connection id");
    expect_u32(response.response_type, COMMAND_MESSAGE_TYPE_PONG, "valid PING produces PONG");
    CommandFrameHeaderDTO_t header;
    expect_u32(
        command_protocol_validate_frame(response.frame, response.length, &header),
        COMMAND_FRAME_VALID,
        "PONG response is a valid frame"
    );
    expect_u32(header.sequence, 41U, "PONG keeps sequence");

    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.requests_received, 1U, "complete request counter");
    expect_u32(diagnostics.pings_received, 1U, "PING counter");
    expect_u32(diagnostics.responses_formed, 1U, "formed response counter");
    expect_u32(diagnostics.pongs_queued, 1U, "queued PONG counter");
    expect_u32(diagnostics.errors_queued, 0U, "no error response for valid PING");
    expect_u32(diagnostics.commands_received, 0U, "PING is not counted as a command");
    expect_u32(diagnostics.commands_rejected, 0U, "valid PING is not rejected");
    expect_u32(diagnostics.request_queue_overflows, 0U, "valid PING does not overflow the request queue");
    expect_u32(diagnostics.response_queue_overflows, 0U, "valid PING does not overflow the response queue");
    expect_u32((uint32_t)diagnostics.last_error, 0U, "valid PING leaves decoder error clear");
}

#define command_protocol_validate_request_dto command_decoder_test_validate_request_dto
#define command_protocol_build_error_response command_decoder_test_build_error_response
#define static
#include "../APP/tasks/command_decoder_task.c"
#undef static
#undef command_protocol_build_error_response
#undef command_protocol_validate_request_dto

static uint16_t command_decoder_test_validate_request_dto(const CommandRequestDTO_t *request) {
    if (test_force_validation_success != 0U) {
        return 0U;
    }
    return command_protocol_validate_request_dto(request);
}

static size_t command_decoder_test_build_error_response(
    uint8_t *frame,
    size_t frame_capacity,
    uint32_t sequence,
    uint16_t error_code
) {
    if (test_force_error_response_build_failure != 0U) {
        return 0U;
    }
    return command_protocol_build_error_response(frame, frame_capacity, sequence, error_code);
}

static void test_rejected_command(void) {
    const uint8_t payload[] = {0xFEU};
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length =
        command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_COMMAND, 42U, payload, sizeof(payload));
    const CommandTaskRequestDTO_t request = make_request(1U, frame, frame_length);

    reset_fixture();
    expect_u32(command_decoder_submit_request(&request), 1U, "command request is queued");
    run_decoder_until_queues_are_empty();

    CommandTaskResponseDTO_t response = {0};
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "ERROR response is available");
    expect_u32(response.response_type, COMMAND_MESSAGE_TYPE_ERROR, "unknown command produces ERROR");
    expect_u32(response.frame[COMMAND_FRAME_HEADER_SIZE], COMMAND_ERROR_UNKNOWN_COMMAND & 0xFFU, "ERROR code low byte");
    expect_u32(
        response.frame[COMMAND_FRAME_HEADER_SIZE + 1U],
        COMMAND_ERROR_UNKNOWN_COMMAND >> 8U,
        "ERROR code high byte"
    );

    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_received, 1U, "received command counter");
    expect_u32(diagnostics.commands_rejected, 1U, "rejected command counter");
    expect_u32(diagnostics.unknown_commands, 1U, "unknown command counter");
    expect_u32(diagnostics.errors_queued, 1U, "queued ERROR counter");
}

static void test_invalid_request_dto(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 43U);
    CommandTaskRequestDTO_t request = make_request(2U, frame, frame_length);
    request.request.header.version = COMMAND_PROTOCOL_VERSION + 1U;

    reset_fixture();
    expect_u32(command_decoder_submit_request(&request), 1U, "invalid semantic request is queued");
    run_decoder_until_queues_are_empty();

    CommandTaskResponseDTO_t response = {0};
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "invalid request produces ERROR");
    expect_u32(response.response_type, COMMAND_MESSAGE_TYPE_ERROR, "invalid request response type");
    expect_u32(response.frame[COMMAND_FRAME_HEADER_SIZE], COMMAND_ERROR_UNSUPPORTED_VERSION, "version error code");

    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.unsupported_version, 1U, "unsupported version counter");
    expect_u32(diagnostics.requests_received, 1U, "semantic invalid request is received");
}

static void test_invalid_payload_length_diagnostic(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 44U);
    CommandTaskRequestDTO_t request = make_request(3U, frame, frame_length);
    request.request.header.payload_length = 1U;

    reset_fixture();
    expect_u32(command_decoder_submit_request(&request), 1U, "invalid payload length request is queued");
    run_decoder_until_queues_are_empty();

    CommandTaskResponseDTO_t response = {0};
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "invalid payload length produces ERROR");
    expect_u32(response.response_type, COMMAND_MESSAGE_TYPE_ERROR, "invalid payload length response type");

    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.invalid_payload_length, 1U, "semantic payload length error is counted");
    expect_u32(diagnostics.responses_formed, 1U, "ERROR response is counted as formed");
}

static void test_connection_change(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 44U);
    const CommandTaskRequestDTO_t first_request = make_request(10U, frame, frame_length);
    const CommandTaskRequestDTO_t second_request = make_request(11U, frame, frame_length);

    reset_fixture();
    expect_u32(command_decoder_submit_request(&first_request), 1U, "first connection request is queued");
    expect_u32(command_decoder_submit_request(&second_request), 1U, "new connection request is queued");
    run_decoder_until_queues_are_empty();

    CommandTaskResponseDTO_t response = {0};
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "first connection response is available");
    expect_u32(response.connection_id, 10U, "first response belongs to first connection");
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "new connection response is available");
    expect_u32(response.connection_id, 11U, "second response belongs to new connection");

    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.requests_received, 2U, "both complete requests are decoded");
}

static void test_queue_overflows(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 45U);
    const CommandTaskRequestDTO_t request = make_request(1U, frame, frame_length);

    reset_fixture();
    for (uint32_t index = 0U; index < COMMAND_DECODER_REQUEST_QUEUE_LENGTH + 1U; ++index) {
        (void)command_decoder_submit_request(&request);
    }
    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.request_queue_overflows, 1U, "request queue overflow counter");
    expect_u32(diagnostics.request_bytes_dropped, (uint32_t)frame_length, "dropped request byte counter");

    reset_fixture();
    for (uint32_t index = 0U; index < COMMAND_DECODER_RESPONSE_QUEUE_LENGTH; ++index) {
        (void)command_decoder_submit_request(&request);
    }
    run_decoder_until_queues_are_empty();
    expect_u32(command_decoder_submit_request(&request), 1U, "request after input queue drain is queued");
    run_decoder_until_queues_are_empty();
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(
        diagnostics.requests_received,
        COMMAND_DECODER_RESPONSE_QUEUE_LENGTH + 1U,
        "all burst requests are decoded"
    );
    expect_u32(
        diagnostics.responses_formed,
        COMMAND_DECODER_RESPONSE_QUEUE_LENGTH + 1U,
        "responses are formed even when the output queue is full"
    );
    expect_u32(diagnostics.response_queue_overflows, 1U, "response queue overflow is reported");
    expect_u32(diagnostics.pongs_queued, COMMAND_DECODER_RESPONSE_QUEUE_LENGTH, "response queue capacity is respected");
}

static void test_invalid_errors_and_counter_edges(void) {
    volatile uint32_t counter = UINT32_MAX;
    command_decoder_counter_increment(&counter);
    expect_u32(counter, UINT32_MAX, "decoder counter increment saturates");
    counter = 0U;
    command_decoder_counter_increment(&counter);
    expect_u32(counter, 1U, "decoder counter increments below saturation");
    counter = UINT32_MAX - 1U;
    command_decoder_counter_add(&counter, 2U);
    expect_u32(counter, UINT32_MAX, "decoder counter addition saturates on overflow");
    counter = 4U;
    command_decoder_counter_add(&counter, 3U);
    expect_u32(counter, 7U, "decoder counter addition succeeds without overflow");
    if (SIZE_MAX > UINT32_MAX) {
        counter = 0U;
        command_decoder_counter_add(&counter, (size_t)UINT32_MAX + 1U);
        expect_u32(counter, UINT32_MAX, "decoder counter addition saturates for oversized size_t");
    }

    reset_fixture();
    command_decoder_record_request_error(UINT16_MAX);
    expect_u32((uint32_t)command_decoder_diagnostics.last_error, UINT16_MAX, "unknown request error is recorded");

    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 47U);
    CommandTaskRequestDTO_t request = make_request(5U, frame, frame_length);
    request.request.header.type = COMMAND_MESSAGE_TYPE_PONG;
    command_decoder_process_request(&request);
    request.request.header.type = COMMAND_MESSAGE_TYPE_COMMAND;
    request.request.header.sequence = 0U;
    request.request.header.payload_length = 1U;
    request.request.payload[0] = COMMAND_CODE_STOP;
    command_decoder_process_request(&request);
    request.request.header.sequence = 48U;
    command_decoder_process_request(&request);
    command_decoder_process_request(NULL);

    CommandDecoderTaskDiagnostics diagnostics;
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.unexpected_type, 1U, "unexpected request type is counted");
    expect_u32(diagnostics.invalid_sequence, 1U, "invalid request sequence is counted");
    expect_u32(diagnostics.invalid_states, 1U, "unsupported command state is counted");
    expect_u32(diagnostics.requests_received, 3U, "null request is ignored");
}

static void test_queue_and_public_api_failures(void) {
    reset_fixture();
    expect_u32(command_decoder_submit_request(NULL), 0U, "null request submission is rejected");
    expect_u32((uint32_t)command_decoder_diagnostics.last_error, COMMAND_FRAME_INVALID_ARGUMENT, "null submission records invalid argument");
    expect_u32(command_decoder_receive_response(NULL, 0U), 0U, "null response output is rejected");
    expect_u32(command_decoder_receive_response(&(CommandTaskResponseDTO_t){0}, 0U), 0U, "empty response queue returns false");
    expect_u32(command_decoder_receive_response(&(CommandTaskResponseDTO_t){0}, 25U), 0U, "timed response receive returns false on empty queue");
    expect_u32(test_last_receive_timeout, 25U, "response timeout is converted to ticks");
    command_decoder_get_diagnostics(NULL);
    command_decoder_get_response_queue_failure(NULL, NULL);
    uint32_t generation = UINT32_MAX;
    command_decoder_get_response_queue_failure(&generation, NULL);
    expect_u32(generation, 0U, "response queue failure generation is read independently");
    uint32_t connection_id = UINT32_MAX;
    command_decoder_get_response_queue_failure(NULL, &connection_id);
    expect_u32(connection_id, 0U, "response queue failure connection is read independently");

    test_force_receive_failure = 1U;
    expect_u32(command_decoder_receive_response(&(CommandTaskResponseDTO_t){0}, 0U), 0U, "queue receive failure is returned to caller");
    test_force_receive_failure = 0U;

    test_queue_create_calls = 0U;
    test_fail_queue_create_call = 1U;
    expect_u32(command_decoder_initialize(), 0U, "missing request queue fails initialization");
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 49U);
    const CommandTaskRequestDTO_t request = make_request(6U, frame, frame_length);
    expect_u32(command_decoder_submit_request(&request), 0U, "request cannot be queued after request queue creation failure");
    expect_u32(command_decoder_receive_response(&(CommandTaskResponseDTO_t){0}, 0U), 0U, "response queue remains empty after request queue failure");

    test_queue_create_calls = 0U;
    test_fail_queue_create_call = 2U;
    expect_u32(command_decoder_initialize(), 0U, "missing response queue fails initialization");
    expect_u32(command_decoder_receive_response(&(CommandTaskResponseDTO_t){0}, 0U), 0U, "missing response queue rejects receive");
    expect_u32(command_decoder_queue_response(8U, frame, frame_length, COMMAND_MESSAGE_TYPE_PONG), 0U, "missing response queue rejects response");
    uint32_t failure_connection_id = 0U;
    command_decoder_get_response_queue_failure(NULL, &failure_connection_id);
    expect_u32(failure_connection_id, 8U, "response queue failure records connection id");
    expect_u32(command_decoder_queue_request(NULL), 0U, "null internal request is rejected");

    reset_fixture();
    expect_u32(command_decoder_queue_response(1U, NULL, 1U, COMMAND_MESSAGE_TYPE_PONG), 0U, "null response frame is rejected");
    expect_u32(
        command_decoder_queue_response(2U, frame, COMMAND_MAX_FRAME_SIZE + 1U, COMMAND_MESSAGE_TYPE_PONG),
        0U,
        "oversized response frame is rejected"
    );
    expect_u32(command_decoder_queue_response(3U, frame, 0U, 0U), 1U, "zero-length unclassified response is queued");
}

static void test_response_build_fallbacks_and_task_receive_failure(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 50U);
    CommandTaskRequestDTO_t request = make_request(7U, frame, frame_length);

    reset_fixture();
    request.request.header.type = COMMAND_MESSAGE_TYPE_COMMAND;
    request.request.header.payload_length = 1U;
    request.request.payload[0] = COMMAND_CODE_STOP;
    test_force_validation_success = 1U;
    command_decoder_process_request(&request);
    CommandTaskResponseDTO_t response = {0};
    expect_u32(command_decoder_receive_response(&response, 0U), 1U, "unsupported valid command produces fallback response");
    expect_u32(response.frame[COMMAND_FRAME_HEADER_SIZE], COMMAND_ERROR_INVALID_STATE, "fallback response reports invalid command state");

    reset_fixture();
    test_force_error_response_build_failure = 1U;
    command_decoder_process_request(&request);
    expect_u32(
        (uint32_t)command_decoder_diagnostics.last_error,
        COMMAND_DECODER_ERROR_RESPONSE_BUILD_FAILED,
        "failed error response build is recorded"
    );

    reset_fixture();
    test_return_false_once = 1U;
    run_decoder_until_queues_are_empty();
    expect_u32(test_return_false_once, 0U, "decoder task continues after a transient input queue receive failure");
}

int main(void) {
    test_ping_request();
    test_rejected_command();
    test_invalid_request_dto();
    test_invalid_payload_length_diagnostic();
    test_connection_change();
    test_queue_overflows();
    test_invalid_errors_and_counter_edges();
    test_queue_and_public_api_failures();
    test_response_build_fallbacks_and_task_receive_failure();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command decoder task unit tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Command decoder task unit tests passed.\n");
    return 0;
}

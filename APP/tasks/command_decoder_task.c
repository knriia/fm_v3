#include "command_decoder_task.h"

#include "FreeRTOS.h"
#include "queue.h"

#include <stddef.h>
#include <string.h>

static volatile CommandDecoderTaskDiagnostics command_decoder_diagnostics;
static volatile uint32_t command_decoder_response_queue_failure_generation;
static volatile uint32_t command_decoder_response_queue_failure_connection_id;
static QueueHandle_t command_decoder_request_queue;
static QueueHandle_t command_decoder_response_queue;
static StaticQueue_t command_decoder_request_queue_control;
static StaticQueue_t command_decoder_response_queue_control;
static uint8_t
    command_decoder_request_queue_storage[COMMAND_DECODER_REQUEST_QUEUE_LENGTH * sizeof(CommandTaskRequestDTO_t)];
static uint8_t
    command_decoder_response_queue_storage[COMMAND_DECODER_RESPONSE_QUEUE_LENGTH * sizeof(CommandTaskResponseDTO_t)];

static void command_decoder_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void command_decoder_counter_add(volatile uint32_t *counter, size_t value) {
    if (value > UINT32_MAX || UINT32_MAX - *counter < (uint32_t)value) {
        *counter = UINT32_MAX;
    } else {
        *counter += (uint32_t)value;
    }
}

static void command_decoder_record_error(int32_t error) { command_decoder_diagnostics.last_error = error; }

static void command_decoder_record_response_queue_overflow(uint32_t connection_id) {
    command_decoder_response_queue_failure_connection_id = connection_id;
    ++command_decoder_response_queue_failure_generation;
    command_decoder_counter_increment(&command_decoder_diagnostics.response_queue_overflows);
    command_decoder_record_error(COMMAND_DECODER_ERROR_RESPONSE_QUEUE_FULL);
}

static uint8_t command_decoder_queue_response(
    uint32_t connection_id,
    const uint8_t *frame,
    size_t frame_length,
    uint8_t response_type
) {
    if (command_decoder_response_queue == NULL || frame == NULL || frame_length > COMMAND_MAX_FRAME_SIZE) {
        command_decoder_record_response_queue_overflow(connection_id);
        return 0U;
    }

    CommandTaskResponseDTO_t response = {
        .connection_id = connection_id,
        .length = (uint16_t)frame_length,
        .response_type = response_type,
    };
    memcpy(response.frame, frame, frame_length);
    if (xQueueSend(command_decoder_response_queue, &response, 0U) != pdTRUE) {
        command_decoder_record_response_queue_overflow(connection_id);
        return 0U;
    }

    if (response_type == COMMAND_MESSAGE_TYPE_PONG) {
        command_decoder_counter_increment(&command_decoder_diagnostics.pongs_queued);
    } else if (response_type == COMMAND_MESSAGE_TYPE_ERROR) {
        command_decoder_counter_increment(&command_decoder_diagnostics.errors_queued);
    }
    return 1U;
}

static void command_decoder_record_request_error(uint16_t error_code) {
    command_decoder_record_error((int32_t)error_code);
    switch (error_code) {
    case COMMAND_ERROR_UNSUPPORTED_VERSION:
        command_decoder_counter_increment(&command_decoder_diagnostics.unsupported_version);
        break;
    case COMMAND_ERROR_UNEXPECTED_TYPE:
        command_decoder_counter_increment(&command_decoder_diagnostics.unexpected_type);
        break;
    case COMMAND_ERROR_INVALID_LENGTH:
        command_decoder_counter_increment(&command_decoder_diagnostics.invalid_payload_length);
        break;
    case COMMAND_ERROR_INVALID_SEQUENCE:
        command_decoder_counter_increment(&command_decoder_diagnostics.invalid_sequence);
        break;
    case COMMAND_ERROR_UNKNOWN_COMMAND:
        command_decoder_counter_increment(&command_decoder_diagnostics.unknown_commands);
        break;
    case COMMAND_ERROR_INVALID_STATE:
        command_decoder_counter_increment(&command_decoder_diagnostics.invalid_states);
        break;
    default:
        break;
    }
}

static void command_decoder_process_request(const CommandTaskRequestDTO_t *request) {
    if (request == NULL) {
        return;
    }

    const CommandFrameHeaderDTO_t *header = &request->request.header;
    command_decoder_counter_increment(&command_decoder_diagnostics.requests_received);

    if (header->type == COMMAND_MESSAGE_TYPE_COMMAND) {
        command_decoder_counter_increment(&command_decoder_diagnostics.commands_received);
    } else if (header->type == COMMAND_MESSAGE_TYPE_PING) {
        command_decoder_counter_increment(&command_decoder_diagnostics.pings_received);
    }

    const uint16_t error_code = command_protocol_validate_request_dto(&request->request);
    if (header->type == COMMAND_MESSAGE_TYPE_COMMAND && error_code != 0U) {
        command_decoder_counter_increment(&command_decoder_diagnostics.commands_rejected);
    }
    if (error_code != 0U) {
        command_decoder_record_request_error(error_code);
    }

    uint8_t response[COMMAND_MAX_FRAME_SIZE];
    size_t response_length = 0U;
    uint8_t response_type = COMMAND_MESSAGE_TYPE_ERROR;
    if (error_code == 0U && header->type == COMMAND_MESSAGE_TYPE_PING) {
        response_type = COMMAND_MESSAGE_TYPE_PONG;
        response_length = command_protocol_build_pong_response(response, sizeof(response), header->sequence);
    } else {
        response_length = command_protocol_build_error_response(
            response,
            sizeof(response),
            header->sequence,
            error_code == 0U ? COMMAND_ERROR_INVALID_STATE : error_code
        );
    }

    if (response_length != 0U) {
        command_decoder_counter_increment(&command_decoder_diagnostics.responses_formed);
        (void)command_decoder_queue_response(request->connection_id, response, response_length, response_type);
    } else {
        command_decoder_record_error(COMMAND_DECODER_ERROR_RESPONSE_BUILD_FAILED);
    }
}

static uint8_t command_decoder_queue_request(const CommandTaskRequestDTO_t *request) {
    if (command_decoder_request_queue == NULL || request == NULL ||
        xQueueSend(command_decoder_request_queue, request, 0U) != pdTRUE) {
        command_decoder_counter_increment(&command_decoder_diagnostics.request_queue_overflows);
        command_decoder_record_error(COMMAND_DECODER_ERROR_REQUEST_QUEUE_FULL);
        return 0U;
    }
    return 1U;
}

uint8_t command_decoder_initialize(void) {
    command_decoder_response_queue_failure_generation = 0U;
    command_decoder_response_queue_failure_connection_id = 0U;
    command_decoder_diagnostics = (CommandDecoderTaskDiagnostics){0};
    command_decoder_request_queue = xQueueCreateStatic(
        COMMAND_DECODER_REQUEST_QUEUE_LENGTH,
        sizeof(CommandTaskRequestDTO_t),
        command_decoder_request_queue_storage,
        &command_decoder_request_queue_control
    );
    command_decoder_response_queue = xQueueCreateStatic(
        COMMAND_DECODER_RESPONSE_QUEUE_LENGTH,
        sizeof(CommandTaskResponseDTO_t),
        command_decoder_response_queue_storage,
        &command_decoder_response_queue_control
    );
    return command_decoder_request_queue != NULL && command_decoder_response_queue != NULL ? 1U : 0U;
}

uint8_t command_decoder_submit_request(const CommandTaskRequestDTO_t *request) {
    if (request == NULL) {
        command_decoder_record_error(COMMAND_FRAME_INVALID_ARGUMENT);
        return 0U;
    }

    if (command_decoder_queue_request(request) == 0U) {
        const size_t frame_length =
            COMMAND_FRAME_HEADER_SIZE + request->request.header.payload_length + COMMAND_FRAME_CRC_SIZE;
        command_decoder_counter_add(&command_decoder_diagnostics.request_bytes_dropped, frame_length);
        return 0U;
    }
    return 1U;
}

uint8_t command_decoder_receive_response(CommandTaskResponseDTO_t *response, uint32_t timeout_ms) {
    if (response == NULL || command_decoder_response_queue == NULL) {
        return 0U;
    }
    const TickType_t timeout_ticks = timeout_ms == 0U ? 0U : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(command_decoder_response_queue, response, timeout_ticks) == pdTRUE ? 1U : 0U;
}

void command_decoder_get_diagnostics(CommandDecoderTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->protocol_version = COMMAND_PROTOCOL_VERSION;
    diagnostics->max_payload_size = COMMAND_MAX_PAYLOAD_SIZE;
    diagnostics->max_frame_size = COMMAND_MAX_FRAME_SIZE;
    diagnostics->requests_received = command_decoder_diagnostics.requests_received;
    diagnostics->request_queue_overflows = command_decoder_diagnostics.request_queue_overflows;
    diagnostics->request_bytes_dropped = command_decoder_diagnostics.request_bytes_dropped;
    diagnostics->commands_received = command_decoder_diagnostics.commands_received;
    diagnostics->commands_rejected = command_decoder_diagnostics.commands_rejected;
    diagnostics->pings_received = command_decoder_diagnostics.pings_received;
    diagnostics->responses_formed = command_decoder_diagnostics.responses_formed;
    diagnostics->pongs_queued = command_decoder_diagnostics.pongs_queued;
    diagnostics->errors_queued = command_decoder_diagnostics.errors_queued;
    diagnostics->invalid_payload_length = command_decoder_diagnostics.invalid_payload_length;
    diagnostics->unsupported_version = command_decoder_diagnostics.unsupported_version;
    diagnostics->unexpected_type = command_decoder_diagnostics.unexpected_type;
    diagnostics->invalid_sequence = command_decoder_diagnostics.invalid_sequence;
    diagnostics->unknown_commands = command_decoder_diagnostics.unknown_commands;
    diagnostics->invalid_states = command_decoder_diagnostics.invalid_states;
    diagnostics->response_queue_overflows = command_decoder_diagnostics.response_queue_overflows;
    diagnostics->last_error = command_decoder_diagnostics.last_error;
}

void CommandDecoderTask(void *argument) {
    (void)argument;
    for (;;) {
        CommandTaskRequestDTO_t request;
        if (xQueueReceive(command_decoder_request_queue, &request, portMAX_DELAY) == pdTRUE) {
            command_decoder_process_request(&request);
        }
    }
}

void command_decoder_get_response_queue_failure(uint32_t *generation, uint32_t *connection_id) {
    if (generation != NULL) {
        *generation = command_decoder_response_queue_failure_generation;
    }
    if (connection_id != NULL) {
        *connection_id = command_decoder_response_queue_failure_connection_id;
    }
}

#include "command_protocol.h"
#include <string.h>

static uint16_t command_read_u16_le(const uint8_t *data) { return (uint16_t)data[0] | ((uint16_t)data[1] << 8U); }

static uint32_t command_read_u32_le(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void command_write_u16_le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void command_write_u32_le(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void command_decode_header(const uint8_t *frame, CommandFrameHeaderDTO_t *header) {
    header->magic = command_read_u16_le(&frame[0]);
    header->version = frame[2];
    header->type = frame[3];
    header->payload_length = command_read_u16_le(&frame[4]);
    header->sequence = command_read_u32_le(&frame[6]);
}

static void command_assembler_discard_prefix(CommandFrameAssembler_t *assembler, size_t count) {
    if (count >= assembler->length) {
        assembler->length = 0U;
        return;
    }

    memmove(assembler->buffer, &assembler->buffer[count], assembler->length - count);
    assembler->length -= count;
}

uint32_t command_protocol_crc32(const uint8_t *data, size_t data_length) {
    uint32_t crc = UINT32_MAX;

    if (data == NULL && data_length != 0U) {
        return 0U;
    }

    for (size_t index = 0U; index < data_length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
        }
    }

    return crc ^ UINT32_MAX;
}

void command_frame_assembler_init(CommandFrameAssembler_t *assembler) {
    if (assembler != NULL) {
        assembler->length = 0U;
        assembler->max_length = 0U;
    }
}

size_t command_frame_assembler_append(CommandFrameAssembler_t *assembler, const uint8_t *data, size_t data_length) {
    if (assembler == NULL || assembler->length > COMMAND_MAX_FRAME_SIZE || (data == NULL && data_length != 0U)) {
        return 0U;
    }

    const size_t available = COMMAND_MAX_FRAME_SIZE - assembler->length;
    const size_t appended = data_length < available ? data_length : available;
    if (appended != 0U) {
        memcpy(&assembler->buffer[assembler->length], data, appended);
        assembler->length += appended;
        if (assembler->length > assembler->max_length) {
            assembler->max_length = assembler->length;
        }
    }
    return appended;
}

CommandFrameAssemblerEvent_t command_frame_assembler_next(const CommandFrameAssembler_t *assembler) {
    CommandFrameAssemblerEvent_t event = {
        .result = COMMAND_FRAME_ASSEMBLER_INVALID_ARGUMENT,
        .validation = COMMAND_FRAME_INVALID_ARGUMENT,
    };
    if (assembler == NULL || assembler->length > COMMAND_MAX_FRAME_SIZE) {
        return event;
    }

    event.result = COMMAND_FRAME_ASSEMBLER_NEED_MORE;
    event.validation = COMMAND_FRAME_VALID;
    if (assembler->length < COMMAND_FRAME_HEADER_SIZE) {
        return event;
    }

    const uint16_t magic = command_read_u16_le(&assembler->buffer[0]);
    if (magic != COMMAND_PROTOCOL_MAGIC) {
        event.result = COMMAND_FRAME_ASSEMBLER_INVALID_FRAME;
        event.frame = assembler->buffer;
        event.frame_length = assembler->length;
        event.consume_length = 1U;
        event.validation = COMMAND_FRAME_INVALID_MAGIC;
        return event;
    }

    const uint16_t payload_length = command_read_u16_le(&assembler->buffer[4]);
    if (payload_length > COMMAND_MAX_PAYLOAD_SIZE) {
        event.result = COMMAND_FRAME_ASSEMBLER_INVALID_FRAME;
        event.frame = assembler->buffer;
        event.frame_length = assembler->length;
        event.consume_length = 1U;
        event.validation = COMMAND_FRAME_INVALID_LENGTH;
        return event;
    }

    const size_t expected_length = COMMAND_FRAME_HEADER_SIZE + payload_length + COMMAND_FRAME_CRC_SIZE;
    if (assembler->length < expected_length) {
        return event;
    }

    event.frame = assembler->buffer;
    event.frame_length = expected_length;
    event.consume_length = expected_length;
    event.validation = command_protocol_decode_request(assembler->buffer, expected_length, &event.command);
    if (event.validation != COMMAND_FRAME_VALID) {
        event.result = COMMAND_FRAME_ASSEMBLER_INVALID_FRAME;
        event.consume_length = 1U;
        return event;
    }

    event.result = COMMAND_FRAME_ASSEMBLER_COMMAND_READY;
    return event;
}

uint8_t command_frame_assembler_consume(CommandFrameAssembler_t *assembler, size_t length) {
    if (assembler == NULL || length > assembler->length) {
        return 0U;
    }
    command_assembler_discard_prefix(assembler, length);
    return 1U;
}

CommandFrameValidationResult_t
command_protocol_decode_request(const uint8_t *frame, size_t frame_length, CommandRequestDTO_t *request) {
    if (request == NULL) {
        return COMMAND_FRAME_INVALID_ARGUMENT;
    }

    const CommandFrameValidationResult_t result =
        command_protocol_validate_frame(frame, frame_length, &request->header);
    if (result != COMMAND_FRAME_VALID) {
        (void)memset(request->payload, 0, sizeof(request->payload));
        return result;
    }

    (void)memset(request->payload, 0, sizeof(request->payload));
    (void)memcpy(request->payload, &frame[COMMAND_FRAME_HEADER_SIZE], request->header.payload_length);
    return COMMAND_FRAME_VALID;
}

uint16_t command_protocol_validate_request_dto(const CommandRequestDTO_t *request) {
    if (request == NULL) {
        return COMMAND_ERROR_INVALID_LENGTH;
    }
    return command_protocol_validate_request(&request->header, request->payload);
}

uint16_t command_protocol_validate_request(const CommandFrameHeaderDTO_t *header, const uint8_t *payload) {
    if (header == NULL || (payload == NULL && header->payload_length != 0U)) {
        return COMMAND_ERROR_INVALID_LENGTH;
    }
    if (header->payload_length > COMMAND_MAX_PAYLOAD_SIZE) {
        return COMMAND_ERROR_INVALID_LENGTH;
    }
    if (header->version != COMMAND_PROTOCOL_VERSION) {
        return COMMAND_ERROR_UNSUPPORTED_VERSION;
    }

    if (header->type == COMMAND_MESSAGE_TYPE_PING) {
        return header->payload_length == 0U ? 0U : COMMAND_ERROR_INVALID_LENGTH;
    }
    if (header->type != COMMAND_MESSAGE_TYPE_COMMAND) {
        return COMMAND_ERROR_UNEXPECTED_TYPE;
    }
    if (header->sequence == 0U) {
        return COMMAND_ERROR_INVALID_SEQUENCE;
    }
    if (header->payload_length == 0U) {
        return COMMAND_ERROR_INVALID_LENGTH;
    }

    switch (payload[0]) {
    case COMMAND_CODE_HOME:
    case COMMAND_CODE_MOTION_OPERATION:
    case COMMAND_CODE_SET_TEMPERATURE:
    case COMMAND_CODE_SET_OUTPUT:
    case COMMAND_CODE_CHANGE_TOOL:
    case COMMAND_CODE_STOP:
        /* The command is recognized; routing and execution are handled after decoding. */
        return COMMAND_ERROR_INVALID_STATE;
    default:
        return COMMAND_ERROR_UNKNOWN_COMMAND;
    }
}

CommandFrameValidationResult_t
command_protocol_validate_frame(const uint8_t *frame, size_t frame_length, CommandFrameHeaderDTO_t *header) {
    if (frame == NULL) {
        return COMMAND_FRAME_INVALID_ARGUMENT;
    }
    if (frame_length < COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE) {
        return COMMAND_FRAME_INVALID_LENGTH;
    }

    CommandFrameHeaderDTO_t decoded_header;
    command_decode_header(frame, &decoded_header);
    if (header != NULL) {
        *header = decoded_header;
    }
    if (decoded_header.magic != COMMAND_PROTOCOL_MAGIC) {
        return COMMAND_FRAME_INVALID_MAGIC;
    }
    if (decoded_header.payload_length > COMMAND_MAX_PAYLOAD_SIZE) {
        return COMMAND_FRAME_INVALID_LENGTH;
    }

    const size_t expected_length = COMMAND_FRAME_HEADER_SIZE + decoded_header.payload_length + COMMAND_FRAME_CRC_SIZE;
    if (frame_length != expected_length) {
        return COMMAND_FRAME_INVALID_LENGTH;
    }

    const uint32_t received_crc = command_read_u32_le(&frame[expected_length - COMMAND_FRAME_CRC_SIZE]);
    const uint32_t calculated_crc = command_protocol_crc32(frame, expected_length - COMMAND_FRAME_CRC_SIZE);
    return received_crc == calculated_crc ? COMMAND_FRAME_VALID : COMMAND_FRAME_INVALID_CRC;
}

CommandFrameValidationResult_t
command_protocol_parse_header(const uint8_t *frame, size_t available_length, CommandFrameHeaderDTO_t *header) {
    if (frame == NULL || header == NULL) {
        return COMMAND_FRAME_INVALID_ARGUMENT;
    }
    if (available_length < COMMAND_FRAME_HEADER_SIZE) {
        return COMMAND_FRAME_INVALID_LENGTH;
    }

    command_decode_header(frame, header);
    if (header->magic != COMMAND_PROTOCOL_MAGIC) {
        return COMMAND_FRAME_INVALID_MAGIC;
    }
    if (header->payload_length > COMMAND_MAX_PAYLOAD_SIZE) {
        return COMMAND_FRAME_INVALID_LENGTH;
    }
    return COMMAND_FRAME_VALID;
}

size_t command_protocol_build_frame(
    uint8_t *frame,
    size_t frame_capacity,
    uint8_t type,
    uint32_t sequence,
    const uint8_t *payload,
    uint16_t payload_length
) {
    const size_t frame_length = COMMAND_FRAME_HEADER_SIZE + payload_length + COMMAND_FRAME_CRC_SIZE;
    if (frame == NULL || (payload == NULL && payload_length != 0U) || payload_length > COMMAND_MAX_PAYLOAD_SIZE ||
        frame_capacity < frame_length) {
        return 0U;
    }

    command_write_u16_le(&frame[0], COMMAND_PROTOCOL_MAGIC);
    frame[2] = COMMAND_PROTOCOL_VERSION;
    frame[3] = type;
    command_write_u16_le(&frame[4], payload_length);
    command_write_u32_le(&frame[6], sequence);
    if (payload_length != 0U) {
        memcpy(&frame[COMMAND_FRAME_HEADER_SIZE], payload, payload_length);
    }

    const uint32_t crc = command_protocol_crc32(frame, frame_length - COMMAND_FRAME_CRC_SIZE);
    command_write_u32_le(&frame[frame_length - COMMAND_FRAME_CRC_SIZE], crc);
    return frame_length;
}

size_t command_protocol_build_response(uint8_t *frame, size_t frame_capacity, const CommandResponseDTO_t *response) {
    if (response == NULL) {
        return 0U;
    }
    return command_protocol_build_frame(
        frame,
        frame_capacity,
        response->type,
        response->sequence,
        response->payload,
        response->payload_length
    );
}

size_t command_protocol_build_pong_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence) {
    const CommandResponseDTO_t response = {
        .type = COMMAND_MESSAGE_TYPE_PONG,
        .sequence = sequence,
        .payload_length = 0U,
    };
    return command_protocol_build_response(frame, frame_capacity, &response);
}

size_t command_protocol_build_ack_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence) {
    const CommandResponseDTO_t response = {
        .type = COMMAND_MESSAGE_TYPE_ACK,
        .sequence = sequence,
        .payload_length = 0U,
    };
    return command_protocol_build_response(frame, frame_capacity, &response);
}

size_t
command_protocol_build_error_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence, uint16_t error_code) {
    CommandResponseDTO_t response = {
        .type = COMMAND_MESSAGE_TYPE_ERROR,
        .sequence = sequence,
        .payload_length = sizeof(CommandErrorPayloadDTO_t),
    };
    command_write_u16_le(response.payload, error_code);
    return command_protocol_build_response(frame, frame_capacity, &response);
}

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

static void command_discard_prefix(CommandFrameParser_t *parser, size_t count) {
    if (count >= parser->length) {
        parser->length = 0U;
        return;
    }

    memmove(parser->buffer, &parser->buffer[count], parser->length - count);
    parser->length -= count;
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

void command_frame_parser_init(CommandFrameParser_t *parser) {
    if (parser != NULL) {
        parser->length = 0U;
    }
}

static void command_frame_parser_process(CommandFrameParser_t *parser, CommandFrameCallback callback, void *context) {
    while (parser->length >= COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE) {
        const uint16_t magic = command_read_u16_le(&parser->buffer[0]);
        if (magic != COMMAND_PROTOCOL_MAGIC) {
            command_discard_prefix(parser, 1U);
            continue;
        }

        const uint16_t payload_length = command_read_u16_le(&parser->buffer[4]);
        if (payload_length > COMMAND_MAX_PAYLOAD_SIZE) {
            command_discard_prefix(parser, 1U);
            continue;
        }

        const size_t frame_length = COMMAND_FRAME_HEADER_SIZE + payload_length + COMMAND_FRAME_CRC_SIZE;
        if (parser->length < frame_length) {
            return;
        }

        const uint32_t received_crc = command_read_u32_le(&parser->buffer[COMMAND_FRAME_HEADER_SIZE + payload_length]);
        const uint32_t calculated_crc = command_protocol_crc32(parser->buffer, frame_length - COMMAND_FRAME_CRC_SIZE);
        if (received_crc != calculated_crc) {
            command_discard_prefix(parser, 1U);
            continue;
        }

        if (callback != NULL) {
            callback(parser->buffer, frame_length, context);
        }
        command_discard_prefix(parser, frame_length);
    }
}

void command_frame_parser_feed(
    CommandFrameParser_t *parser,
    const uint8_t *data,
    size_t data_length,
    CommandFrameCallback callback,
    void *context
) {
    if (parser == NULL || (data == NULL && data_length != 0U)) {
        return;
    }

    for (size_t index = 0U; index < data_length; ++index) {
        if (parser->length == COMMAND_MAX_FRAME_SIZE) {
            command_discard_prefix(parser, 1U);
        }

        parser->buffer[parser->length] = data[index];
        ++parser->length;
        command_frame_parser_process(parser, callback, context);
    }
}

uint16_t command_protocol_validate_request(const CommandFrameHeaderDTO_t *header, const uint8_t *payload) {
    if (header == NULL || (payload == NULL && header->payload_length != 0U)) {
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
        /* The command codes are reserved, but their executors are not part of this task yet. */
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

size_t command_protocol_build_pong_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence) {
    return command_protocol_build_frame(frame, frame_capacity, COMMAND_MESSAGE_TYPE_PONG, sequence, NULL, 0U);
}

size_t
command_protocol_build_error_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence, uint16_t error_code) {
    uint8_t payload[sizeof(CommandErrorPayloadDTO_t)];
    command_write_u16_le(payload, error_code);
    return command_protocol_build_frame(
        frame,
        frame_capacity,
        COMMAND_MESSAGE_TYPE_ERROR,
        sequence,
        payload,
        sizeof(payload)
    );
}
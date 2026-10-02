#include "command_protocol.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;

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

typedef struct {
    uint32_t count;
    size_t lengths[4];
    uint8_t frames[4][COMMAND_MAX_FRAME_SIZE];
} FrameCapture;

typedef struct {
    uint32_t count;
    CommandFrameValidationResult_t results[16];
} ErrorCapture;

static void capture_frame(const uint8_t *frame, size_t frame_length, void *context) {
    FrameCapture *capture = context;
    if (capture->count < 4U) {
        capture->lengths[capture->count] = frame_length;
        memcpy(capture->frames[capture->count], frame, frame_length);
    }
    ++capture->count;
}

static void capture_error(CommandFrameValidationResult_t result, void *context) {
    ErrorCapture *capture = context;
    if (capture->count < sizeof(capture->results) / sizeof(capture->results[0])) {
        capture->results[capture->count] = result;
    }
    ++capture->count;
}

static size_t build_ping(uint8_t *frame, uint32_t sequence) {
    return command_protocol_build_frame(frame, COMMAND_MAX_FRAME_SIZE, COMMAND_MESSAGE_TYPE_PING, sequence, NULL, 0U);
}

static void test_crc(void) {
    const uint8_t text[] = "123456789";
    expect_u32(command_protocol_crc32(text, sizeof(text) - 1U), 0xCBF43926U, "CRC-32");
    expect_u32(command_protocol_crc32(NULL, 0U), 0U, "empty CRC input");
    expect_u32(command_protocol_crc32(NULL, 1U), 0U, "null CRC input is rejected");
}

static void test_parser_error_callbacks_and_arguments(void) {
    CommandFrameParser_t parser;
    ErrorCapture errors = {0};
    command_frame_parser_init(NULL);
    command_frame_parser_init(&parser);

    command_frame_parser_feed_ex(NULL, NULL, 1U, NULL, capture_error, &errors);
    command_frame_parser_feed_ex(&parser, NULL, 1U, NULL, capture_error, &errors);
    command_frame_parser_feed_ex(&parser, NULL, 1U, NULL, NULL, &errors);
    command_frame_parser_feed_ex(&parser, NULL, 0U, NULL, NULL, NULL);
    expect_u32(errors.count, 2U, "invalid parser inputs notify a configured error callback");
    expect_u32(errors.results[0], COMMAND_FRAME_INVALID_ARGUMENT, "null parser error code");
    expect_u32(errors.results[1], COMMAND_FRAME_INVALID_ARGUMENT, "null data error code");

    uint8_t invalid_magic[COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE] = {0};
    command_frame_parser_init(&parser);
    command_frame_parser_feed_ex(&parser, invalid_magic, sizeof(invalid_magic), NULL, capture_error, &errors);
    expect_u32(errors.results[2], COMMAND_FRAME_INVALID_MAGIC, "parser reports invalid magic");

    uint8_t invalid_length[COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE] = {
        (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU),
        (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U),
        COMMAND_PROTOCOL_VERSION,
        COMMAND_MESSAGE_TYPE_PING,
        (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU),
        (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U),
    };
    command_frame_parser_init(&parser);
    command_frame_parser_feed_ex(&parser, invalid_length, sizeof(invalid_length), NULL, capture_error, &errors);
    expect_u32(errors.results[3], COMMAND_FRAME_INVALID_LENGTH, "parser reports oversized payload");

    uint8_t invalid_crc[COMMAND_MAX_FRAME_SIZE];
    const size_t valid_length = build_ping(invalid_crc, 19U);
    invalid_crc[valid_length - 1U] ^= 1U;
    command_frame_parser_init(&parser);
    command_frame_parser_feed_ex(&parser, invalid_crc, valid_length, NULL, capture_error, &errors);
    expect_u32(errors.results[4], COMMAND_FRAME_INVALID_CRC, "parser reports invalid CRC");

    uint8_t valid_frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(valid_frame, 20U);
    command_frame_parser_init(&parser);
    command_frame_parser_feed_ex(&parser, valid_frame, frame_length, NULL, capture_error, &errors);
    expect_u32(errors.count, 5U, "valid frame with no callback is silently consumed");

    command_frame_parser_init(&parser);
    memset(parser.buffer, 0, sizeof(parser.buffer));
    parser.length = COMMAND_MAX_FRAME_SIZE;
    command_frame_parser_feed_ex(&parser, valid_frame, 1U, NULL, NULL, NULL);
    expect_u32((uint32_t)parser.max_length, COMMAND_MAX_FRAME_SIZE, "full parser buffer updates high-water mark");
}

static void test_partial_and_compound_stream(void) {
    uint8_t first[COMMAND_MAX_FRAME_SIZE];
    uint8_t second[COMMAND_MAX_FRAME_SIZE];
    uint8_t combined[COMMAND_MAX_FRAME_SIZE * 2U];
    const size_t first_length = build_ping(first, 11U);
    const size_t second_length = build_ping(second, 12U);
    FrameCapture capture = {0};
    CommandFrameParser_t parser;
    command_frame_parser_init(&parser);

    command_frame_parser_feed(&parser, first, 3U, capture_frame, &capture);
    expect_u32(capture.count, 0U, "partial frame is not delivered early");
    command_frame_parser_feed(&parser, &first[3], first_length - 3U, capture_frame, &capture);
    expect_u32(capture.count, 1U, "partial frame is delivered after completion");
    expect_u32((uint32_t)capture.lengths[0], (uint32_t)first_length, "partial frame length");

    memcpy(combined, first, first_length);
    memcpy(&combined[first_length], second, second_length);
    command_frame_parser_init(&parser);
    capture.count = 0U;
    command_frame_parser_feed(&parser, combined, first_length + second_length, capture_frame, &capture);
    expect_u32(capture.count, 2U, "two frames in one receive are delivered separately");
}

static void test_crc_and_size_rejection(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    FrameCapture capture = {0};
    CommandFrameParser_t parser;
    const size_t frame_length = build_ping(frame, 20U);
    frame[frame_length - 1U] ^= 0x80U;

    command_frame_parser_init(&parser);
    command_frame_parser_feed(&parser, frame, frame_length, capture_frame, &capture);
    expect_u32(capture.count, 0U, "invalid CRC is rejected");

    uint8_t oversized_header[COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE] = {0x5AU,
        0xA5U,
        COMMAND_PROTOCOL_VERSION,
        COMMAND_MESSAGE_TYPE_PING,
        0x01U,
        0x01U,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U};
    capture.count = 0U;
    command_frame_parser_init(&parser);
    command_frame_parser_feed(&parser, oversized_header, sizeof(oversized_header), capture_frame, &capture);
    expect_u32(capture.count, 0U, "payload larger than maximum is rejected");

    const uint8_t payload_with_magic[] = {0xEEU, 0x5AU, 0xA5U};
    const size_t command_length = command_protocol_build_frame(
        frame,
        sizeof(frame),
        COMMAND_MESSAGE_TYPE_COMMAND,
        21U,
        payload_with_magic,
        sizeof(payload_with_magic)
    );
    capture.count = 0U;
    command_frame_parser_init(&parser);
    command_frame_parser_feed(&parser, frame, command_length, capture_frame, &capture);
    expect_u32(capture.count, 1U, "magic inside payload does not split a frame");
}

static void test_request_validation(void) {
    CommandFrameHeaderDTO_t header = {.magic = COMMAND_PROTOCOL_MAGIC,
        .version = COMMAND_PROTOCOL_VERSION,
        .type = COMMAND_MESSAGE_TYPE_PING,
        .payload_length = 0U,
        .sequence = 1U};
    expect_u32(command_protocol_validate_request(&header, NULL), 0U, "valid PING request");

    header.payload_length = 1U;
    expect_u32(
        command_protocol_validate_request(&header, (const uint8_t[]){0U}),
        COMMAND_ERROR_INVALID_LENGTH,
        "PING payload length"
    );

    header.type = COMMAND_MESSAGE_TYPE_COMMAND;
    header.payload_length = 0U;
    expect_u32(command_protocol_validate_request(&header, NULL), COMMAND_ERROR_INVALID_LENGTH, "empty COMMAND");

    header.payload_length = 1U;
    header.sequence = 0U;
    expect_u32(
        command_protocol_validate_request(&header, (const uint8_t[]){COMMAND_CODE_STOP}),
        COMMAND_ERROR_INVALID_SEQUENCE,
        "zero COMMAND sequence"
    );

    header.sequence = 2U;
    expect_u32(
        command_protocol_validate_request(&header, (const uint8_t[]){0xFEU}),
        COMMAND_ERROR_UNKNOWN_COMMAND,
        "unknown command code"
    );
    expect_u32(
        command_protocol_validate_request(&header, (const uint8_t[]){COMMAND_CODE_STOP}),
        COMMAND_ERROR_INVALID_STATE,
        "known but unsupported command"
    );

    header.type = COMMAND_MESSAGE_TYPE_PONG;
    header.payload_length = 0U;
    expect_u32(
        command_protocol_validate_request(&header, NULL),
        COMMAND_ERROR_UNEXPECTED_TYPE,
        "server response type is not accepted as request"
    );

    header.version = 2U;
    expect_u32(
        command_protocol_validate_request(&header, NULL),
        COMMAND_ERROR_UNSUPPORTED_VERSION,
        "unsupported protocol version"
    );

    expect_u32(command_protocol_validate_request(NULL, NULL), COMMAND_ERROR_INVALID_LENGTH, "null request header");
    header.version = COMMAND_PROTOCOL_VERSION;
    header.type = COMMAND_MESSAGE_TYPE_COMMAND;
    header.payload_length = 1U;
    expect_u32(command_protocol_validate_request(&header, NULL), COMMAND_ERROR_INVALID_LENGTH, "null request payload");
    header.sequence = 1U;
    const uint8_t reserved_codes[] = {
        COMMAND_CODE_HOME,
        COMMAND_CODE_MOTION_OPERATION,
        COMMAND_CODE_SET_TEMPERATURE,
        COMMAND_CODE_SET_OUTPUT,
        COMMAND_CODE_CHANGE_TOOL,
        COMMAND_CODE_STOP,
    };
    for (size_t index = 0U; index < sizeof(reserved_codes); ++index) {
        expect_u32(
            command_protocol_validate_request(&header, &reserved_codes[index]),
            COMMAND_ERROR_INVALID_STATE,
            "reserved command is rejected as unsupported state"
        );
    }
}

static void test_frame_validation_and_decode_errors(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 27U);
    CommandFrameHeaderDTO_t header;
    expect_u32(command_protocol_validate_frame(NULL, frame_length, &header), COMMAND_FRAME_INVALID_ARGUMENT, "null frame");
    expect_u32(
        command_protocol_validate_frame(frame, COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE - 1U, &header),
        COMMAND_FRAME_INVALID_LENGTH,
        "frame shorter than its fixed header"
    );

    uint8_t altered[COMMAND_MAX_FRAME_SIZE];
    memcpy(altered, frame, frame_length);
    altered[0] ^= 1U;
    expect_u32(command_protocol_validate_frame(altered, frame_length, &header), COMMAND_FRAME_INVALID_MAGIC, "invalid frame magic");

    uint8_t oversized[COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE] = {
        (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU),
        (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U),
        COMMAND_PROTOCOL_VERSION,
        COMMAND_MESSAGE_TYPE_PING,
        (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU),
        (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U),
    };
    expect_u32(command_protocol_validate_frame(oversized, sizeof(oversized), &header), COMMAND_FRAME_INVALID_LENGTH, "oversized frame payload");
    expect_u32(command_protocol_validate_frame(frame, frame_length - 1U, &header), COMMAND_FRAME_INVALID_LENGTH, "truncated frame");
    expect_u32(command_protocol_validate_frame(frame, frame_length + 1U, &header), COMMAND_FRAME_INVALID_LENGTH, "trailing frame data");

    memcpy(altered, frame, frame_length);
    altered[frame_length - 1U] ^= 1U;
    expect_u32(command_protocol_validate_frame(altered, frame_length, NULL), COMMAND_FRAME_INVALID_CRC, "invalid frame CRC without header output");
    expect_u32(command_protocol_validate_frame(frame, frame_length, NULL), COMMAND_FRAME_VALID, "valid frame without header output");

    expect_u32(command_protocol_decode_request(frame, frame_length, NULL), COMMAND_FRAME_INVALID_ARGUMENT, "null decode output");
    CommandRequestDTO_t request;
    memset(&request, 0xFF, sizeof(request));
    expect_u32(command_protocol_decode_request(altered, frame_length, &request), COMMAND_FRAME_INVALID_CRC, "decode rejects invalid CRC");
    expect_u32(request.payload[0], 0U, "failed decode clears payload storage");
    expect_u32(command_protocol_decode_request(frame, frame_length, &request), COMMAND_FRAME_VALID, "valid decode succeeds");
    expect_u32(command_protocol_validate_request_dto(NULL), COMMAND_ERROR_INVALID_LENGTH, "null request DTO");
}

static void test_frame_build_errors(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const uint8_t payload[] = {1U};
    expect_u32((uint32_t)command_protocol_build_frame(NULL, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 1U, NULL, 0U), 0U, "null frame output");
    expect_u32((uint32_t)command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 1U, NULL, 1U), 0U, "null payload with nonzero length");
    expect_u32(
        (uint32_t)command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 1U, payload, (uint16_t)(COMMAND_MAX_PAYLOAD_SIZE + 1U)),
        0U,
        "payload above protocol maximum"
    );
    expect_u32(
        (uint32_t)command_protocol_build_frame(frame, COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE - 1U, COMMAND_MESSAGE_TYPE_PING, 1U, NULL, 0U),
        0U,
        "insufficient frame capacity"
    );
    expect_u32((uint32_t)command_protocol_build_response(frame, sizeof(frame), NULL), 0U, "null response DTO");

    const CommandResponseDTO_t invalid_response = {
        .type = COMMAND_MESSAGE_TYPE_ERROR,
        .sequence = 1U,
        .payload_length = (uint16_t)(COMMAND_MAX_PAYLOAD_SIZE + 1U),
    };
    expect_u32(
        (uint32_t)command_protocol_build_response(frame, sizeof(frame), &invalid_response),
        0U,
        "response payload above protocol maximum is rejected"
    );
}

static void test_response_builders(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    CommandFrameHeaderDTO_t header;
    const size_t pong_length = command_protocol_build_pong_response(frame, sizeof(frame), 31U);
    expect_u32((uint32_t)pong_length, 14U, "PONG frame size");
    expect_u32(command_protocol_validate_frame(frame, pong_length, &header), COMMAND_FRAME_VALID, "PONG frame CRC");
    expect_u32(header.type, COMMAND_MESSAGE_TYPE_PONG, "PONG type");
    expect_u32(header.sequence, 31U, "PONG sequence");

    const size_t error_length =
        command_protocol_build_error_response(frame, sizeof(frame), 32U, COMMAND_ERROR_INVALID_LENGTH);
    expect_u32((uint32_t)error_length, 16U, "ERROR frame size");
    expect_u32(command_protocol_validate_frame(frame, error_length, &header), COMMAND_FRAME_VALID, "ERROR frame CRC");
    expect_u32(header.type, COMMAND_MESSAGE_TYPE_ERROR, "ERROR type");
    expect_u32(header.payload_length, 2U, "ERROR payload size");
    expect_u32(header.sequence, 32U, "ERROR sequence");
    expect_u32(frame[COMMAND_FRAME_HEADER_SIZE], COMMAND_ERROR_INVALID_LENGTH & 0xFFU, "ERROR code low byte");
    expect_u32(frame[COMMAND_FRAME_HEADER_SIZE + 1U], COMMAND_ERROR_INVALID_LENGTH >> 8U, "ERROR code high byte");
}

static void test_transport_dtos(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t request_length = build_ping(frame, 35U);
    CommandRequestDTO_t request = {0};
    expect_u32(
        command_protocol_decode_request(frame, request_length, &request),
        COMMAND_FRAME_VALID,
        "transport request DTO is decoded"
    );
    expect_u32(request.header.sequence, 35U, "request DTO keeps sequence");
    expect_u32(request.payload[0], 0U, "request DTO owns payload storage");
    expect_u32(command_protocol_validate_request_dto(&request), 0U, "request DTO is validated");

    const CommandResponseDTO_t response = {
        .type = COMMAND_MESSAGE_TYPE_PONG,
        .sequence = 36U,
        .payload_length = 0U,
    };
    const size_t response_length = command_protocol_build_response(frame, sizeof(frame), &response);
    CommandFrameHeaderDTO_t response_header;
    expect_u32(
        command_protocol_validate_frame(frame, response_length, &response_header),
        COMMAND_FRAME_VALID,
        "transport response DTO is serialized"
    );
    expect_u32(response_header.sequence, 36U, "response DTO keeps sequence");
}
int main(void) {
    test_crc();
    test_parser_error_callbacks_and_arguments();
    test_partial_and_compound_stream();
    test_crc_and_size_rejection();
    test_request_validation();
    test_frame_validation_and_decode_errors();
    test_frame_build_errors();
    test_response_builders();
    test_transport_dtos();
    return test_failures == 0U ? 0 : 1;
}

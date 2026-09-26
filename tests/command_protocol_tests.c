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

static void capture_frame(const uint8_t *frame, size_t frame_length, void *context) {
    FrameCapture *capture = context;
    if (capture->count < 4U) {
        capture->lengths[capture->count] = frame_length;
        memcpy(capture->frames[capture->count], frame, frame_length);
    }
    ++capture->count;
}

static size_t build_ping(uint8_t *frame, uint32_t sequence) {
    return command_protocol_build_frame(frame, COMMAND_MAX_FRAME_SIZE, COMMAND_MESSAGE_TYPE_PING, sequence, NULL, 0U);
}

static void test_crc(void) {
    const uint8_t text[] = "123456789";
    expect_u32(command_protocol_crc32(text, sizeof(text) - 1U), 0xCBF43926U, "CRC-32");
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

int main(void) {
    test_crc();
    test_partial_and_compound_stream();
    test_crc_and_size_rejection();
    test_request_validation();
    test_response_builders();
    return test_failures == 0U ? 0 : 1;
}

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

static size_t build_ping(uint8_t *frame, uint32_t sequence) {
    return command_protocol_build_frame(
        frame,
        COMMAND_MAX_FRAME_SIZE,
        COMMAND_MESSAGE_TYPE_PING,
        sequence,
        NULL,
        0U
    );
}

static void test_crc_and_assembler_buffer_operations(void) {
    static const uint8_t crc_text[] = "123456789";
    expect_u32(command_protocol_crc32(NULL, 0U), 0U, "empty CRC input is supported");
    expect_u32(command_protocol_crc32(NULL, 1U), 0U, "null nonempty CRC input is rejected");
    expect_u32(command_protocol_crc32(crc_text, sizeof(crc_text) - 1U), 0xCBF43926U, "CRC32 matches standard vector");

    command_frame_assembler_init(NULL);
    CommandFrameAssembler_t assembler = {0};
    assembler.max_length = 20U;
    command_frame_assembler_init(&assembler);
    expect_u32((uint32_t)assembler.length, 0U, "assembler init clears length");
    expect_u32((uint32_t)assembler.max_length, 0U, "assembler init clears high-water mark");

    const uint8_t prefix[] = {0x5AU, 0xA5U, 1U, 2U};
    expect_u32((uint32_t)command_frame_assembler_append(NULL, prefix, sizeof(prefix)), 0U, "null assembler rejects append");
    assembler.length = COMMAND_MAX_FRAME_SIZE + 1U;
    expect_u32(
        (uint32_t)command_frame_assembler_append(&assembler, prefix, sizeof(prefix)),
        0U,
        "oversized assembler state rejects append"
    );

    command_frame_assembler_init(&assembler);
    expect_u32(
        (uint32_t)command_frame_assembler_append(&assembler, NULL, sizeof(prefix)),
        0U,
        "null nonempty append data is rejected"
    );
    expect_u32(
        (uint32_t)command_frame_assembler_append(&assembler, NULL, 0U),
        0U,
        "null empty append data is accepted"
    );
    expect_u32((uint32_t)command_frame_assembler_append(&assembler, prefix, 0U), 0U, "zero-byte append is accepted");
    expect_u32((uint32_t)assembler.max_length, 0U, "empty append does not raise high-water mark");
    expect_u32(
        (uint32_t)command_frame_assembler_append(&assembler, prefix, sizeof(prefix)),
        (uint32_t)sizeof(prefix),
        "assembler appends available bytes"
    );
    expect_u32((uint32_t)assembler.max_length, (uint32_t)sizeof(prefix), "append updates high-water mark");
    assembler.max_length = COMMAND_MAX_FRAME_SIZE;
    expect_u32(
        (uint32_t)command_frame_assembler_append(&assembler, prefix, 1U),
        1U,
        "assembler can append after an existing prefix"
    );
    expect_u32((uint32_t)assembler.max_length, COMMAND_MAX_FRAME_SIZE, "high-water mark never decreases");
    assembler.length = COMMAND_MAX_FRAME_SIZE;
    expect_u32((uint32_t)command_frame_assembler_append(&assembler, prefix, 1U), 0U, "full assembler appends no bytes");

    command_frame_assembler_init(&assembler);
    expect_u32(command_frame_assembler_consume(NULL, 0U), 0U, "null assembler rejects consume");
    expect_u32(command_frame_assembler_consume(&assembler, 1U), 0U, "consume beyond buffered length is rejected");
    (void)command_frame_assembler_append(&assembler, prefix, sizeof(prefix));
    expect_u32(command_frame_assembler_consume(&assembler, 2U), 1U, "consume removes a prefix");
    expect_u32((uint32_t)assembler.length, 2U, "prefix consume keeps trailing bytes");
    expect_u32(assembler.buffer[0], 1U, "prefix consume shifts trailing data");
    expect_u32(command_frame_assembler_consume(&assembler, assembler.length), 1U, "consume can clear the full buffer");
    expect_u32((uint32_t)assembler.length, 0U, "full consume clears length");
    expect_u32(command_frame_assembler_consume(&assembler, 0U), 1U, "zero-byte consume is valid");
}

static void test_assembler_frame_events(void) {
    CommandFrameAssembler_t assembler = {0};
    CommandFrameAssemblerEvent_t event = command_frame_assembler_next(NULL);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_INVALID_ARGUMENT, "null assembler yields invalid argument");

    assembler.length = COMMAND_MAX_FRAME_SIZE + 1U;
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_INVALID_ARGUMENT, "oversized assembler yields invalid argument");

    command_frame_assembler_init(&assembler);
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_NEED_MORE, "empty assembler needs more data");
    const uint8_t short_header[] = {0x5AU, 0xA5U};
    (void)command_frame_assembler_append(&assembler, short_header, sizeof(short_header));
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_NEED_MORE, "short header needs more data");

    command_frame_assembler_init(&assembler);
    uint8_t bad_magic[COMMAND_FRAME_HEADER_SIZE] = {0};
    bad_magic[4] = 0U;
    bad_magic[5] = 0U;
    (void)command_frame_assembler_append(&assembler, bad_magic, sizeof(bad_magic));
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_INVALID_FRAME, "bad magic is an invalid frame");
    expect_u32(event.validation, COMMAND_FRAME_INVALID_MAGIC, "bad magic reports its validation error");
    expect_u32((uint32_t)event.consume_length, 1U, "bad magic discards one byte for resynchronization");

    command_frame_assembler_init(&assembler);
    uint8_t oversized_header[COMMAND_FRAME_HEADER_SIZE] = {0};
    oversized_header[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    oversized_header[1] = (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U);
    oversized_header[4] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU);
    oversized_header[5] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U);
    (void)command_frame_assembler_append(&assembler, oversized_header, sizeof(oversized_header));
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_INVALID_FRAME, "oversized header is an invalid frame");
    expect_u32(event.validation, COMMAND_FRAME_INVALID_LENGTH, "oversized header reports invalid length");

    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 11U);
    command_frame_assembler_init(&assembler);
    (void)command_frame_assembler_append(&assembler, frame, COMMAND_FRAME_HEADER_SIZE);
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_NEED_MORE, "valid incomplete frame needs more data");
    (void)command_frame_assembler_append(
        &assembler,
        &frame[COMMAND_FRAME_HEADER_SIZE],
        frame_length - COMMAND_FRAME_HEADER_SIZE
    );
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_COMMAND_READY, "complete PING frame is returned");
    expect_u32(event.validation, COMMAND_FRAME_VALID, "complete frame is validated");
    expect_u32((uint32_t)event.frame_length, (uint32_t)frame_length, "assembler reports frame length");
    expect_u32(event.command.header.sequence, 11U, "assembler returns decoded request");
    expect_u32(event.command.header.type, COMMAND_MESSAGE_TYPE_PING, "decoded request retains message type");

    command_frame_assembler_init(&assembler);
    frame[frame_length - 1U] ^= 0x80U;
    (void)command_frame_assembler_append(&assembler, frame, frame_length);
    event = command_frame_assembler_next(&assembler);
    expect_u32(event.result, COMMAND_FRAME_ASSEMBLER_INVALID_FRAME, "invalid CRC is rejected by assembler");
    expect_u32(event.validation, COMMAND_FRAME_INVALID_CRC, "assembler reports invalid CRC");
}

static void test_header_frame_and_request_validation(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const size_t frame_length = build_ping(frame, 21U);
    CommandFrameHeaderDTO_t header = {0};
    expect_u32(command_protocol_parse_header(NULL, COMMAND_FRAME_HEADER_SIZE, &header), COMMAND_FRAME_INVALID_ARGUMENT,
               "null header input is rejected");
    expect_u32(command_protocol_parse_header(frame, COMMAND_FRAME_HEADER_SIZE, NULL), COMMAND_FRAME_INVALID_ARGUMENT,
               "null header output is rejected");
    expect_u32(
        command_protocol_parse_header(frame, COMMAND_FRAME_HEADER_SIZE - 1U, &header),
        COMMAND_FRAME_INVALID_LENGTH,
        "short header is rejected"
    );
    expect_u32(command_protocol_parse_header(frame, frame_length, &header), COMMAND_FRAME_VALID, "valid header is parsed");
    expect_u32(header.sequence, 21U, "header parser extracts sequence");

    uint8_t altered[COMMAND_MAX_FRAME_SIZE];
    memcpy(altered, frame, frame_length);
    altered[0] ^= 1U;
    expect_u32(command_protocol_parse_header(altered, frame_length, &header), COMMAND_FRAME_INVALID_MAGIC,
               "header parser rejects bad magic");
    expect_u32(header.sequence, 21U, "header parser extracts fields despite bad magic");
    altered[0] = (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU);
    altered[1] = (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U);
    altered[4] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU);
    altered[5] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U);
    expect_u32(command_protocol_parse_header(altered, sizeof(altered), &header), COMMAND_FRAME_INVALID_LENGTH,
               "header parser rejects oversized payload");

    expect_u32(command_protocol_validate_frame(NULL, frame_length, &header), COMMAND_FRAME_INVALID_ARGUMENT,
               "null frame is rejected");
    expect_u32(
        command_protocol_validate_frame(frame, COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE - 1U, &header),
        COMMAND_FRAME_INVALID_LENGTH,
        "frame shorter than fixed fields is rejected"
    );
    memcpy(altered, frame, frame_length);
    altered[0] ^= 1U;
    expect_u32(command_protocol_validate_frame(altered, frame_length, &header), COMMAND_FRAME_INVALID_MAGIC,
               "frame validation rejects bad magic");
    memcpy(altered, frame, frame_length);
    altered[4] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) & 0xFFU);
    altered[5] = (uint8_t)((COMMAND_MAX_PAYLOAD_SIZE + 1U) >> 8U);
    expect_u32(command_protocol_validate_frame(altered, frame_length, &header), COMMAND_FRAME_INVALID_LENGTH,
               "frame validation rejects oversized payload");
    expect_u32(command_protocol_validate_frame(frame, frame_length - 1U, &header), COMMAND_FRAME_INVALID_LENGTH,
               "truncated frame is rejected");
    expect_u32(command_protocol_validate_frame(frame, frame_length + 1U, &header), COMMAND_FRAME_INVALID_LENGTH,
               "frame with trailing data is rejected");
    memcpy(altered, frame, frame_length);
    altered[frame_length - 1U] ^= 1U;
    expect_u32(command_protocol_validate_frame(altered, frame_length, NULL), COMMAND_FRAME_INVALID_CRC,
               "bad CRC is rejected without header output");
    expect_u32(command_protocol_validate_frame(frame, frame_length, NULL), COMMAND_FRAME_VALID,
               "valid frame does not require header output");

    CommandRequestDTO_t request;
    memset(&request, 0xFF, sizeof(request));
    expect_u32(command_protocol_decode_request(frame, frame_length, NULL), COMMAND_FRAME_INVALID_ARGUMENT,
               "null decoded request is rejected");
    expect_u32(command_protocol_decode_request(altered, frame_length, &request), COMMAND_FRAME_INVALID_CRC,
               "decode clears payload after frame validation failure");
    expect_u32(request.payload[0], 0U, "failed request decoding clears payload bytes");
    expect_u32(command_protocol_decode_request(frame, frame_length, &request), COMMAND_FRAME_VALID,
               "valid request is decoded");
    expect_u32(request.header.sequence, 21U, "request DTO preserves sequence");
    expect_u32(command_protocol_validate_request_dto(NULL), COMMAND_ERROR_INVALID_LENGTH, "null request DTO is invalid");

    CommandFrameHeaderDTO_t request_header = {
        .version = COMMAND_PROTOCOL_VERSION,
        .type = COMMAND_MESSAGE_TYPE_PING,
        .payload_length = 0U,
        .sequence = 1U,
    };
    expect_u32(command_protocol_validate_request(NULL, NULL), COMMAND_ERROR_INVALID_LENGTH, "null header DTO is invalid");
    request_header.payload_length = 1U;
    expect_u32(command_protocol_validate_request(&request_header, NULL), COMMAND_ERROR_INVALID_LENGTH,
               "null nonempty payload is invalid");
    expect_u32(command_protocol_validate_request(&request_header, (const uint8_t[]){0U}),
               COMMAND_ERROR_INVALID_LENGTH, "PING with payload is invalid");
    request_header.payload_length = COMMAND_MAX_PAYLOAD_SIZE + 1U;
    expect_u32(command_protocol_validate_request(&request_header, (const uint8_t[]){0U}),
               COMMAND_ERROR_INVALID_LENGTH, "payload above protocol limit is invalid");
    request_header.payload_length = 0U;
    request_header.version = 2U;
    expect_u32(command_protocol_validate_request(&request_header, NULL), COMMAND_ERROR_UNSUPPORTED_VERSION,
               "unsupported protocol version is rejected");
    request_header.version = COMMAND_PROTOCOL_VERSION;
    request_header.type = COMMAND_MESSAGE_TYPE_PONG;
    expect_u32(command_protocol_validate_request(&request_header, NULL), COMMAND_ERROR_UNEXPECTED_TYPE,
               "server message type is not a request");
    request_header.type = COMMAND_MESSAGE_TYPE_COMMAND;
    request_header.sequence = 0U;
    request_header.payload_length = 1U;
    expect_u32(command_protocol_validate_request(&request_header, (const uint8_t[]){COMMAND_CODE_STOP}),
               COMMAND_ERROR_INVALID_SEQUENCE, "command sequence zero is rejected");
    request_header.sequence = 1U;
    request_header.payload_length = 0U;
    expect_u32(command_protocol_validate_request(&request_header, NULL), COMMAND_ERROR_INVALID_LENGTH,
               "empty command is rejected");
    request_header.payload_length = 1U;
    const uint8_t known_commands[] = {
        COMMAND_CODE_HOME,
        COMMAND_CODE_MOTION_OPERATION,
        COMMAND_CODE_SET_TEMPERATURE,
        COMMAND_CODE_SET_OUTPUT,
        COMMAND_CODE_CHANGE_TOOL,
        COMMAND_CODE_STOP,
    };
    for (size_t index = 0U; index < sizeof(known_commands); ++index) {
        expect_u32(
            command_protocol_validate_request(&request_header, &known_commands[index]),
            COMMAND_ERROR_INVALID_STATE,
            "recognized command is left for the decoder"
        );
    }
    expect_u32(command_protocol_validate_request(&request_header, (const uint8_t[]){0xFEU}),
               COMMAND_ERROR_UNKNOWN_COMMAND, "unknown command code is rejected");
}

static void test_frame_and_response_builders(void) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    const uint8_t payload[] = {1U, 2U, 3U};
    expect_u32(
        (uint32_t)command_protocol_build_frame(NULL, sizeof(frame), COMMAND_MESSAGE_TYPE_COMMAND, 1U, payload, 3U),
        0U,
        "null output buffer is rejected"
    );
    expect_u32(
        (uint32_t)command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_COMMAND, 1U, NULL, 1U),
        0U,
        "null nonempty payload is rejected"
    );
    expect_u32(
        (uint32_t)command_protocol_build_frame(
            frame,
            sizeof(frame),
            COMMAND_MESSAGE_TYPE_COMMAND,
            1U,
            payload,
            COMMAND_MAX_PAYLOAD_SIZE + 1U
        ),
        0U,
        "payload above maximum is rejected"
    );
    expect_u32(
        (uint32_t)command_protocol_build_frame(
            frame,
            COMMAND_FRAME_HEADER_SIZE + COMMAND_FRAME_CRC_SIZE - 1U,
            COMMAND_MESSAGE_TYPE_PING,
            1U,
            NULL,
            0U
        ),
        0U,
        "insufficient frame capacity is rejected"
    );
    expect_u32(
        (uint32_t)command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_COMMAND, 1U, payload, 3U),
        17U,
        "command frame is built with payload"
    );
    expect_u32(
        (uint32_t)command_protocol_build_frame(frame, sizeof(frame), COMMAND_MESSAGE_TYPE_PING, 1U, NULL, 0U),
        14U,
        "empty-payload frame is built"
    );

    expect_u32((uint32_t)command_protocol_build_response(frame, sizeof(frame), NULL), 0U, "null response DTO is rejected");
    const CommandResponseDTO_t too_large_response = {
        .type = COMMAND_MESSAGE_TYPE_ERROR,
        .sequence = 1U,
        .payload_length = COMMAND_MAX_PAYLOAD_SIZE + 1U,
    };
    expect_u32(
        (uint32_t)command_protocol_build_response(frame, sizeof(frame), &too_large_response),
        0U,
        "response with oversized payload is rejected"
    );

    CommandFrameHeaderDTO_t header = {0};
    const size_t pong_length = command_protocol_build_pong_response(frame, sizeof(frame), 31U);
    expect_u32((uint32_t)pong_length, 14U, "PONG frame size is correct");
    expect_u32(command_protocol_validate_frame(frame, pong_length, &header), COMMAND_FRAME_VALID, "PONG frame CRC is valid");
    expect_u32(header.type, COMMAND_MESSAGE_TYPE_PONG, "PONG type is correct");
    expect_u32(header.sequence, 31U, "PONG sequence is preserved");

    const size_t ack_length = command_protocol_build_ack_response(frame, sizeof(frame), 32U);
    expect_u32((uint32_t)ack_length, 14U, "ACK frame size is correct");
    expect_u32(command_protocol_validate_frame(frame, ack_length, &header), COMMAND_FRAME_VALID, "ACK frame CRC is valid");
    expect_u32(header.type, COMMAND_MESSAGE_TYPE_ACK, "ACK type is correct");

    const size_t error_length =
        command_protocol_build_error_response(frame, sizeof(frame), 35U, COMMAND_ERROR_INVALID_LENGTH);
    expect_u32((uint32_t)error_length, 16U, "ERROR frame size is correct");
    expect_u32(command_protocol_validate_frame(frame, error_length, &header), COMMAND_FRAME_VALID, "ERROR frame CRC is valid");
    expect_u32(header.type, COMMAND_MESSAGE_TYPE_ERROR, "ERROR type is correct");
    expect_u32(header.payload_length, 2U, "ERROR code occupies two bytes");
    expect_u32(header.sequence, 35U, "ERROR sequence is preserved");
    expect_u32(frame[COMMAND_FRAME_HEADER_SIZE], COMMAND_ERROR_INVALID_LENGTH, "ERROR payload is little endian");
}

int main(void) {
    test_crc_and_assembler_buffer_operations();
    test_assembler_frame_events();
    test_header_frame_and_request_validation();
    test_frame_and_response_builders();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command protocol tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command protocol tests passed.\n");
    return 0;
}

#include "command_decoder.h"
#include "command_protocol.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;
static uint8_t test_force_protocol_validation_success;

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

static uint16_t test_command_protocol_validate_request_dto(const CommandRequestDTO_t *request);

#define static
#define command_protocol_validate_request_dto test_command_protocol_validate_request_dto
#include "../APP/protocol/command/command_decoder.c"
#undef command_protocol_validate_request_dto
#undef static

static uint16_t test_command_protocol_validate_request_dto(const CommandRequestDTO_t *request) {
    if (test_force_protocol_validation_success != 0U && request != NULL &&
        request->header.type == COMMAND_MESSAGE_TYPE_COMMAND) {
        return 0U;
    }
    return command_protocol_validate_request_dto(request);
}

static void reset_decoder(void) {
    command_decoder_diagnostics = (CommandDecoderDiagnostics){0};
    test_force_protocol_validation_success = 0U;
}

static CommandRequestDTO_t make_request(uint8_t type, uint32_t sequence, const uint8_t *payload, uint16_t length) {
    CommandRequestDTO_t request = {
        .header = {
            .magic = COMMAND_PROTOCOL_MAGIC,
            .version = COMMAND_PROTOCOL_VERSION,
            .type = type,
            .payload_length = length,
            .sequence = sequence,
        },
    };
    if (payload != NULL && length != 0U) {
        memcpy(request.payload, payload, length);
    }
    return request;
}

static void write_u32_le(uint8_t *destination, uint32_t value) {
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static void write_u16_le(uint8_t *destination, uint16_t value) {
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void test_ping_and_protocol_errors(void) {
    reset_decoder();
    CommandRequestDTO_t request = make_request(COMMAND_MESSAGE_TYPE_PING, 101U, NULL, 0U);
    CommandDecoderResult_t result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_RESPONSE_READY, "PING returns response result");
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_PONG, "PING maps to PONG");
    expect_u32(result.error_code, 0U, "PONG has no error code");

    request.header.version = 2U;
    result = command_decoder_decode_request(&request);
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_ERROR, "unsupported version returns ERROR");
    expect_u32(result.error_code, COMMAND_ERROR_UNSUPPORTED_VERSION, "unsupported version error is preserved");

    request = make_request(COMMAND_MESSAGE_TYPE_PONG, 102U, NULL, 0U);
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_UNEXPECTED_TYPE, "response message type is rejected as request");

    request = make_request(COMMAND_MESSAGE_TYPE_PING, 103U, (const uint8_t[]){1U}, 1U);
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_INVALID_LENGTH, "PING payload is rejected");

    request = make_request(COMMAND_MESSAGE_TYPE_COMMAND, 0U, (const uint8_t[]){COMMAND_CODE_STOP}, 1U);
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_INVALID_SEQUENCE, "zero command sequence is rejected");

    request = make_request(COMMAND_MESSAGE_TYPE_COMMAND, 104U, (const uint8_t[]){0xFEU}, 1U);
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_UNKNOWN_COMMAND, "unknown command returns ERROR");

    request = make_request(COMMAND_MESSAGE_TYPE_COMMAND, 105U, NULL, 0U);
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_INVALID_LENGTH, "empty command returns ERROR");

    request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        106U,
        (const uint8_t[]){COMMAND_CODE_STOP},
        1U
    );
    test_force_protocol_validation_success = 1U;
    result = command_decoder_decode_request(&request);
    test_force_protocol_validation_success = 0U;
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "valid request can proceed from a zero protocol error");

    result = command_decoder_decode_request(NULL);
    expect_u32(result.type, COMMAND_DECODER_RESULT_INVALID_ARGUMENT, "null request is an internal invalid argument");
    expect_u32((uint32_t)command_decoder_diagnostics.last_error, COMMAND_FRAME_INVALID_ARGUMENT, "null request is diagnosed");
}

static void test_home_command_validation_and_decode(void) {
    reset_decoder();
    CommandRequestDTO_t request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        201U,
        (const uint8_t[]){COMMAND_CODE_HOME, 0x07U},
        2U
    );
    CommandDecoderResult_t result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "HOME produces a decoded command");
    expect_u32(result.command.sequence, 201U, "HOME preserves sequence");
    expect_u32(result.command.code, COMMAND_CODE_HOME, "HOME preserves command code");
    expect_u32(result.command.parameters.home.axes, 7U, "HOME decodes all axis bits");

    request = make_request(COMMAND_MESSAGE_TYPE_COMMAND, 202U, (const uint8_t[]){COMMAND_CODE_HOME}, 1U);
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_INVALID_LENGTH, "HOME requires an axis byte");

    request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        203U,
        (const uint8_t[]){COMMAND_CODE_HOME, 0U},
        2U
    );
    result = command_decoder_decode_request(&request);
    expect_u32(result.error_code, COMMAND_ERROR_INVALID_PARAMETER, "HOME rejects an empty axis mask");

    request.payload[1] = 0x08U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "HOME rejects reserved axis bits"
    );
}

static CommandRequestDTO_t make_motion_request(uint32_t sequence) {
    uint8_t payload[30] = {0};
    payload[0] = COMMAND_CODE_MOTION_OPERATION;
    payload[1] = 0x03U;
    write_u32_le(&payload[2], (uint32_t)-12);
    write_u32_le(&payload[6], 0x12345678U);
    write_u32_le(&payload[10], 0x80000000U);
    write_u32_le(&payload[14], 1000U);
    write_u32_le(&payload[18], (uint32_t)-5);
    write_u32_le(&payload[22], 200U);
    write_u16_le(&payload[26], 1000U);
    write_u16_le(&payload[28], 9999U);
    return make_request(COMMAND_MESSAGE_TYPE_COMMAND, sequence, payload, sizeof(payload));
}

static void test_motion_operation_validation_and_decode(void) {
    reset_decoder();
    CommandRequestDTO_t request = make_motion_request(301U);
    CommandDecoderResult_t result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "motion operation decodes");
    expect_u32(result.command.sequence, 301U, "motion operation preserves sequence");
    expect_u32(result.command.code, COMMAND_CODE_MOTION_OPERATION, "motion operation preserves code");
    expect_u32(result.command.parameters.motion_operation.operation_flags, 3U, "motion flags decode");
    expect_u32((uint32_t)result.command.parameters.motion_operation.x, (uint32_t)-12, "negative X decodes");
    expect_u32((uint32_t)result.command.parameters.motion_operation.y, 0x12345678U, "positive Y decodes");
    expect_u32((uint32_t)result.command.parameters.motion_operation.z, 0x80000000U, "INT32_MIN decodes");
    expect_u32(result.command.parameters.motion_operation.speed, 1000U, "linear speed decodes");
    expect_u32((uint32_t)result.command.parameters.motion_operation.e_delta, (uint32_t)-5, "negative E delta decodes");
    expect_u32(result.command.parameters.motion_operation.e_speed, 200U, "E speed decodes");
    expect_u32(result.command.parameters.motion_operation.spindle_pwm, 1000U, "spindle PWM decodes");
    expect_u32(result.command.parameters.motion_operation.laser_pwm, 9999U, "laser PWM decodes");

    request.header.payload_length = 29U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_LENGTH,
        "motion operation requires thirty payload bytes"
    );

    request = make_motion_request(302U);
    request.payload[1] = 0U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "motion operation rejects an empty flag set"
    );

    request.payload[1] = 0x10U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "motion operation rejects reserved flags"
    );

    request = make_motion_request(303U);
    write_u32_le(&request.payload[14], 0U);
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "requested linear motion requires nonzero speed"
    );

    request = make_motion_request(304U);
    write_u32_le(&request.payload[22], 0U);
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "requested extruder motion requires nonzero speed"
    );

    request = make_motion_request(305U);
    write_u16_le(&request.payload[26], 10001U);
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "spindle PWM above its limit is rejected"
    );

    request = make_motion_request(306U);
    write_u16_le(&request.payload[28], 10001U);
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "laser PWM above its limit is rejected"
    );

    request = make_motion_request(307U);
    request.payload[1] = 0x04U;
    write_u32_le(&request.payload[14], 0U);
    write_u32_le(&request.payload[22], 0U);
    expect_u32(
        command_decoder_validate_command(&request),
        0U,
        "unused motion axes do not require speed values"
    );
}

static void test_temperature_output_tool_and_stop_commands(void) {
    reset_decoder();
    CommandRequestDTO_t request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        401U,
        (const uint8_t[]){COMMAND_CODE_SET_TEMPERATURE, 2U, 0x34U, 0x12U},
        4U
    );
    CommandDecoderResult_t result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "temperature command decodes");
    expect_u32(result.command.parameters.set_temperature.heater_id, 2U, "heater id decodes");
    expect_u32(result.command.parameters.set_temperature.temperature, 0x1234U, "temperature decodes little endian");
    request.header.payload_length = 3U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_LENGTH,
        "temperature command rejects wrong payload size"
    );

    request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        402U,
        (const uint8_t[]){COMMAND_CODE_SET_OUTPUT, 3U, 1U},
        3U
    );
    result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "output command decodes");
    expect_u32(result.command.parameters.set_output.output_id, 3U, "output id decodes");
    expect_u32(result.command.parameters.set_output.state, 1U, "output state decodes");
    request.payload[2] = 2U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_PARAMETER,
        "output state above one is rejected"
    );
    request.header.payload_length = 2U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_LENGTH,
        "output command rejects wrong payload size"
    );

    request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        403U,
        (const uint8_t[]){COMMAND_CODE_CHANGE_TOOL, 5U},
        2U
    );
    result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "tool change decodes");
    expect_u32(result.command.parameters.change_tool.tool_id, 5U, "tool id decodes");
    request.header.payload_length = 1U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_LENGTH,
        "tool change rejects wrong payload size"
    );

    request = make_request(
        COMMAND_MESSAGE_TYPE_COMMAND,
        404U,
        (const uint8_t[]){COMMAND_CODE_STOP},
        1U
    );
    result = command_decoder_decode_request(&request);
    expect_u32(result.type, COMMAND_DECODER_RESULT_COMMAND_READY, "STOP decodes");
    expect_u32(result.command.code, COMMAND_CODE_STOP, "STOP code is preserved");
    request.header.payload_length = 2U;
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_INVALID_LENGTH,
        "STOP rejects an extra payload byte"
    );

    request = make_request(COMMAND_MESSAGE_TYPE_COMMAND, 405U, (const uint8_t[]){0xFEU}, 1U);
    expect_u32(
        command_decoder_validate_command(&request),
        COMMAND_ERROR_UNKNOWN_COMMAND,
        "decoder rejects unknown command identifier"
    );
    (void)command_decoder_decode_command(&request);
}

static void test_decoder_diagnostics_and_error_mapping(void) {
    reset_decoder();
    command_decoder_record_request_error(COMMAND_ERROR_UNSUPPORTED_VERSION);
    command_decoder_record_request_error(COMMAND_ERROR_UNEXPECTED_TYPE);
    command_decoder_record_request_error(COMMAND_ERROR_INVALID_LENGTH);
    command_decoder_record_request_error(COMMAND_ERROR_INVALID_SEQUENCE);
    command_decoder_record_request_error(COMMAND_ERROR_UNKNOWN_COMMAND);
    command_decoder_record_request_error(COMMAND_ERROR_INVALID_STATE);
    command_decoder_record_request_error(UINT16_MAX);

    CommandRequestDTO_t request = make_request(COMMAND_MESSAGE_TYPE_PING, 501U, NULL, 0U);
    command_decoder_diagnostics.requests_received = UINT32_MAX;
    command_decoder_diagnostics.pings_received = UINT32_MAX;
    (void)command_decoder_decode_request(&request);

    CommandDecoderDiagnostics diagnostics = {0};
    command_decoder_get_diagnostics(NULL);
    command_decoder_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.protocol_version, COMMAND_PROTOCOL_VERSION, "diagnostics report protocol version");
    expect_u32(diagnostics.max_payload_size, COMMAND_MAX_PAYLOAD_SIZE, "diagnostics report maximum payload");
    expect_u32(diagnostics.max_frame_size, COMMAND_MAX_FRAME_SIZE, "diagnostics report maximum frame");
    expect_u32(diagnostics.requests_received, UINT32_MAX, "request counter saturates");
    expect_u32(diagnostics.request_queue_overflows, 0U, "removed request queue reports zero");
    expect_u32(diagnostics.request_bytes_dropped, 0U, "removed request buffer reports zero");
    expect_u32(diagnostics.commands_received, 0U, "command count is copied");
    expect_u32(diagnostics.commands_rejected, 0U, "rejected command count is copied");
    expect_u32(diagnostics.pings_received, UINT32_MAX, "ping counter saturates");
    expect_u32(diagnostics.responses_formed, 1U, "formed response count is copied");
    expect_u32(diagnostics.pongs_formed, 1U, "formed PONG count is copied");
    expect_u32(diagnostics.errors_formed, 0U, "formed ERROR count is copied");
    expect_u32(diagnostics.invalid_payload_length, 1U, "invalid length count is mapped");
    expect_u32(diagnostics.unsupported_version, 1U, "version error count is mapped");
    expect_u32(diagnostics.unexpected_type, 1U, "type error count is mapped");
    expect_u32(diagnostics.invalid_sequence, 1U, "sequence error count is mapped");
    expect_u32(diagnostics.unknown_commands, 1U, "unknown command count is mapped");
    expect_u32(diagnostics.invalid_states, 1U, "invalid state count is mapped");
    expect_u32(diagnostics.response_queue_overflows, 0U, "removed response queue reports zero");
    expect_u32(diagnostics.last_error, UINT16_MAX, "last error stores unknown error code");
}

int main(void) {
    test_ping_and_protocol_errors();
    test_home_command_validation_and_decode();
    test_motion_operation_validation_and_decode();
    test_temperature_output_tool_and_stop_commands();
    test_decoder_diagnostics_and_error_mapping();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command decoder unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command decoder unit tests passed.\n");
    return 0;
}

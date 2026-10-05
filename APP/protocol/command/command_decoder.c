#include "command_decoder.h"

#include <stddef.h>

static volatile CommandDecoderDiagnostics command_decoder_diagnostics;

static void command_decoder_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void command_decoder_record_error(int32_t error) { command_decoder_diagnostics.last_error = error; }

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

static uint16_t command_decoder_read_u16_le(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t command_decoder_read_u32_le(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static int32_t command_decoder_read_i32_le(const uint8_t *data) {
    const uint32_t value = command_decoder_read_u32_le(data);
    if (value <= INT32_MAX) {
        return (int32_t)value;
    }
    return -(int32_t)(UINT32_MAX - value) - 1;
}

static uint16_t command_decoder_validate_command(const CommandRequestDTO_t *request) {
    const uint8_t *payload = request->payload;
    const uint16_t payload_length = request->header.payload_length;

    switch (payload[0]) {
    case COMMAND_CODE_HOME:
        if (payload_length != 2U) {
            return COMMAND_ERROR_INVALID_LENGTH;
        }
        return payload[1] != 0U && (payload[1] & 0xF8U) == 0U ? 0U : COMMAND_ERROR_INVALID_PARAMETER;

    case COMMAND_CODE_MOTION_OPERATION: {
        if (payload_length != 30U) {
            return COMMAND_ERROR_INVALID_LENGTH;
        }
        const uint8_t operation_flags = payload[1];
        if (operation_flags == 0U || (operation_flags & 0xF0U) != 0U) {
            return COMMAND_ERROR_INVALID_PARAMETER;
        }
        const uint32_t speed = command_decoder_read_u32_le(&payload[14]);
        const uint32_t e_speed = command_decoder_read_u32_le(&payload[22]);
        const uint16_t spindle_pwm = command_decoder_read_u16_le(&payload[26]);
        const uint16_t laser_pwm = command_decoder_read_u16_le(&payload[28]);
        if (((operation_flags & COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE) != 0U && speed == 0U) ||
            ((operation_flags & 0x02U) != 0U && e_speed == 0U) || spindle_pwm > 10000U || laser_pwm > 10000U) {
            return COMMAND_ERROR_INVALID_PARAMETER;
        }
        return 0U;
    }

    case COMMAND_CODE_SET_TEMPERATURE:
        return payload_length == 4U ? 0U : COMMAND_ERROR_INVALID_LENGTH;

    case COMMAND_CODE_SET_OUTPUT:
        if (payload_length != 3U) {
            return COMMAND_ERROR_INVALID_LENGTH;
        }
        return payload[2] <= 1U ? 0U : COMMAND_ERROR_INVALID_PARAMETER;

    case COMMAND_CODE_CHANGE_TOOL:
        return payload_length == 2U ? 0U : COMMAND_ERROR_INVALID_LENGTH;

    case COMMAND_CODE_STOP:
        return payload_length == 1U ? 0U : COMMAND_ERROR_INVALID_LENGTH;

    default:
        return COMMAND_ERROR_UNKNOWN_COMMAND;
    }
}

static CommandDTO_t command_decoder_decode_command(const CommandRequestDTO_t *request) {
    const uint8_t *payload = request->payload;
    CommandDTO_t command = {
        .sequence = request->header.sequence,
        .code = (CommandCode_t)payload[0],
    };

    switch (command.code) {
    case COMMAND_CODE_HOME:
        command.parameters.home.axes = payload[1];
        break;

    case COMMAND_CODE_MOTION_OPERATION:
        command.parameters.motion_operation.operation_flags = payload[1];
        command.parameters.motion_operation.x = command_decoder_read_i32_le(&payload[2]);
        command.parameters.motion_operation.y = command_decoder_read_i32_le(&payload[6]);
        command.parameters.motion_operation.z = command_decoder_read_i32_le(&payload[10]);
        command.parameters.motion_operation.speed = command_decoder_read_u32_le(&payload[14]);
        command.parameters.motion_operation.e_delta = command_decoder_read_i32_le(&payload[18]);
        command.parameters.motion_operation.e_speed = command_decoder_read_u32_le(&payload[22]);
        command.parameters.motion_operation.spindle_pwm = command_decoder_read_u16_le(&payload[26]);
        command.parameters.motion_operation.laser_pwm = command_decoder_read_u16_le(&payload[28]);
        break;

    case COMMAND_CODE_SET_TEMPERATURE:
        command.parameters.set_temperature.heater_id = payload[1];
        command.parameters.set_temperature.temperature = command_decoder_read_u16_le(&payload[2]);
        break;

    case COMMAND_CODE_SET_OUTPUT:
        command.parameters.set_output.output_id = payload[1];
        command.parameters.set_output.state = payload[2];
        break;

    case COMMAND_CODE_CHANGE_TOOL:
        command.parameters.change_tool.tool_id = payload[1];
        break;

    case COMMAND_CODE_STOP:
        break;

    default:
        break;
    }

    return command;
}

CommandDecoderResult_t command_decoder_decode_request(const CommandRequestDTO_t *request) {
    CommandDecoderResult_t result = {
        .type = COMMAND_DECODER_RESULT_INVALID_ARGUMENT,
    };
    if (request == NULL) {
        command_decoder_record_error(COMMAND_FRAME_INVALID_ARGUMENT);
        return result;
    }

    const CommandFrameHeaderDTO_t *header = &request->header;
    command_decoder_counter_increment(&command_decoder_diagnostics.requests_received);
    if (header->type == COMMAND_MESSAGE_TYPE_COMMAND) {
        command_decoder_counter_increment(&command_decoder_diagnostics.commands_received);
    } else if (header->type == COMMAND_MESSAGE_TYPE_PING) {
        command_decoder_counter_increment(&command_decoder_diagnostics.pings_received);
    }

    uint16_t error_code = command_protocol_validate_request_dto(request);
    if (header->type == COMMAND_MESSAGE_TYPE_COMMAND &&
        (error_code == 0U || error_code == COMMAND_ERROR_INVALID_STATE)) {
        const uint16_t command_error = command_decoder_validate_command(request);
        if (command_error == 0U) {
            result.type = COMMAND_DECODER_RESULT_COMMAND_READY;
            result.command = command_decoder_decode_command(request);
            return result;
        }
        error_code = command_error;
    }
    if (header->type == COMMAND_MESSAGE_TYPE_COMMAND) {
        command_decoder_counter_increment(&command_decoder_diagnostics.commands_rejected);
    }
    if (error_code != 0U) {
        command_decoder_record_request_error(error_code);
        result.type = COMMAND_DECODER_RESULT_RESPONSE_READY;
        result.response_type = COMMAND_MESSAGE_TYPE_ERROR;
        result.error_code = error_code;
        command_decoder_counter_increment(&command_decoder_diagnostics.errors_formed);
        command_decoder_counter_increment(&command_decoder_diagnostics.responses_formed);
        return result;
    }

    result.type = COMMAND_DECODER_RESULT_RESPONSE_READY;
    result.response_type = COMMAND_MESSAGE_TYPE_PONG;
    result.error_code = 0U;
    command_decoder_counter_increment(&command_decoder_diagnostics.pongs_formed);
    command_decoder_counter_increment(&command_decoder_diagnostics.responses_formed);
    return result;
}

void command_decoder_get_diagnostics(CommandDecoderDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->protocol_version = COMMAND_PROTOCOL_VERSION;
    diagnostics->max_payload_size = COMMAND_MAX_PAYLOAD_SIZE;
    diagnostics->max_frame_size = COMMAND_MAX_FRAME_SIZE;
    diagnostics->requests_received = command_decoder_diagnostics.requests_received;
    diagnostics->request_queue_overflows = 0U;
    diagnostics->request_bytes_dropped = 0U;
    diagnostics->commands_received = command_decoder_diagnostics.commands_received;
    diagnostics->commands_rejected = command_decoder_diagnostics.commands_rejected;
    diagnostics->pings_received = command_decoder_diagnostics.pings_received;
    diagnostics->responses_formed = command_decoder_diagnostics.responses_formed;
    diagnostics->pongs_formed = command_decoder_diagnostics.pongs_formed;
    diagnostics->errors_formed = command_decoder_diagnostics.errors_formed;
    diagnostics->invalid_payload_length = command_decoder_diagnostics.invalid_payload_length;
    diagnostics->unsupported_version = command_decoder_diagnostics.unsupported_version;
    diagnostics->unexpected_type = command_decoder_diagnostics.unexpected_type;
    diagnostics->invalid_sequence = command_decoder_diagnostics.invalid_sequence;
    diagnostics->unknown_commands = command_decoder_diagnostics.unknown_commands;
    diagnostics->invalid_states = command_decoder_diagnostics.invalid_states;
    diagnostics->response_queue_overflows = 0U;
    diagnostics->last_error = command_decoder_diagnostics.last_error;
}

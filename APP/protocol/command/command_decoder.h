#ifndef FM_V3_COMMAND_DECODER_H
#define FM_V3_COMMAND_DECODER_H

#include "command_protocol.h"

#include <stdint.h>

#define COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE 0x01U

typedef struct {
    uint8_t axes;
} CommandHomeParameters_t;

typedef struct {
    uint8_t operation_flags;
    int32_t x;
    int32_t y;
    int32_t z;
    uint32_t speed;
    int32_t e_delta;
    uint32_t e_speed;
    uint16_t spindle_pwm;
    uint16_t laser_pwm;
} CommandMotionOperationParameters_t;

typedef struct {
    uint8_t heater_id;
    uint16_t temperature;
} CommandSetTemperatureParameters_t;

typedef struct {
    uint8_t output_id;
    uint8_t state;
} CommandSetOutputParameters_t;

typedef struct {
    uint8_t tool_id;
} CommandChangeToolParameters_t;

typedef struct {
    uint32_t sequence;
    CommandCode_t code;
    union {
        CommandHomeParameters_t home;
        CommandMotionOperationParameters_t motion_operation;
        CommandSetTemperatureParameters_t set_temperature;
        CommandSetOutputParameters_t set_output;
        CommandChangeToolParameters_t change_tool;
    } parameters;
} CommandDTO_t;

typedef enum {
    COMMAND_DECODER_RESULT_INVALID_ARGUMENT = 0,
    COMMAND_DECODER_RESULT_COMMAND_READY,
    COMMAND_DECODER_RESULT_RESPONSE_READY
} CommandDecoderResultType_t;

typedef struct {
    uint32_t protocol_version;
    uint32_t max_payload_size;
    uint32_t max_frame_size;
    uint32_t requests_received;
    uint32_t request_queue_overflows; /* Reserved for diagnostic wire compatibility. */
    uint32_t request_bytes_dropped;   /* Reserved for diagnostic wire compatibility. */
    uint32_t commands_received;
    uint32_t commands_rejected;
    uint32_t pings_received;
    uint32_t responses_formed;
    uint32_t pongs_formed;
    uint32_t errors_formed;
    uint32_t invalid_payload_length;
    uint32_t unsupported_version;
    uint32_t unexpected_type;
    uint32_t invalid_sequence;
    uint32_t unknown_commands;
    uint32_t invalid_states;
    uint32_t response_queue_overflows; /* Reserved for diagnostic wire compatibility. */
    int32_t last_error;
} CommandDecoderDiagnostics;

typedef struct {
    CommandDecoderResultType_t type;
    CommandDTO_t command;
    uint8_t response_type;
    uint16_t error_code;
} CommandDecoderResult_t;

CommandDecoderResult_t command_decoder_decode_request(const CommandRequestDTO_t *request);
void command_decoder_get_diagnostics(CommandDecoderDiagnostics *diagnostics);

#endif /* FM_V3_COMMAND_DECODER_H */

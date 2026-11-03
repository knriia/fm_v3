#ifndef FM_V3_COMMAND_DECODER_H
#define FM_V3_COMMAND_DECODER_H

#include "command_protocol.h"

#include <stdint.h>

#define COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE 0x01U

typedef struct {
    uint8_t axes;
} CommandHomeParameters_t;

typedef struct {
    uint8_t axes;
} CommandCalibrateParameters_t;

/* Relative X move; travel and speed limits come from the calibration and are checked by the planner. */
typedef struct {
    int32_t delta_um;
    uint32_t speed_um_per_s;
} CommandXAxisJogParameters_t;

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
        CommandCalibrateParameters_t calibrate;
        CommandXAxisJogParameters_t x_jog;
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
    uint32_t commands_received;
    uint32_t commands_rejected;
    /* Error code of the last rejected request; the client gets the same code in its ERROR response. */
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

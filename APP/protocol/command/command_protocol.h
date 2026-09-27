#ifndef FM_V3_COMMAND_PROTOCOL_H
#define FM_V3_COMMAND_PROTOCOL_H

#include "command_transport_dto.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t buffer[COMMAND_MAX_FRAME_SIZE];
    size_t length;
    size_t max_length;
} CommandFrameParser_t;

typedef enum {
    COMMAND_FRAME_VALID = 0,
    COMMAND_FRAME_INVALID_ARGUMENT,
    COMMAND_FRAME_INVALID_LENGTH,
    COMMAND_FRAME_INVALID_MAGIC,
    COMMAND_FRAME_INVALID_CRC
} CommandFrameValidationResult_t;

typedef void (*CommandFrameCallback)(const uint8_t *frame, size_t frame_length, void *context);
typedef void (*CommandFrameErrorCallback)(CommandFrameValidationResult_t result, void *context);

void command_frame_parser_init(CommandFrameParser_t *parser);

void command_frame_parser_feed_ex(
    CommandFrameParser_t *parser,
    const uint8_t *data,
    size_t data_length,
    CommandFrameCallback callback,
    CommandFrameErrorCallback error_callback,
    void *context
);

void command_frame_parser_feed(
    CommandFrameParser_t *parser,
    const uint8_t *data,
    size_t data_length,
    CommandFrameCallback callback,
    void *context
);

uint32_t command_protocol_crc32(const uint8_t *data, size_t data_length);

CommandFrameValidationResult_t
command_protocol_validate_frame(const uint8_t *frame, size_t frame_length, CommandFrameHeaderDTO_t *header);

CommandFrameValidationResult_t
command_protocol_decode_request(const uint8_t *frame, size_t frame_length, CommandRequestDTO_t *request);

uint16_t command_protocol_validate_request(const CommandFrameHeaderDTO_t *header, const uint8_t *payload);
uint16_t command_protocol_validate_request_dto(const CommandRequestDTO_t *request);

size_t command_protocol_build_frame(
    uint8_t *frame,
    size_t frame_capacity,
    uint8_t type,
    uint32_t sequence,
    const uint8_t *payload,
    uint16_t payload_length
);

size_t command_protocol_build_response(uint8_t *frame, size_t frame_capacity, const CommandResponseDTO_t *response);

size_t command_protocol_build_pong_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence);
size_t
command_protocol_build_error_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence, uint16_t error_code);

#endif /* FM_V3_COMMAND_PROTOCOL_H */

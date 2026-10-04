#ifndef FM_V3_COMMAND_PROTOCOL_H
#define FM_V3_COMMAND_PROTOCOL_H

#include "command_transport_dto.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t buffer[COMMAND_MAX_FRAME_SIZE];
    size_t length;
    size_t max_length;
} CommandFrameAssembler_t;

typedef enum {
    COMMAND_FRAME_VALID = 0,
    COMMAND_FRAME_INVALID_ARGUMENT,
    COMMAND_FRAME_INVALID_LENGTH,
    COMMAND_FRAME_INVALID_MAGIC,
    COMMAND_FRAME_INVALID_CRC
} CommandFrameValidationResult_t;

typedef enum {
    COMMAND_FRAME_ASSEMBLER_NEED_MORE = 0,
    COMMAND_FRAME_ASSEMBLER_COMMAND_READY,
    COMMAND_FRAME_ASSEMBLER_INVALID_FRAME,
    COMMAND_FRAME_ASSEMBLER_INVALID_ARGUMENT
} CommandFrameAssemblerResult_t;

typedef struct {
    CommandFrameAssemblerResult_t result;
    const uint8_t *frame;
    size_t frame_length;
    size_t consume_length;
    CommandFrameValidationResult_t validation;
    CommandRequestDTO_t command;
} CommandFrameAssemblerEvent_t;

void command_frame_assembler_init(CommandFrameAssembler_t *assembler);

/* Appends as many bytes as fit and returns the number consumed. */
size_t command_frame_assembler_append(CommandFrameAssembler_t *assembler, const uint8_t *data, size_t data_length);

/* Returns the next complete command; its frame view stays valid until consume(). */
CommandFrameAssemblerEvent_t command_frame_assembler_next(const CommandFrameAssembler_t *assembler);

uint8_t command_frame_assembler_consume(CommandFrameAssembler_t *assembler, size_t length);

uint32_t command_protocol_crc32(const uint8_t *data, size_t data_length);

CommandFrameValidationResult_t
command_protocol_validate_frame(const uint8_t *frame, size_t frame_length, CommandFrameHeaderDTO_t *header);

/* Extracts all header fields when available, even when the return value reports bad magic or length. */
CommandFrameValidationResult_t
command_protocol_parse_header(const uint8_t *frame, size_t available_length, CommandFrameHeaderDTO_t *header);

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
size_t command_protocol_build_ack_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence);
size_t
command_protocol_build_error_response(uint8_t *frame, size_t frame_capacity, uint32_t sequence, uint16_t error_code);

#endif /* FM_V3_COMMAND_PROTOCOL_H */

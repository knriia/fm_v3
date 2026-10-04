#ifndef FM_V3_COMMAND_TRANSPORT_DTO_H
#define FM_V3_COMMAND_TRANSPORT_DTO_H

#include <stdint.h>

#define COMMAND_PROTOCOL_MAGIC 0xA55AU
#define COMMAND_PROTOCOL_VERSION 1U
#define COMMAND_MAX_PAYLOAD_SIZE 256U
#define COMMAND_FRAME_HEADER_SIZE 10U
#define COMMAND_FRAME_CRC_SIZE 4U
#define COMMAND_MAX_FRAME_SIZE (COMMAND_FRAME_HEADER_SIZE + COMMAND_MAX_PAYLOAD_SIZE + COMMAND_FRAME_CRC_SIZE)

#define COMMAND_MESSAGE_TYPE_COMMAND 0x01U
#define COMMAND_MESSAGE_TYPE_ACK 0x02U
#define COMMAND_MESSAGE_TYPE_DONE 0x03U
#define COMMAND_MESSAGE_TYPE_DUPLICATE 0x04U
#define COMMAND_MESSAGE_TYPE_PING 0x05U
#define COMMAND_MESSAGE_TYPE_PONG 0x06U
#define COMMAND_MESSAGE_TYPE_CANCELLED 0x07U
#define COMMAND_MESSAGE_TYPE_ERROR 0xFFU
/* Command identifiers encoded in a command request payload. */
typedef enum {
    COMMAND_CODE_HOME = 0x01U,
    COMMAND_CODE_MOTION_OPERATION = 0x02U,
    COMMAND_CODE_SET_TEMPERATURE = 0x07U,
    COMMAND_CODE_SET_OUTPUT = 0x08U,
    COMMAND_CODE_CHANGE_TOOL = 0x09U,
    COMMAND_CODE_STOP = 0x0AU
} CommandCode_t;

#define COMMAND_ERROR_UNSUPPORTED_VERSION 0x0001U
#define COMMAND_ERROR_UNEXPECTED_TYPE 0x0002U
#define COMMAND_ERROR_INVALID_LENGTH 0x0003U
#define COMMAND_ERROR_UNKNOWN_COMMAND 0x0004U
#define COMMAND_ERROR_INVALID_PARAMETER 0x0005U
#define COMMAND_ERROR_INVALID_FRAME_FORMAT 0x0006U
#define COMMAND_ERROR_INVALID_FRAME_CRC 0x0007U
#define COMMAND_ERROR_TRUNCATED_FRAME 0x0008U
#define COMMAND_ERROR_INVALID_STATE 0x0009U
#define COMMAND_ERROR_DECODER_QUEUE_FULL 0x000AU
#define COMMAND_ERROR_INVALID_SEQUENCE 0x000DU

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_length;
    uint32_t sequence;
} CommandFrameHeaderDTO_t;

typedef struct __attribute__((packed)) {
    uint16_t error_code;
} CommandErrorPayloadDTO_t;

/* Complete, transport-valid request copied out of the protocol parser buffer. */
typedef struct {
    CommandFrameHeaderDTO_t header;
    uint8_t payload[COMMAND_MAX_PAYLOAD_SIZE];
} CommandRequestDTO_t;

/* Response model owned by the caller until it is serialized into a frame. */
typedef struct {
    uint8_t type;
    uint32_t sequence;
    uint16_t payload_length;
    uint8_t payload[COMMAND_MAX_PAYLOAD_SIZE];
} CommandResponseDTO_t;

_Static_assert(sizeof(CommandFrameHeaderDTO_t) == COMMAND_FRAME_HEADER_SIZE, "Invalid command frame header size");
_Static_assert(sizeof(CommandErrorPayloadDTO_t) == 2U, "Invalid command error payload size");

#endif /* FM_V3_COMMAND_TRANSPORT_DTO_H */

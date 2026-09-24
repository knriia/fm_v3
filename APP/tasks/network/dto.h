#ifndef FM_V3_DTO_H
#define FM_V3_DTO_H

#include <stdint.h>

/* All multi-byte fields use the MCU's little-endian representation. */
#define NETWORK_PROTOCOL_MAGIC 0x4447U
#define NETWORK_PROTOCOL_VERSION 1U
#define NETWORK_MESSAGE_TYPE_DIAGNOSTIC_SNAPSHOT 1U
#define NETWORK_MESSAGE_TYPE_TELEMETRY 2U

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t message_type;
    uint16_t header_length;
    uint16_t payload_length;
} NetworkFrameHeaderDTO_t;

_Static_assert(sizeof(NetworkFrameHeaderDTO_t) == 8U, "Invalid network frame header size");

#endif /* FM_V3_DTO_H */

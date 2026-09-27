#ifndef FM_V3_COMMAND_TASK_DTO_H
#define FM_V3_COMMAND_TASK_DTO_H

#include "command_transport_dto.h"

#include <stdint.h>

/* Complete request passed from the TCP task to the command decoder task. */
typedef struct {
    uint32_t connection_id;
    CommandRequestDTO_t request;
} CommandTaskRequestDTO_t;

/* Serialized response passed from the command decoder task back to the TCP task. */
typedef struct {
    uint32_t connection_id;
    uint16_t length;
    uint8_t response_type;
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
} CommandTaskResponseDTO_t;

#endif /* FM_V3_COMMAND_TASK_DTO_H */
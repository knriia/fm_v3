#ifndef FM_V3_COMMAND_DECODER_TASK_H
#define FM_V3_COMMAND_DECODER_TASK_H

#include "command_protocol.h"
#include "command_task_dto.h"

#include <stdint.h>

#define COMMAND_DECODER_TASK_STACK_SIZE_BYTES (1024U * 4U)
#define COMMAND_DECODER_REQUEST_QUEUE_LENGTH 8U
#define COMMAND_DECODER_RESPONSE_QUEUE_LENGTH 8U

#define COMMAND_DECODER_ERROR_REQUEST_QUEUE_FULL (-1)
#define COMMAND_DECODER_ERROR_RESPONSE_QUEUE_FULL (-2)
#define COMMAND_DECODER_ERROR_RESPONSE_BUILD_FAILED (-3)

typedef struct {
    uint32_t protocol_version;
    uint32_t max_payload_size;
    uint32_t max_frame_size;
    uint32_t requests_received;
    uint32_t request_queue_overflows;
    uint32_t request_bytes_dropped;
    uint32_t commands_received;
    uint32_t commands_rejected;
    uint32_t pings_received;
    uint32_t responses_formed;
    uint32_t pongs_queued;
    uint32_t errors_queued;
    uint32_t invalid_payload_length;
    uint32_t unsupported_version;
    uint32_t unexpected_type;
    uint32_t invalid_sequence;
    uint32_t unknown_commands;
    uint32_t invalid_states;
    uint32_t response_queue_overflows;
    int32_t last_error;
} CommandDecoderTaskDiagnostics;

uint8_t command_decoder_initialize(void);
uint8_t command_decoder_submit_request(const CommandTaskRequestDTO_t *request);
uint8_t command_decoder_receive_response(CommandTaskResponseDTO_t *response, uint32_t timeout_ms);
void command_decoder_get_response_queue_failure(uint32_t *generation, uint32_t *connection_id);
void command_decoder_get_diagnostics(CommandDecoderTaskDiagnostics *diagnostics);
void CommandDecoderTask(void *argument);

#endif /* FM_V3_COMMAND_DECODER_TASK_H */

#ifndef FM_V3_COMMAND_TASK_H
#define FM_V3_COMMAND_TASK_H

#include <stdint.h>

#define COMMAND_NETWORK_TASK_PORT 2628
#define COMMAND_TASK_STACK_SIZE_BYTES (1024U * 4U)
#define COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS 1000U

typedef struct {
    uint32_t connections_accepted;
    uint32_t connections_closed;
    uint32_t active_connection;
    uint32_t complete_frames_received;
    /* Bad magic, bad length, bad CRC, or a frame cut off by the assembly timeout or a disconnect. */
    uint32_t invalid_frames;
    uint32_t buffer_max_bytes;
    uint32_t response_send_errors;
    uint32_t recv_errors;
    uint32_t decoder_errors;
    /* ACKs held back because the command took the last free slot of the command queue. */
    uint32_t acks_deferred;
    int32_t last_error;
} CommandTaskDiagnostics;

void CommandTask(void *argument);
void command_task_get_diagnostics(CommandTaskDiagnostics *diagnostics);

#endif /* FM_V3_COMMAND_TASK_H */

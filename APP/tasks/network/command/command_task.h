#ifndef FM_V3_COMMAND_TASK_H
#define FM_V3_COMMAND_TASK_H

#include <stdint.h>

#define COMMAND_NETWORK_TASK_PORT 2628
#define COMMAND_TASK_STACK_SIZE_BYTES (1024U * 4U)
#define COMMAND_TASK_RECEIVE_TIMEOUT_MS 10U
#define COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS 1000U

typedef struct {
    uint32_t port;
    uint32_t connections_accepted;
    uint32_t connections_closed;
    uint32_t active_connection;
    uint32_t bytes_received;
    uint32_t complete_frames_received;
    uint32_t invalid_magic;
    uint32_t invalid_frame_length;
    uint32_t invalid_crc;
    uint32_t buffer_current_bytes;
    uint32_t buffer_max_bytes;
    uint32_t bytes_sent;
    uint32_t response_attempts;
    uint32_t response_successes;
    uint32_t response_send_errors;
    uint32_t partial_writes;
    uint32_t netconn_alloc_errors;
    uint32_t bind_errors;
    uint32_t listen_errors;
    uint32_t accept_errors;
    uint32_t recv_errors;
    uint32_t decoder_errors;
    int32_t last_error;
} CommandTaskDiagnostics;

void CommandTask(void *argument);
void command_task_get_diagnostics(CommandTaskDiagnostics *diagnostics);

#endif /* FM_V3_COMMAND_TASK_H */

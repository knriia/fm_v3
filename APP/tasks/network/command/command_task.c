#include "command_task.h"

#include "command/command_queue.h"
#include "command_decoder.h"
#include "command_protocol.h"
#include "network_events.h"
#include "task_context.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define COMMAND_TASK_FLAG_RX (1UL << 0)
#define COMMAND_TASK_FLAG_TX (1UL << 1)
#define COMMAND_TASK_FLAG_ERROR (1UL << 2)
#define COMMAND_TASK_FLAG_QUEUE_SPACE (1UL << 3)
#define COMMAND_TASK_FLAGS_ALL                                                                                         \
    (COMMAND_TASK_FLAG_RX | COMMAND_TASK_FLAG_TX | COMMAND_TASK_FLAG_ERROR | COMMAND_TASK_FLAG_QUEUE_SPACE)

/* The largest response is ERROR: header, 16-bit error code and CRC. */
#define COMMAND_TASK_MAX_RESPONSE_SIZE (COMMAND_FRAME_HEADER_SIZE + 2U + COMMAND_FRAME_CRC_SIZE)
#define COMMAND_TASK_TX_BUFFER_SIZE 256U
/* Input is read only while a held ACK and the reply to one new frame still fit into the send buffer. */
#define COMMAND_TASK_TX_RESERVE_SIZE (2U * COMMAND_TASK_MAX_RESPONSE_SIZE)

static volatile CommandTaskDiagnostics command_task_diagnostics;

static osThreadId_t command_task_thread_id = NULL;

typedef struct {
    struct netconn *client;
    TickType_t partial_frame_started_at;
    uint8_t partial_frame_active;
    uint8_t disconnect_requested;
    CommandFrameAssembler_t *assembler;
    /* ACK held back because the command filled the queue; sent once the queue reports free space. */
    uint8_t ack_pending;
    uint32_t pending_ack_sequence;
    /* Responses not yet accepted by LwIP; bytes [tx_sent, tx_length) are still to be written. */
    uint8_t tx_buffer[COMMAND_TASK_TX_BUFFER_SIZE];
    size_t tx_length;
    size_t tx_sent;
    /* Reading stopped because the send buffer was full; it resumes once responses drain. */
    uint8_t rx_paused;
} CommandTaskAssemblerContext_t;

typedef enum {
    COMMAND_TASK_ASSEMBLY_NEED_MORE = 0,
    COMMAND_TASK_ASSEMBLY_COMMAND_READY,
    COMMAND_TASK_ASSEMBLY_FRAME_ERROR,
    COMMAND_TASK_ASSEMBLY_ERROR
} CommandTaskAssemblyResult_t;

typedef struct {
    CommandTaskAssemblyResult_t result;
    CommandRequestDTO_t command;
    uint32_t sequence;
    uint16_t error_code;
} CommandTaskAssemblyEvent_t;

static uint8_t
command_task_send_response(CommandTaskAssemblerContext_t *assembler_context, const uint8_t *frame, size_t frame_length);
static CommandTaskAssemblyEvent_t
command_task_assemble_pbuf(struct pbuf *buffer, CommandTaskAssemblerContext_t *assembler_context);

static struct netconn *command_task_get_client(const CommandTaskAssemblerContext_t *assembler_context) {
    return assembler_context == NULL ? NULL : assembler_context->client;
}

static void command_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void command_counter_add(volatile uint32_t *counter, size_t value) {
    if (value > UINT32_MAX || UINT32_MAX - *counter < (uint32_t)value) {
        *counter = UINT32_MAX;
    } else {
        *counter += (uint32_t)value;
    }
}

static void command_record_error(err_t error) { command_task_diagnostics.last_error = (int32_t)error; }

static uint8_t command_task_send_protocol_response(
    CommandTaskAssemblerContext_t *assembler_context,
    uint8_t response_type,
    uint32_t sequence,
    uint16_t error_code
) {
    uint8_t frame[COMMAND_MAX_FRAME_SIZE];
    size_t response_length = 0U;
    if (response_type == COMMAND_MESSAGE_TYPE_PONG) {
        response_length = command_protocol_build_pong_response(frame, sizeof(frame), sequence);
    } else if (response_type == COMMAND_MESSAGE_TYPE_ACK) {
        response_length = command_protocol_build_ack_response(frame, sizeof(frame), sequence);
    } else if (response_type == COMMAND_MESSAGE_TYPE_ERROR) {
        response_length = command_protocol_build_error_response(frame, sizeof(frame), sequence, error_code);
    }

    return command_task_send_response(assembler_context, frame, response_length);
}

static uint8_t command_task_assembler_has_incomplete_command(const CommandFrameAssembler_t *assembler) {
    if (assembler == NULL || assembler->length == 0U) {
        return 0U;
    }
    if (assembler->length == 1U) {
        return assembler->buffer[0] == (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU) ? 1U : 0U;
    }
    if (assembler->buffer[0] != (uint8_t)(COMMAND_PROTOCOL_MAGIC & 0xFFU) ||
        assembler->buffer[1] != (uint8_t)(COMMAND_PROTOCOL_MAGIC >> 8U)) {
        return 0U;
    }
    if (assembler->length < COMMAND_FRAME_HEADER_SIZE) {
        return 1U;
    }

    CommandFrameHeaderDTO_t header = {0};
    if (command_protocol_parse_header(assembler->buffer, assembler->length, &header) != COMMAND_FRAME_VALID) {
        return 0U;
    }
    const size_t expected_length = COMMAND_FRAME_HEADER_SIZE + header.payload_length + COMMAND_FRAME_CRC_SIZE;
    return assembler->length < expected_length ? 1U : 0U;
}

static void command_task_update_partial_frame_tracking(CommandTaskAssemblerContext_t *assembler_context) {
    if (assembler_context == NULL) {
        return;
    }
    if (command_task_assembler_has_incomplete_command(assembler_context->assembler) == 0U) {
        assembler_context->partial_frame_active = 0U;
        return;
    }
    if (assembler_context->partial_frame_active == 0U) {
        assembler_context->partial_frame_started_at = xTaskGetTickCount();
        assembler_context->partial_frame_active = 1U;
    }
}

static uint8_t
command_task_finalize_incomplete_frame(CommandTaskAssemblerContext_t *assembler_context, uint8_t send_rejection) {
    if (assembler_context == NULL || assembler_context->disconnect_requested != 0U ||
        assembler_context->partial_frame_active == 0U) {
        return 1U;
    }

    uint32_t sequence = 0U;
    uint8_t sequence_available = 0U;
    CommandFrameHeaderDTO_t header = {0};
    const CommandFrameAssembler_t *assembler = assembler_context->assembler;
    if (command_protocol_parse_header(assembler->buffer, assembler->length, &header) == COMMAND_FRAME_VALID) {
        sequence = header.sequence;
        sequence_available = 1U;
    }

    command_counter_increment(&command_task_diagnostics.invalid_frame_length);
    command_task_diagnostics.last_error = (int32_t)COMMAND_ERROR_TRUNCATED_FRAME;
    uint8_t response_sent = 1U;
    if (send_rejection != 0U && sequence_available != 0U) {
        response_sent = command_task_send_protocol_response(
            assembler_context,
            COMMAND_MESSAGE_TYPE_ERROR,
            sequence,
            COMMAND_ERROR_TRUNCATED_FRAME
        );
    }
    assembler_context->partial_frame_active = 0U;
    command_frame_assembler_init(assembler_context->assembler);
    command_task_diagnostics.buffer_current_bytes = 0U;
    if (response_sent == 0U) {
        assembler_context->disconnect_requested = 1U;
        return 0U;
    }
    return 1U;
}

static uint8_t command_task_reject_expired_incomplete_frame(CommandTaskAssemblerContext_t *assembler_context) {
    if (assembler_context == NULL || assembler_context->partial_frame_active == 0U) {
        return 1U;
    }

    const TickType_t timeout_ticks = pdMS_TO_TICKS(COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS);
    const TickType_t elapsed_ticks = xTaskGetTickCount() - assembler_context->partial_frame_started_at;
    if (elapsed_ticks < timeout_ticks) {
        return 1U;
    }
    return command_task_finalize_incomplete_frame(assembler_context, 1U);
}

static uint16_t command_task_record_frame_error(
    CommandFrameValidationResult_t result,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t *sequence
) {
    if (sequence != NULL) {
        *sequence = 0U;
    }

    if (result == COMMAND_FRAME_VALID) {
        return 0U;
    }

    command_task_diagnostics.last_error = (int32_t)result;
    switch (result) {
    case COMMAND_FRAME_INVALID_MAGIC:
        command_counter_increment(&command_task_diagnostics.invalid_magic);
        break;
    case COMMAND_FRAME_INVALID_LENGTH:
        command_counter_increment(&command_task_diagnostics.invalid_frame_length);
        break;
    case COMMAND_FRAME_INVALID_CRC:
        command_counter_increment(&command_task_diagnostics.invalid_crc);
        break;
    default:
        break;
    }

    uint16_t error_code = 0U;
    if (result == COMMAND_FRAME_INVALID_MAGIC || result == COMMAND_FRAME_INVALID_LENGTH) {
        error_code = COMMAND_ERROR_INVALID_FRAME_FORMAT;
    } else if (result == COMMAND_FRAME_INVALID_CRC) {
        error_code = COMMAND_ERROR_INVALID_FRAME_CRC;
    }
    if (error_code != 0U && frame != NULL) {
        CommandFrameHeaderDTO_t header = {0};
        (void)command_protocol_parse_header(frame, frame_length, &header);
        if (sequence != NULL) {
            *sequence = header.sequence;
        }
    }
    return error_code;
}

void command_task_get_diagnostics(CommandTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->port = COMMAND_NETWORK_TASK_PORT;
    diagnostics->connections_accepted = command_task_diagnostics.connections_accepted;
    diagnostics->connections_closed = command_task_diagnostics.connections_closed;
    diagnostics->active_connection = command_task_diagnostics.active_connection;
    diagnostics->bytes_received = command_task_diagnostics.bytes_received;
    diagnostics->complete_frames_received = command_task_diagnostics.complete_frames_received;
    diagnostics->invalid_magic = command_task_diagnostics.invalid_magic;
    diagnostics->invalid_frame_length = command_task_diagnostics.invalid_frame_length;
    diagnostics->invalid_crc = command_task_diagnostics.invalid_crc;
    diagnostics->buffer_current_bytes = command_task_diagnostics.buffer_current_bytes;
    diagnostics->buffer_max_bytes = command_task_diagnostics.buffer_max_bytes;
    diagnostics->bytes_sent = command_task_diagnostics.bytes_sent;
    diagnostics->response_attempts = command_task_diagnostics.response_attempts;
    diagnostics->response_successes = command_task_diagnostics.response_successes;
    diagnostics->response_send_errors = command_task_diagnostics.response_send_errors;
    diagnostics->partial_writes = command_task_diagnostics.partial_writes;
    diagnostics->netconn_alloc_errors = command_task_diagnostics.netconn_alloc_errors;
    diagnostics->bind_errors = command_task_diagnostics.bind_errors;
    diagnostics->listen_errors = command_task_diagnostics.listen_errors;
    diagnostics->accept_errors = command_task_diagnostics.accept_errors;
    diagnostics->recv_errors = command_task_diagnostics.recv_errors;
    diagnostics->decoder_errors = command_task_diagnostics.decoder_errors;
    diagnostics->last_error = command_task_diagnostics.last_error;
}

static size_t command_task_tx_free_space(const CommandTaskAssemblerContext_t *assembler_context) {
    return COMMAND_TASK_TX_BUFFER_SIZE - (assembler_context->tx_length - assembler_context->tx_sent);
}

/* Hands buffered responses to LwIP without blocking. A full LwIP send buffer leaves the rest for the TX flag.
 * Returns 1 to keep the client connected and 0 to close it. */
static uint8_t command_task_flush_responses(CommandTaskAssemblerContext_t *assembler_context) {
    while (assembler_context->tx_sent < assembler_context->tx_length) {
        const size_t remaining = assembler_context->tx_length - assembler_context->tx_sent;
        size_t bytes_written = 0U;
        const err_t write_error = netconn_write_partly(
            assembler_context->client,
            &assembler_context->tx_buffer[assembler_context->tx_sent],
            remaining,
            NETCONN_COPY | NETCONN_DONTBLOCK,
            &bytes_written
        );

        if (write_error == ERR_WOULDBLOCK) {
            return 1U;
        }
        if (bytes_written > remaining) {
            command_counter_increment(&command_task_diagnostics.response_send_errors);
            command_record_error(ERR_ARG);
            return 0U;
        }
        command_counter_add(&command_task_diagnostics.bytes_sent, bytes_written);
        if (write_error != ERR_OK || bytes_written == 0U) {
            command_counter_increment(&command_task_diagnostics.response_send_errors);
            command_record_error(write_error == ERR_OK ? ERR_BUF : write_error);
            return 0U;
        }

        assembler_context->tx_sent += bytes_written;
        if (bytes_written < remaining) {
            command_counter_increment(&command_task_diagnostics.partial_writes);
            return 1U;
        }
    }

    assembler_context->tx_length = 0U;
    assembler_context->tx_sent = 0U;
    return 1U;
}

/* Queues one response and tries to send it at once. Returns 1 to keep the client connected and 0 to close it. */
static uint8_t command_task_send_response(
    CommandTaskAssemblerContext_t *assembler_context,
    const uint8_t *frame,
    size_t frame_length
) {
    if (assembler_context == NULL || assembler_context->client == NULL || frame == NULL || frame_length == 0U ||
        frame_length > COMMAND_MAX_FRAME_SIZE) {
        command_counter_increment(&command_task_diagnostics.response_send_errors);
        command_record_error(ERR_ARG);
        return 0U;
    }

    command_counter_increment(&command_task_diagnostics.response_attempts);
    if (command_task_tx_free_space(assembler_context) < frame_length) {
        command_counter_increment(&command_task_diagnostics.response_send_errors);
        command_record_error(ERR_MEM);
        return 0U;
    }
    if (assembler_context->tx_length + frame_length > COMMAND_TASK_TX_BUFFER_SIZE) {
        const size_t pending = assembler_context->tx_length - assembler_context->tx_sent;
        (void)memmove(assembler_context->tx_buffer, &assembler_context->tx_buffer[assembler_context->tx_sent], pending);
        assembler_context->tx_length = pending;
        assembler_context->tx_sent = 0U;
    }
    (void)memcpy(&assembler_context->tx_buffer[assembler_context->tx_length], frame, frame_length);
    assembler_context->tx_length += frame_length;
    command_counter_increment(&command_task_diagnostics.response_successes);

    return command_task_flush_responses(assembler_context);
}

static CommandTaskAssemblyEvent_t
command_task_assemble_pbuf(struct pbuf *buffer, CommandTaskAssemblerContext_t *assembler_context) {
    CommandTaskAssemblyEvent_t task_event = {
        .result = COMMAND_TASK_ASSEMBLY_NEED_MORE,
    };
    if (buffer == NULL || assembler_context == NULL || assembler_context->assembler == NULL) {
        command_record_error(ERR_ARG);
        task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
        return task_event;
    }

    const size_t pbuf_total_length = buffer->tot_len;
    size_t pbuf_bytes_appended = 0U;
    for (const struct pbuf *segment = buffer; segment != NULL; segment = segment->next) {
        const void *data = segment->payload;
        const u16_t data_length = segment->len;
        if (data == NULL && data_length != 0U) {
            command_counter_increment(&command_task_diagnostics.recv_errors);
            command_record_error(ERR_BUF);
            task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
            return task_event;
        }

        if (data_length == 0U) {
            continue;
        }

        command_counter_add(&command_task_diagnostics.bytes_received, data_length);
        size_t data_offset = 0U;
        for (;;) {
            const CommandFrameAssemblerEvent_t assembler_event =
                command_frame_assembler_next(assembler_context->assembler);
            if (assembler_event.result == COMMAND_FRAME_ASSEMBLER_NEED_MORE) {
                if (data_offset == data_length) {
                    break;
                }

                const size_t appended = command_frame_assembler_append(
                    assembler_context->assembler,
                    &((const uint8_t *)data)[data_offset],
                    (size_t)data_length - data_offset
                );
                if (appended == 0U) {
                    command_record_error(ERR_BUF);
                    task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                    return task_event;
                }

                data_offset += appended;
                pbuf_bytes_appended += appended;
                if (assembler_context->assembler->max_length > command_task_diagnostics.buffer_max_bytes) {
                    command_task_diagnostics.buffer_max_bytes = (uint32_t)assembler_context->assembler->max_length;
                }
                command_task_diagnostics.buffer_current_bytes = (uint32_t)assembler_context->assembler->length;
                command_task_update_partial_frame_tracking(assembler_context);
                continue;
            }
            if (assembler_event.result == COMMAND_FRAME_ASSEMBLER_INVALID_ARGUMENT) {
                command_record_error(ERR_ARG);
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }

            if (assembler_event.result == COMMAND_FRAME_ASSEMBLER_COMMAND_READY) {
                task_event.command = assembler_event.command;
            } else {
                task_event.error_code = command_task_record_frame_error(
                    assembler_event.validation,
                    assembler_event.frame,
                    assembler_event.frame_length,
                    &task_event.sequence
                );
                if (task_event.error_code == 0U) {
                    command_record_error(ERR_ARG);
                    task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                    return task_event;
                }

                task_event.result = COMMAND_TASK_ASSEMBLY_FRAME_ERROR;
                command_frame_assembler_init(assembler_context->assembler);
                assembler_context->partial_frame_active = 0U;
                command_task_diagnostics.buffer_current_bytes = 0U;
                return task_event;
            }

            if (command_frame_assembler_consume(assembler_context->assembler, assembler_event.consume_length) == 0U) {
                command_record_error(ERR_BUF);
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }
            command_task_diagnostics.buffer_current_bytes = (uint32_t)assembler_context->assembler->length;
            command_task_update_partial_frame_tracking(assembler_context);
            if (assembler_context->disconnect_requested != 0U) {
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }

            const uint8_t trailing_bytes = data_offset < data_length || assembler_context->assembler->length != 0U ||
                                           pbuf_bytes_appended < pbuf_total_length;
            if (trailing_bytes != 0U) {
                command_record_error(ERR_VAL);
                assembler_context->disconnect_requested = 1U;
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }
            task_event.result = COMMAND_TASK_ASSEMBLY_COMMAND_READY;
            return task_event;
        }
    }

    return task_event;
}

static void command_task_netconn_event(struct netconn *conn, enum netconn_evt evt, u16_t len) {
    if (command_task_thread_id == NULL) {
        return;
    }
    (void)conn;
    (void)len;
    switch (evt) {
    case NETCONN_EVT_RCVPLUS:
        osThreadFlagsSet(command_task_thread_id, COMMAND_TASK_FLAG_RX);
        break;
    case NETCONN_EVT_SENDPLUS:
        osThreadFlagsSet(command_task_thread_id, COMMAND_TASK_FLAG_TX);
        break;
    case NETCONN_EVT_ERROR:
        osThreadFlagsSet(command_task_thread_id, COMMAND_TASK_FLAG_ERROR);
        break;
    default:
        break;
    }
}

/* Runs in the consumer task when a command leaves the queue. */
static void command_task_queue_space_available(void) {
    if (command_task_thread_id == NULL) {
        return;
    }
    (void)osThreadFlagsSet(command_task_thread_id, COMMAND_TASK_FLAG_QUEUE_SPACE);
}

/* Returns 1 to keep the client connected and 0 to close it. */
static uint8_t command_task_send_pending_ack(CommandTaskAssemblerContext_t *assembler_context) {
    if (assembler_context->ack_pending == 0U) {
        return 1U;
    }
    assembler_context->ack_pending = 0U;
    if (command_task_send_protocol_response(
            assembler_context,
            COMMAND_MESSAGE_TYPE_ACK,
            assembler_context->pending_ack_sequence,
            0U
        ) == 0U) {
        assembler_context->disconnect_requested = 1U;
        return 0U;
    }
    return 1U;
}

/* Returns 1 to keep the client connected and 0 to close it. */
static uint8_t command_task_handle_assembly_event(
    CommandTaskAssemblerContext_t *assembler_context,
    const CommandTaskAssemblyEvent_t *assembly_event
) {
    if (assembly_event->result == COMMAND_TASK_ASSEMBLY_ERROR) {
        (void)command_task_finalize_incomplete_frame(assembler_context, 0U);
        return 0U;
    }
    if (assembly_event->result == COMMAND_TASK_ASSEMBLY_FRAME_ERROR) {
        if (command_task_send_protocol_response(
                assembler_context,
                COMMAND_MESSAGE_TYPE_ERROR,
                assembly_event->sequence,
                assembly_event->error_code
            ) == 0U) {
            assembler_context->disconnect_requested = 1U;
            return 0U;
        }
        return 1U;
    }
    if (assembly_event->result != COMMAND_TASK_ASSEMBLY_COMMAND_READY) {
        return 1U;
    }

    command_counter_increment(&command_task_diagnostics.complete_frames_received);
    assembler_context->partial_frame_active = 0U;

    const CommandDecoderResult_t decoder_result = command_decoder_decode_request(&assembly_event->command);
    if (decoder_result.type == COMMAND_DECODER_RESULT_INVALID_ARGUMENT) {
        command_counter_increment(&command_task_diagnostics.decoder_errors);
        command_record_error(ERR_ARG);
        assembler_context->disconnect_requested = 1U;
        (void)command_task_finalize_incomplete_frame(assembler_context, 0U);
        return 0U;
    }

    uint8_t response_type = decoder_result.response_type;
    uint16_t error_code = decoder_result.error_code;
    uint32_t response_sequence = assembly_event->command.header.sequence;
    if (decoder_result.type == COMMAND_DECODER_RESULT_COMMAND_READY) {
        const CommandDTO_t *decoded_command = &decoder_result.command;
        response_sequence = decoded_command->sequence;
        const CommandQueuePutResult_t queue_result = command_queue_try_send(decoded_command);
        if (queue_result == COMMAND_QUEUE_PUT_OK) {
            /* The command fitted, so space was freed: the older held ACK goes first to keep the order. */
            if (command_task_send_pending_ack(assembler_context) == 0U) {
                return 0U;
            }
            if (command_queue_free_slots() == 0U) {
                assembler_context->ack_pending = 1U;
                assembler_context->pending_ack_sequence = response_sequence;
                return 1U;
            }
            response_type = COMMAND_MESSAGE_TYPE_ACK;
            error_code = 0U;
        } else {
            response_type = COMMAND_MESSAGE_TYPE_ERROR;
            error_code =
                queue_result == COMMAND_QUEUE_PUT_FULL ? COMMAND_ERROR_QUEUE_FULL : COMMAND_ERROR_INVALID_STATE;
        }
    }
    if (command_task_send_protocol_response(assembler_context, response_type, response_sequence, error_code) == 0U) {
        assembler_context->disconnect_requested = 1U;
        return 0U;
    }
    return 1U;
}

/* Reads everything LwIP already holds for the client without blocking.
 * Returns 1 to keep the client connected and 0 to close it. */
static uint8_t command_task_receive_available(CommandTaskAssemblerContext_t *assembler_context) {
    for (;;) {
        /* Unread data stays in LwIP and its TCP window holds the client back until responses drain. */
        if (command_task_tx_free_space(assembler_context) < COMMAND_TASK_TX_RESERVE_SIZE) {
            assembler_context->rx_paused = 1U;
            return 1U;
        }

        struct pbuf *buffer = NULL;
        const err_t receive_error = netconn_recv_tcp_pbuf_flags(assembler_context->client, &buffer, NETCONN_DONTBLOCK);
        if (receive_error == ERR_WOULDBLOCK) {
            return 1U;
        }
        if (receive_error != ERR_OK) {
            command_counter_increment(&command_task_diagnostics.recv_errors);
            command_record_error(receive_error);
            (void)command_task_finalize_incomplete_frame(assembler_context, 0U);
            return 0U;
        }
        if (buffer == NULL) {
            command_counter_increment(&command_task_diagnostics.recv_errors);
            command_record_error(ERR_BUF);
            (void)command_task_finalize_incomplete_frame(assembler_context, 0U);
            return 0U;
        }

        assembler_context->disconnect_requested = 0U;
        const CommandTaskAssemblyEvent_t assembly_event = command_task_assemble_pbuf(buffer, assembler_context);
        (void)pbuf_free(buffer);
        if (command_task_handle_assembly_event(assembler_context, &assembly_event) == 0U) {
            return 0U;
        }
    }
}

/* Ticks to wait for task flags: forever without a partial frame, 0 once its deadline has passed. */
static uint32_t command_task_flags_wait_timeout(const CommandTaskAssemblerContext_t *assembler_context) {
    if (assembler_context->partial_frame_active == 0U) {
        return osWaitForever;
    }

    const TickType_t timeout_ticks = pdMS_TO_TICKS(COMMAND_TASK_FRAME_ASSEMBLY_TIMEOUT_MS);
    const TickType_t elapsed_ticks = xTaskGetTickCount() - assembler_context->partial_frame_started_at;
    if (elapsed_ticks >= timeout_ticks) {
        return 0U;
    }
    return (uint32_t)(timeout_ticks - elapsed_ticks);
}

static void command_task_serve_client(struct netconn *client) {
    CommandFrameAssembler_t assembler;
    command_frame_assembler_init(&assembler);
    CommandTaskAssemblerContext_t assembler_context = {
        .client = client,
        .partial_frame_started_at = 0U,
        .partial_frame_active = 0U,
        .disconnect_requested = 0U,
        .assembler = &assembler,
    };

    /* Data may arrive before the flags are cleared after accept, so read once before the first wait. */
    uint8_t read_now = 1U;
    for (;;) {
        if (read_now == 0U) {
            const uint32_t timeout = command_task_flags_wait_timeout(&assembler_context);
            uint32_t flags = osFlagsErrorTimeout;
            if (timeout != 0U) {
                flags = osThreadFlagsWait(COMMAND_TASK_FLAGS_ALL, osFlagsWaitAny, timeout);
            }
            if (flags == osFlagsErrorTimeout) {
                if (command_task_reject_expired_incomplete_frame(&assembler_context) == 0U) {
                    return;
                }
                continue;
            }
            configASSERT((flags & osFlagsError) == 0U);
            /* The held ACK belongs to an older command, so it is sent before any new data is processed. */
            if ((flags & COMMAND_TASK_FLAG_QUEUE_SPACE) != 0U) {
                if (command_task_send_pending_ack(&assembler_context) == 0U) {
                    return;
                }
            }
            /* A broken connection also fails the write, so ERROR flushes too. */
            if ((flags & (COMMAND_TASK_FLAG_TX | COMMAND_TASK_FLAG_ERROR)) != 0U) {
                if (command_task_flush_responses(&assembler_context) == 0U) {
                    return;
                }
            }
            if ((flags & (COMMAND_TASK_FLAG_RX | COMMAND_TASK_FLAG_ERROR)) != 0U) {
                read_now = 1U;
            }
            /* The RX flag was consumed when reading paused, so drained responses resume reading on their own. */
            if (assembler_context.rx_paused != 0U &&
                command_task_tx_free_space(&assembler_context) >= COMMAND_TASK_TX_RESERVE_SIZE) {
                read_now = 1U;
            }
            if (read_now == 0U) {
                continue;
            }
        }
        read_now = 0U;
        assembler_context.rx_paused = 0U;

        /* A connection error is reported by the next receive call, so ERROR is handled as a read too. */
        if (command_task_receive_available(&assembler_context) == 0U) {
            return;
        }
    }
}

void CommandTask(void *argument) {
    NetworkTaskContext *context = argument;
    configASSERT(context != NULL);
    configASSERT(context->lwip_flags != NULL);
    command_task_thread_id = osThreadGetId();
    command_queue_set_space_listener(command_task_queue_space_available);

    const uint32_t wait_result =
        osEventFlagsWait(context->lwip_flags, LWIP_READY_FLAG, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
    configASSERT((wait_result & osFlagsError) == 0U);

    for (;;) {
        struct netconn *listener = netconn_new_with_callback(NETCONN_TCP, command_task_netconn_event);
        if (listener == NULL) {
            command_counter_increment(&command_task_diagnostics.netconn_alloc_errors);
            command_record_error(ERR_MEM);
            osDelay(10U);
            continue;
        }

        const err_t bind_error = netconn_bind(listener, IP_ADDR_ANY, COMMAND_NETWORK_TASK_PORT);
        if (bind_error != ERR_OK) {
            command_counter_increment(&command_task_diagnostics.bind_errors);
            command_record_error(bind_error);
            netconn_delete(listener);
            osDelay(10U);
            continue;
        }

        const err_t listen_error = netconn_listen(listener);
        if (listen_error != ERR_OK) {
            command_counter_increment(&command_task_diagnostics.listen_errors);
            command_record_error(listen_error);
            netconn_close(listener);
            netconn_delete(listener);
            osDelay(10U);
            continue;
        }

        for (;;) {
            struct netconn *client = NULL;
            const err_t accept_error = netconn_accept(listener, &client);

            if (accept_error != ERR_OK) {
                command_counter_increment(&command_task_diagnostics.accept_errors);
                command_record_error(accept_error);
                if (accept_error == ERR_ABRT) {
                    osDelay(10U);
                    continue;
                }
                break;
            }
            osThreadFlagsClear(COMMAND_TASK_FLAGS_ALL);
            command_task_diagnostics.buffer_current_bytes = 0U;
            command_counter_increment(&command_task_diagnostics.connections_accepted);
            command_task_diagnostics.active_connection = 1U;

            command_task_serve_client(client);

            netconn_close(client);
            netconn_delete(client);
            command_task_diagnostics.buffer_current_bytes = 0U;
            command_task_diagnostics.active_connection = 0U;
            command_counter_increment(&command_task_diagnostics.connections_closed);
        }

        netconn_close(listener);
        netconn_delete(listener);
    }
}

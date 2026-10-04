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
#include "lwip/netbuf.h"

#include <stddef.h>
#include <stdint.h>

static volatile CommandTaskDiagnostics command_task_diagnostics;
static CommandFrameAssembler_t command_task_assembler;

typedef struct {
    struct netconn *client;
    TickType_t partial_frame_started_at;
    uint8_t partial_frame_active;
    uint8_t disconnect_requested;
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

static uint8_t command_task_send_response(struct netconn *client, const uint8_t *frame, size_t frame_length);
static CommandTaskAssemblyEvent_t
command_task_assemble_netbuf(struct netbuf *buffer, CommandTaskAssemblerContext_t *assembler_context);

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
    struct netconn *client,
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

    return command_task_send_response(client, frame, response_length);
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
    if (command_task_assembler_has_incomplete_command(&command_task_assembler) == 0U) {
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
    if (command_protocol_parse_header(command_task_assembler.buffer, command_task_assembler.length, &header) ==
        COMMAND_FRAME_VALID) {
        sequence = header.sequence;
        sequence_available = 1U;
    }

    command_counter_increment(&command_task_diagnostics.invalid_frame_length);
    command_task_diagnostics.last_error = (int32_t)COMMAND_ERROR_TRUNCATED_FRAME;
    uint8_t response_sent = 1U;
    if (send_rejection != 0U && sequence_available != 0U) {
        response_sent = command_task_send_protocol_response(
            assembler_context->client,
            COMMAND_MESSAGE_TYPE_ERROR,
            sequence,
            COMMAND_ERROR_TRUNCATED_FRAME
        );
    }
    assembler_context->partial_frame_active = 0U;
    command_frame_assembler_init(&command_task_assembler);
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

static uint8_t command_task_send_response(struct netconn *client, const uint8_t *frame, size_t frame_length) {
    if (client == NULL || frame == NULL || frame_length == 0U || frame_length > COMMAND_MAX_FRAME_SIZE) {
        command_counter_increment(&command_task_diagnostics.response_send_errors);
        command_record_error(ERR_ARG);
        return 0U;
    }

    size_t total_written = 0U;
    uint8_t response_was_partial = 0U;
    command_counter_increment(&command_task_diagnostics.response_attempts);

    while (total_written < frame_length) {
        const size_t remaining = frame_length - total_written;
        size_t bytes_written = 0U;
        const err_t write_error =
            netconn_write_partly(client, &frame[total_written], remaining, NETCONN_COPY, &bytes_written);

        if (bytes_written > remaining) {
            command_counter_increment(&command_task_diagnostics.response_send_errors);
            if (response_was_partial != 0U) {
                command_counter_increment(&command_task_diagnostics.partial_writes);
            }
            command_record_error(ERR_ARG);
            return 0U;
        }

        command_counter_add(&command_task_diagnostics.bytes_sent, bytes_written);
        if (write_error != ERR_OK || bytes_written == 0U) {
            command_counter_increment(&command_task_diagnostics.response_send_errors);
            if (total_written != 0U || (bytes_written != 0U && bytes_written < remaining)) {
                response_was_partial = 1U;
            }
            if (response_was_partial != 0U) {
                command_counter_increment(&command_task_diagnostics.partial_writes);
            }
            command_record_error(write_error == ERR_OK ? ERR_BUF : write_error);
            return 0U;
        }

        total_written += bytes_written;
        if (bytes_written < remaining) {
            response_was_partial = 1U;
        }
    }

    if (response_was_partial != 0U) {
        command_counter_increment(&command_task_diagnostics.partial_writes);
    }
    command_counter_increment(&command_task_diagnostics.response_successes);
    return 1U;
}

static CommandTaskAssemblyEvent_t
command_task_assemble_netbuf(struct netbuf *buffer, CommandTaskAssemblerContext_t *assembler_context) {
    CommandTaskAssemblyEvent_t task_event = {
        .result = COMMAND_TASK_ASSEMBLY_NEED_MORE,
    };
    netbuf_first(buffer);
    const size_t netbuf_total_length = netbuf_len(buffer);
    size_t netbuf_bytes_appended = 0U;
    do {
        void *data = NULL;
        u16_t data_length = 0U;
        const err_t data_error = netbuf_data(buffer, &data, &data_length);
        if (data_error != ERR_OK) {
            command_counter_increment(&command_task_diagnostics.recv_errors);
            command_record_error(data_error);
            task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
            return task_event;
        }
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
            const CommandFrameAssemblerEvent_t assembler_event = command_frame_assembler_next(&command_task_assembler);
            if (assembler_event.result == COMMAND_FRAME_ASSEMBLER_NEED_MORE) {
                if (data_offset == data_length) {
                    break;
                }

                const size_t appended = command_frame_assembler_append(
                    &command_task_assembler,
                    &((const uint8_t *)data)[data_offset],
                    (size_t)data_length - data_offset
                );
                if (appended == 0U) {
                    command_record_error(ERR_BUF);
                    task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                    return task_event;
                }

                data_offset += appended;
                netbuf_bytes_appended += appended;
                if (command_task_assembler.max_length > command_task_diagnostics.buffer_max_bytes) {
                    command_task_diagnostics.buffer_max_bytes = (uint32_t)command_task_assembler.max_length;
                }
                command_task_diagnostics.buffer_current_bytes = (uint32_t)command_task_assembler.length;
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
                command_frame_assembler_init(&command_task_assembler);
                assembler_context->partial_frame_active = 0U;
                command_task_diagnostics.buffer_current_bytes = 0U;
                return task_event;
            }

            if (command_frame_assembler_consume(&command_task_assembler, assembler_event.consume_length) == 0U) {
                command_record_error(ERR_BUF);
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }
            command_task_diagnostics.buffer_current_bytes = (uint32_t)command_task_assembler.length;
            command_task_update_partial_frame_tracking(assembler_context);
            if (assembler_context->disconnect_requested != 0U) {
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }

            const uint8_t trailing_bytes = data_offset < data_length || command_task_assembler.length != 0U ||
                                           netbuf_bytes_appended < netbuf_total_length;
            if (trailing_bytes != 0U) {
                command_record_error(ERR_VAL);
                assembler_context->disconnect_requested = 1U;
                task_event.result = COMMAND_TASK_ASSEMBLY_ERROR;
                return task_event;
            }
            task_event.result = COMMAND_TASK_ASSEMBLY_COMMAND_READY;
            return task_event;
        }
    } while (netbuf_next(buffer) >= 0);

    return task_event;
}

void CommandTask(void *argument) {
    NetworkTaskContext *context = argument;
    if ((context == NULL) || (context->lwip_flags == NULL)) {
        Error_Handler();
        return;
    }

    if (osEventFlagsWait(context->lwip_flags, LWIP_READY_FLAG, osFlagsWaitAll, osWaitForever) & osFlagsError) {
        Error_Handler();
        return;
    }

    for (;;) {
        struct netconn *listener = netconn_new(NETCONN_TCP);
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
                break;
            }
            if (client == NULL) {
                command_counter_increment(&command_task_diagnostics.accept_errors);
                command_record_error(ERR_CONN);
                break;
            }

            command_frame_assembler_init(&command_task_assembler);
            command_task_diagnostics.buffer_current_bytes = 0U;
            command_counter_increment(&command_task_diagnostics.connections_accepted);
            command_task_diagnostics.active_connection = 1U;
            netconn_set_recvtimeout(client, COMMAND_TASK_RECEIVE_TIMEOUT_MS);
            CommandTaskAssemblerContext_t assembler_context = {
                .client = client,
                .partial_frame_started_at = 0U,
                .partial_frame_active = 0U,
                .disconnect_requested = 0U,
            };
            for (;;) {
                struct netbuf *buffer = NULL;
                const err_t receive_error = netconn_recv(client, &buffer);
                if (receive_error == ERR_TIMEOUT) {
                    if (command_task_reject_expired_incomplete_frame(&assembler_context) == 0U) {
                        break;
                    }
                    continue;
                }
                if (receive_error != ERR_OK) {
                    command_counter_increment(&command_task_diagnostics.recv_errors);
                    command_record_error(receive_error);
                    (void)command_task_finalize_incomplete_frame(&assembler_context, 0U);
                    break;
                }
                if (buffer == NULL) {
                    command_counter_increment(&command_task_diagnostics.recv_errors);
                    command_record_error(ERR_BUF);
                    (void)command_task_finalize_incomplete_frame(&assembler_context, 0U);
                    break;
                }

                assembler_context.disconnect_requested = 0U;
                const CommandTaskAssemblyEvent_t assembly_event =
                    command_task_assemble_netbuf(buffer, &assembler_context);
                netbuf_delete(buffer);
                if (assembly_event.result == COMMAND_TASK_ASSEMBLY_ERROR) {
                    (void)command_task_finalize_incomplete_frame(&assembler_context, 0U);
                    break;
                }
                if (assembly_event.result == COMMAND_TASK_ASSEMBLY_FRAME_ERROR) {
                    if (command_task_send_protocol_response(
                            client,
                            COMMAND_MESSAGE_TYPE_ERROR,
                            assembly_event.sequence,
                            assembly_event.error_code
                        ) == 0U) {
                        assembler_context.disconnect_requested = 1U;
                        break;
                    }
                    continue;
                }
                if (assembly_event.result == COMMAND_TASK_ASSEMBLY_COMMAND_READY) {
                    command_counter_increment(&command_task_diagnostics.complete_frames_received);
                    assembler_context.partial_frame_active = 0U;

                    const CommandDecoderResult_t decoder_result =
                        command_decoder_decode_request(&assembly_event.command);
                    if (decoder_result.type == COMMAND_DECODER_RESULT_INVALID_ARGUMENT) {
                        command_counter_increment(&command_task_diagnostics.decoder_errors);
                        command_record_error(ERR_ARG);
                        assembler_context.disconnect_requested = 1U;
                        (void)command_task_finalize_incomplete_frame(&assembler_context, 0U);
                        break;
                    }

                    uint8_t response_type = decoder_result.response_type;
                    uint16_t error_code = decoder_result.error_code;
                    uint32_t response_sequence = assembly_event.command.header.sequence;
                    if (decoder_result.type == COMMAND_DECODER_RESULT_COMMAND_READY) {
                        const CommandDTO_t *decoded_command = &decoder_result.command;
                        response_sequence = decoded_command->sequence;
                        const CommandQueuePutResult_t queue_result = command_queue_try_send(decoded_command);
                        if (queue_result == COMMAND_QUEUE_PUT_OK) {
                            response_type = COMMAND_MESSAGE_TYPE_ACK;
                            error_code = 0U;
                        } else {
                            response_type = COMMAND_MESSAGE_TYPE_ERROR;
                            error_code = queue_result == COMMAND_QUEUE_PUT_FULL ? COMMAND_ERROR_QUEUE_FULL
                                                                                : COMMAND_ERROR_INVALID_STATE;
                        }
                    }
                    if (command_task_send_protocol_response(client, response_type, response_sequence, error_code) ==
                        0U) {
                        assembler_context.disconnect_requested = 1U;
                        break;
                    }
                }
            }

            netconn_close(client);
            netconn_delete(client);
            command_frame_assembler_init(&command_task_assembler);
            command_task_diagnostics.buffer_current_bytes = 0U;
            command_task_diagnostics.active_connection = 0U;
            command_counter_increment(&command_task_diagnostics.connections_closed);
        }

        netconn_close(listener);
        netconn_delete(listener);
    }
}

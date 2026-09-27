#include "command_task.h"

#include "command_decoder_task.h"
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
static uint32_t command_task_next_connection_id;
static uint32_t command_task_active_connection_id;
static uint32_t command_task_seen_response_queue_overflows;
static CommandFrameParser_t command_task_frame_parser;

typedef struct {
    uint32_t connection_id;
    uint8_t disconnect_requested;
} CommandTaskParserContext_t;

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

static uint32_t command_task_next_connection(void) {
    ++command_task_next_connection_id;
    if (command_task_next_connection_id == 0U) {
        ++command_task_next_connection_id;
    }
    return command_task_next_connection_id;
}

static void command_task_record_frame_error(CommandFrameValidationResult_t result, void *context) {
    (void)context;
    if (result != COMMAND_FRAME_VALID) {
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
    }
}

static void command_task_snapshot_response_queue_failures(void) {
    uint32_t generation = 0U;
    command_decoder_get_response_queue_failure(&generation, NULL);
    command_task_seen_response_queue_overflows = generation;
}

static void command_task_process_frame(const uint8_t *frame, size_t frame_length, void *context) {
    CommandTaskParserContext_t *parser_context = context;
    if (parser_context != NULL && parser_context->disconnect_requested != 0U) {
        command_counter_increment(&command_task_diagnostics.complete_frames_received);
        return;
    }
    const uint32_t connection_id =
        parser_context == NULL ? command_task_active_connection_id : parser_context->connection_id;
    CommandTaskRequestDTO_t request = {
        .connection_id = connection_id,
    };
    const CommandFrameValidationResult_t validation =
        command_protocol_decode_request(frame, frame_length, &request.request);
    if (validation != COMMAND_FRAME_VALID) {
        command_task_record_frame_error(validation, NULL);
        return;
    }

    command_counter_increment(&command_task_diagnostics.complete_frames_received);
    if (command_decoder_submit_request(&request) == 0U) {
        command_counter_increment(&command_task_diagnostics.decoder_queue_errors);
        command_record_error(ERR_MEM);
        if (parser_context != NULL) {
            parser_context->disconnect_requested = 1U;
        }
    }
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
    diagnostics->decoder_queue_errors = command_task_diagnostics.decoder_queue_errors;
    diagnostics->last_error = command_task_diagnostics.last_error;
}

static uint8_t command_task_send_response(struct netconn *client, const CommandTaskResponseDTO_t *response) {
    if (client == NULL || response == NULL || response->length == 0U || response->length > sizeof(response->frame)) {
        command_counter_increment(&command_task_diagnostics.response_send_errors);
        command_record_error(ERR_ARG);
        return 0U;
    }

    size_t total_written = 0U;
    uint8_t response_was_partial = 0U;
    command_counter_increment(&command_task_diagnostics.response_attempts);

    while (total_written < response->length) {
        const size_t remaining = response->length - total_written;
        size_t bytes_written = 0U;
        const err_t write_error =
            netconn_write_partly(client, &response->frame[total_written], remaining, NETCONN_COPY, &bytes_written);

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

static uint8_t command_task_drain_responses(struct netconn *client, uint32_t connection_id) {
    CommandTaskResponseDTO_t response;
    while (command_decoder_receive_response(&response, 0U) != 0U) {
        if (response.connection_id == connection_id) {
            if (command_task_send_response(client, &response) == 0U) {
                return 0U;
            }
        }
    }

    uint32_t overflow_generation = 0U;
    uint32_t overflow_connection_id = 0U;
    command_decoder_get_response_queue_failure(&overflow_generation, &overflow_connection_id);
    if (overflow_generation != command_task_seen_response_queue_overflows) {
        command_task_seen_response_queue_overflows = overflow_generation;
        if (overflow_connection_id == connection_id) {
            command_record_error(ERR_MEM);
            return 0U;
        }
    }
    return 1U;
}

static uint8_t command_task_process_netbuf(struct netbuf *buffer, CommandTaskParserContext_t *parser_context) {
    if (buffer == NULL) {
        return 1U;
    }

    netbuf_first(buffer);
    do {
        void *data = NULL;
        u16_t data_length = 0U;
        const err_t data_error = netbuf_data(buffer, &data, &data_length);
        if (data_error != ERR_OK) {
            command_counter_increment(&command_task_diagnostics.recv_errors);
            command_record_error(data_error);
            return 0U;
        }
        if (data != NULL && data_length != 0U) {
            command_counter_add(&command_task_diagnostics.bytes_received, data_length);
            command_frame_parser_feed_ex(
                &command_task_frame_parser,
                data,
                data_length,
                command_task_process_frame,
                command_task_record_frame_error,
                parser_context
            );
            command_task_diagnostics.buffer_current_bytes = (uint32_t)command_task_frame_parser.length;
            if (command_task_frame_parser.max_length > command_task_diagnostics.buffer_max_bytes) {
                command_task_diagnostics.buffer_max_bytes = (uint32_t)command_task_frame_parser.max_length;
            }
            if (parser_context != NULL && parser_context->disconnect_requested != 0U) {
                return 0U;
            }
        }
    } while (netbuf_next(buffer) >= 0);
    return 1U;
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

    command_task_snapshot_response_queue_failures();

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

            const uint32_t connection_id = command_task_next_connection();
            command_task_active_connection_id = connection_id;
            command_task_snapshot_response_queue_failures();
            command_frame_parser_init(&command_task_frame_parser);
            command_task_diagnostics.buffer_current_bytes = 0U;
            command_counter_increment(&command_task_diagnostics.connections_accepted);
            command_task_diagnostics.active_connection = 1U;
            netconn_set_recvtimeout(client, COMMAND_TASK_RECEIVE_TIMEOUT_MS);
            for (;;) {
                struct netbuf *buffer = NULL;
                const err_t receive_error = netconn_recv(client, &buffer);
                if (receive_error == ERR_TIMEOUT) {
                    if (command_task_drain_responses(client, connection_id) == 0U) {
                        break;
                    }
                    continue;
                }
                if (receive_error != ERR_OK) {
                    command_counter_increment(&command_task_diagnostics.recv_errors);
                    command_record_error(receive_error);
                    (void)command_task_drain_responses(client, connection_id);
                    break;
                }
                if (buffer == NULL) {
                    command_counter_increment(&command_task_diagnostics.recv_errors);
                    command_record_error(ERR_BUF);
                    break;
                }

                CommandTaskParserContext_t parser_context = {
                    .connection_id = connection_id,
                    .disconnect_requested = 0U,
                };
                const uint8_t keep_connection = command_task_process_netbuf(buffer, &parser_context);
                netbuf_delete(buffer);
                if (keep_connection == 0U || command_task_drain_responses(client, connection_id) == 0U) {
                    break;
                }
            }

            netconn_close(client);
            netconn_delete(client);
            command_task_active_connection_id = 0U;
            command_frame_parser_init(&command_task_frame_parser);
            command_task_diagnostics.buffer_current_bytes = 0U;
            command_task_diagnostics.active_connection = 0U;
            command_counter_increment(&command_task_diagnostics.connections_closed);
        }

        netconn_close(listener);
        netconn_delete(listener);
    }
}

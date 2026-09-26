#include "command_task.h"

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

void command_task_get_diagnostics(CommandTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->port = COMMAND_NETWORK_TASK_PORT;
    diagnostics->protocol_version = COMMAND_PROTOCOL_VERSION;
    diagnostics->max_payload_size = COMMAND_MAX_PAYLOAD_SIZE;
    diagnostics->max_frame_size = COMMAND_MAX_FRAME_SIZE;
    diagnostics->connections_accepted = command_task_diagnostics.connections_accepted;
    diagnostics->connections_closed = command_task_diagnostics.connections_closed;
    diagnostics->active_connection = command_task_diagnostics.active_connection;
    diagnostics->bytes_received = command_task_diagnostics.bytes_received;
    diagnostics->bytes_sent = command_task_diagnostics.bytes_sent;
    diagnostics->frames_received = command_task_diagnostics.frames_received;
    diagnostics->commands_received = command_task_diagnostics.commands_received;
    diagnostics->commands_rejected = command_task_diagnostics.commands_rejected;
    diagnostics->pings_received = command_task_diagnostics.pings_received;
    diagnostics->pongs_sent = command_task_diagnostics.pongs_sent;
    diagnostics->errors_sent = command_task_diagnostics.errors_sent;
    diagnostics->response_attempts = command_task_diagnostics.response_attempts;
    diagnostics->response_successes = command_task_diagnostics.response_successes;
    diagnostics->response_send_errors = command_task_diagnostics.response_send_errors;
    diagnostics->partial_writes = command_task_diagnostics.partial_writes;
    diagnostics->netconn_alloc_errors = command_task_diagnostics.netconn_alloc_errors;
    diagnostics->bind_errors = command_task_diagnostics.bind_errors;
    diagnostics->listen_errors = command_task_diagnostics.listen_errors;
    diagnostics->accept_errors = command_task_diagnostics.accept_errors;
    diagnostics->recv_errors = command_task_diagnostics.recv_errors;
    diagnostics->last_error = command_task_diagnostics.last_error;
}

static void
command_task_send_response(struct netconn *client, const uint8_t *frame, size_t frame_length, uint8_t response_type) {
    size_t bytes_written = 0U;
    command_counter_increment(&command_task_diagnostics.response_attempts);
    const err_t write_error = netconn_write_partly(client, frame, frame_length, NETCONN_COPY, &bytes_written);
    command_counter_add(&command_task_diagnostics.bytes_sent, bytes_written);

    if (write_error != ERR_OK || bytes_written != frame_length) {
        command_counter_increment(&command_task_diagnostics.response_send_errors);
        if (bytes_written != frame_length) {
            command_counter_increment(&command_task_diagnostics.partial_writes);
        }
        command_record_error(write_error == ERR_OK ? ERR_BUF : write_error);
        return;
    }

    command_counter_increment(&command_task_diagnostics.response_successes);
    if (response_type == COMMAND_MESSAGE_TYPE_PONG) {
        command_counter_increment(&command_task_diagnostics.pongs_sent);
    } else if (response_type == COMMAND_MESSAGE_TYPE_ERROR) {
        command_counter_increment(&command_task_diagnostics.errors_sent);
    }
}

static void command_task_process_frame(const uint8_t *frame, size_t frame_length, void *context) {
    struct netconn *client = context;
    if (client == NULL) {
        return;
    }

    CommandFrameHeaderDTO_t header;
    if (command_protocol_validate_frame(frame, frame_length, &header) != COMMAND_FRAME_VALID) {
        return;
    }

    command_counter_increment(&command_task_diagnostics.frames_received);
    if (header.type == COMMAND_MESSAGE_TYPE_COMMAND) {
        command_counter_increment(&command_task_diagnostics.commands_received);
    } else if (header.type == COMMAND_MESSAGE_TYPE_PING) {
        command_counter_increment(&command_task_diagnostics.pings_received);
    }

    const uint8_t *payload = &frame[COMMAND_FRAME_HEADER_SIZE];
    const uint16_t error_code = command_protocol_validate_request(&header, payload);
    if (header.type == COMMAND_MESSAGE_TYPE_COMMAND && error_code != 0U) {
        command_counter_increment(&command_task_diagnostics.commands_rejected);
    }

    uint8_t response[COMMAND_MAX_FRAME_SIZE];
    size_t response_length = 0U;
    uint8_t response_type = COMMAND_MESSAGE_TYPE_ERROR;

    if (error_code == 0U && header.type == COMMAND_MESSAGE_TYPE_PING) {
        response_type = COMMAND_MESSAGE_TYPE_PONG;
        response_length = command_protocol_build_pong_response(response, sizeof(response), header.sequence);
    } else {
        response_length = command_protocol_build_error_response(
            response,
            sizeof(response),
            header.sequence,
            error_code == 0U ? COMMAND_ERROR_INVALID_STATE : error_code
        );
    }

    if (response_length != 0U) {
        command_task_send_response(client, response, response_length, response_type);
    }
}

static void command_task_process_netbuf(struct netbuf *buffer, CommandFrameParser_t *parser, struct netconn *client) {
    if (buffer == NULL) {
        return;
    }

    netbuf_first(buffer);

    do {
        void *data = NULL;
        u16_t data_length = 0U;
        if (netbuf_data(buffer, &data, &data_length) == ERR_OK && data != NULL && data_length != 0U) {
            command_counter_add(&command_task_diagnostics.bytes_received, data_length);
            command_frame_parser_feed(parser, data, data_length, command_task_process_frame, client);
        }
    } while (netbuf_next(buffer) >= 0);
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

            command_counter_increment(&command_task_diagnostics.connections_accepted);
            command_task_diagnostics.active_connection = 1U;
            CommandFrameParser_t parser;
            command_frame_parser_init(&parser);
            for (;;) {
                struct netbuf *buffer = NULL;
                const err_t receive_error = netconn_recv(client, &buffer);
                if (receive_error != ERR_OK) {
                    command_counter_increment(&command_task_diagnostics.recv_errors);
                    command_record_error(receive_error);
                    break;
                }
                command_task_process_netbuf(buffer, &parser, client);
                netbuf_delete(buffer);
            }

            netconn_close(client);
            netconn_delete(client);
            command_task_diagnostics.active_connection = 0U;
            command_counter_increment(&command_task_diagnostics.connections_closed);
        }

        netconn_close(listener);
        netconn_delete(listener);
    }
}
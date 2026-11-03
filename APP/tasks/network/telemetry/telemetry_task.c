#include "telemetry_task.h"
#include "encoder.h"
#include "network_events.h"
#include "task_context.h"
#include "telemetry_dto.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/opt.h"

#include <stddef.h>
#include <stdint.h>

void TelemetryTask(void *argument) {
    NetworkTaskContext *context = argument;
    configASSERT(context != NULL);
    configASSERT(context->lwip_flags != NULL);

    const uint32_t wait_result =
        osEventFlagsWait(context->lwip_flags, LWIP_READY_FLAG, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
    configASSERT((wait_result & osFlagsError) == 0U);

    uint32_t sequence = 0U;
    for (;;) {
        struct netconn *connection = netconn_new(NETCONN_TCP);
        if (connection == NULL) {
            osDelay(10U);
            continue;
        }

        if (netconn_bind(connection, IP_ADDR_ANY, TELEMETRY_NETWORK_TASK_PORT) != ERR_OK) {
            netconn_delete(connection);
            osDelay(10U);
            continue;
        }

        if (netconn_listen(connection) != ERR_OK) {
            netconn_close(connection);
            netconn_delete(connection);
            osDelay(10U);
            continue;
        }

        struct netconn *client = NULL;
        while (netconn_accept(connection, &client) == ERR_OK) {
            netconn_set_sendtimeout(client, TELEMETRY_NETWORK_SEND_TIMEOUT_MS);
            for (;;) {
                TelemetryFrameDTO_t frame;
                TelemetryCoordinates_t coordinates;
                size_t bytes_written = 0U;
                coordinates.x = encoder_ox_get_coordinate();
                coordinates.y = encoder_oy_get_coordinate();
                coordinates.z = encoder_oz_get_coordinate();
                telemetry_build_frame(&frame, ++sequence, &coordinates);

                const err_t write_error =
                    netconn_write_partly(client, &frame, sizeof(frame), NETCONN_COPY, &bytes_written);
                if (write_error != ERR_OK || bytes_written != sizeof(frame)) {
                    break;
                }
                osDelay(TELEMETRY_NETWORK_TASK_INTERVAL_MS);
            }

            netconn_close(client);
            netconn_delete(client);
        }

        netconn_close(connection);
        netconn_delete(connection);
    }
}

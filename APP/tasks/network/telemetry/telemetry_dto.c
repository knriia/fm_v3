#include "telemetry_dto.h"

#include <stddef.h>

void telemetry_build_frame(TelemetryFrameDTO_t *frame, uint32_t sequence, const TelemetryCoordinates_t *coordinates) {
    if ((frame == NULL) || (coordinates == NULL)) {
        return;
    }

    *frame = (TelemetryFrameDTO_t){
        .header =
            {
                .magic = NETWORK_PROTOCOL_MAGIC,
                .version = NETWORK_PROTOCOL_VERSION,
                .message_type = NETWORK_MESSAGE_TYPE_TELEMETRY,
                .header_length = sizeof(NetworkFrameHeaderDTO_t),
                .payload_length = sizeof(TelemetryPayloadDTO_t),
            },
        .payload =
            {
                .sequence = sequence,
                .x = coordinates->x,
                .y = coordinates->y,
                .z = coordinates->z,
                .is_test_data = 0U,
                .reserved = {0U, 0U, 0U},
            },
    };
}

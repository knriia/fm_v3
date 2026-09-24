#include "telemetry_dto.h"

#include <stddef.h>

void telemetry_build_test_frame(TelemetryFrameDTO_t *frame, uint32_t sequence) {
    if (frame == NULL) {
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
                .x = TELEMETRY_TEST_COORDINATE_X,
                .y = TELEMETRY_TEST_COORDINATE_Y,
                .z = TELEMETRY_TEST_COORDINATE_Z,
                .is_test_data = 1U,
                .reserved = {0U, 0U, 0U},
            },
    };
}

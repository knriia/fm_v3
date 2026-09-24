#include "telemetry_dto.h"

#include <stdint.h>
#include <stdio.h>

static uint32_t test_failures;

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

static void expect_i32(int32_t actual, int32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %ld, got %ld)\n", message, (long)expected, (long)actual);
    }
}

int main(void) {
    TelemetryFrameDTO_t frame = {0};
    telemetry_build_test_frame(&frame, 42U);

    expect_u32(frame.header.magic, NETWORK_PROTOCOL_MAGIC, "telemetry magic");
    expect_u32(frame.header.version, NETWORK_PROTOCOL_VERSION, "telemetry protocol version");
    expect_u32(frame.header.message_type, NETWORK_MESSAGE_TYPE_TELEMETRY, "telemetry message type");
    expect_u32(frame.header.header_length, sizeof(NetworkFrameHeaderDTO_t), "telemetry header length");
    expect_u32(frame.header.payload_length, sizeof(TelemetryPayloadDTO_t), "telemetry payload length");
    expect_u32(frame.payload.sequence, 42U, "telemetry sequence");
    expect_i32(frame.payload.x, TELEMETRY_TEST_COORDINATE_X, "telemetry test x");
    expect_i32(frame.payload.y, TELEMETRY_TEST_COORDINATE_Y, "telemetry test y");
    expect_i32(frame.payload.z, TELEMETRY_TEST_COORDINATE_Z, "telemetry test z");
    expect_u32(frame.payload.is_test_data, 1U, "telemetry test data flag");
    expect_u32(sizeof(frame), 28U, "telemetry frame size");

    telemetry_build_test_frame(&frame, UINT32_MAX);
    expect_u32(frame.payload.sequence, UINT32_MAX, "telemetry sequence maximum");

    return test_failures == 0U ? 0 : 1;
}

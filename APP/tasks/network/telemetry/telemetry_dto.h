#ifndef FM_V3_TELEMETRY_DTO_H
#define FM_V3_TELEMETRY_DTO_H

#include "dto.h"

#include <stdint.h>

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} TelemetryCoordinates_t;

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t is_test_data;
    uint8_t reserved[3];
} TelemetryPayloadDTO_t;

typedef struct __attribute__((packed)) {
    NetworkFrameHeaderDTO_t header;
    TelemetryPayloadDTO_t payload;
} TelemetryFrameDTO_t;

void telemetry_build_frame(TelemetryFrameDTO_t *frame, uint32_t sequence, const TelemetryCoordinates_t *coordinates);

_Static_assert(sizeof(TelemetryPayloadDTO_t) == 20U, "Invalid telemetry payload size");
_Static_assert(sizeof(TelemetryFrameDTO_t) == 28U, "Invalid telemetry frame size");

#endif /* FM_V3_TELEMETRY_DTO_H */

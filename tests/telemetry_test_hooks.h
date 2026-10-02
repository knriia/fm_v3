#ifndef FM_V3_TELEMETRY_TEST_HOOKS_H
#define FM_V3_TELEMETRY_TEST_HOOKS_H

#include <stdint.h>

void telemetry_task_test_seed_counters(
    uint32_t connections_accepted,
    uint32_t send_attempts,
    uint32_t send_successes,
    uint32_t bytes_sent
);

#endif /* FM_V3_TELEMETRY_TEST_HOOKS_H */

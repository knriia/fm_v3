#ifndef FM_V3_COMMAND_QUEUE_H
#define FM_V3_COMMAND_QUEUE_H

#include "cmsis_os.h"
#include "command_decoder.h"

#include <stdint.h>

#define COMMAND_QUEUE_CAPACITY 100U

typedef enum { COMMAND_QUEUE_PUT_OK = 0, COMMAND_QUEUE_PUT_FULL, COMMAND_QUEUE_PUT_ERROR } CommandQueuePutResult_t;

typedef struct {
    uint32_t initialized;
    uint32_t capacity;
    uint32_t queued;
    uint32_t available;
    uint32_t max_queued;
    uint32_t enqueued_total;
    uint32_t full_rejections;
    uint32_t enqueue_errors;
} CommandQueueDiagnostics_t;

osStatus_t command_queue_init(void);
CommandQueuePutResult_t command_queue_try_send(const CommandDTO_t *command);
osStatus_t command_queue_receive(CommandDTO_t *command, uint32_t timeout_ms);
void command_queue_get_diagnostics(CommandQueueDiagnostics_t *diagnostics);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void command_queue_test_seed_counters(uint32_t enqueued_total, uint32_t full_rejections, uint32_t enqueue_errors);
#endif

#endif /* FM_V3_COMMAND_QUEUE_H */

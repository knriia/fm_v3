#ifndef FM_V3_COMMAND_QUEUE_H
#define FM_V3_COMMAND_QUEUE_H

#include "cmsis_os.h"
#include "command_decoder.h"

#include <stdint.h>

#define COMMAND_QUEUE_CAPACITY 100U

typedef enum { COMMAND_QUEUE_PUT_OK = 0, COMMAND_QUEUE_PUT_FULL, COMMAND_QUEUE_PUT_ERROR } CommandQueuePutResult_t;

/* Sent as is in the diagnostic frame. */
typedef struct {
    uint32_t queued;
    uint32_t max_queued;
    uint32_t enqueued_total;
    uint32_t full_rejections;
    uint32_t enqueue_errors;
} CommandQueueDiagnostics_t;

/* Called from the consumer task after a command leaves the queue; must only signal and return. */
typedef void (*CommandQueueSpaceListener_t)(void);

osStatus_t command_queue_init(void);
CommandQueuePutResult_t command_queue_try_send(const CommandDTO_t *command);
osStatus_t command_queue_receive(CommandDTO_t *command, uint32_t timeout_ms);
uint32_t command_queue_free_slots(void);
/* Drops every queued command; used by STOP, which cancels everything accepted before it. */
void command_queue_discard_all(void);
void command_queue_set_space_listener(CommandQueueSpaceListener_t listener);
void command_queue_get_diagnostics(CommandQueueDiagnostics_t *diagnostics);

#ifdef FM_V3_ENABLE_TEST_HOOKS
void command_queue_test_seed_counters(uint32_t enqueued_total, uint32_t full_rejections, uint32_t enqueue_errors);
#endif

#endif /* FM_V3_COMMAND_QUEUE_H */

#ifndef FM_V3_COMMAND_RESULT_QUEUE_H
#define FM_V3_COMMAND_RESULT_QUEUE_H

#include "cmsis_os.h"
#include "command/command_queue.h"

#include <stdint.h>

/* Moves that can wait for their second result at once (MOVE_RESULT_TRACKER_CAPACITY; MotionTask checks the two are
 * equal). */
#define COMMAND_RESULT_QUEUE_PENDING_MOVES 128U
/* A command that waits in the command queue still owes its first result, and a move that has been accepted owes its
 * final one, so the queue holds both and one more: a result is not lost while the client reads slowly. */
#define COMMAND_RESULT_QUEUE_CAPACITY (COMMAND_QUEUE_CAPACITY + COMMAND_RESULT_QUEUE_PENDING_MOVES + 1U)

/* What MotionTask decided about one command: a DONE frame with a COMMAND_DONE_* result code, or an ERROR frame with a
 * COMMAND_ERROR_* code. */
typedef struct {
    uint32_t sequence;
    uint8_t response_type;
    uint16_t code;
    /* Of an ERROR frame: see CommandErrorPayloadDTO_t. */
    uint8_t cause;
    uint8_t axis;
    uint8_t detail;
} CommandResult_t;

/* Called from the producer task after a result is queued; must only signal and return. */
typedef void (*CommandResultListener_t)(void);

osStatus_t command_result_queue_init(void);
/* Only MotionTask posts. Never waits; a result that does not fit is dropped, which takes a client that has left
 * COMMAND_RESULT_QUEUE_CAPACITY results unread. */
void command_result_queue_post(uint32_t sequence, uint8_t response_type, uint16_t code);
/* The same with the stop details of an ERROR frame; command_result_queue_post gives cause 0, COMMAND_ERROR_AXIS_NONE
 * and detail 0. */
void command_result_queue_post_detailed(
    uint32_t sequence,
    uint8_t response_type,
    uint16_t code,
    uint8_t cause,
    uint8_t axis,
    uint8_t detail
);
/* Never waits; osOK when a result was taken, osErrorResource when none is queued. */
osStatus_t command_result_queue_try_receive(CommandResult_t *result);
/* Drops every queued result; a new connection must not receive the results of the previous one. */
void command_result_queue_discard_all(void);
void command_result_queue_set_listener(CommandResultListener_t listener);

#endif /* FM_V3_COMMAND_RESULT_QUEUE_H */

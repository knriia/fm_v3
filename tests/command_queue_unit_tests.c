#include "command/command_queue.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t capacity;
    uint32_t message_size;
    uint32_t count;
    CommandDTO_t messages[COMMAND_QUEUE_CAPACITY];
} TestMessageQueue_t;

static uint32_t test_failures;
static uint8_t test_create_fails;
static osStatus_t test_next_put_status;
static osStatus_t test_next_get_status;
static uint32_t test_new_calls;
static uint32_t test_put_calls;
static uint32_t test_get_calls;
static uint32_t test_last_put_timeout;
static uint32_t test_last_get_timeout;
static uint8_t test_last_put_priority;
static uint8_t test_attributes_were_null;
static TestMessageQueue_t test_queue;
static uint32_t test_space_listener_calls;

static void test_space_listener(void) { ++test_space_listener_calls; }

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

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr) {
    ++test_new_calls;
    test_attributes_were_null = attr == NULL ? 1U : 0U;
    if (test_create_fails != 0U) {
        test_create_fails = 0U;
        return NULL;
    }
    memset(&test_queue, 0, sizeof(test_queue));
    test_queue.capacity = msg_count;
    test_queue.message_size = msg_size;
    return &test_queue;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t queue_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout) {
    ++test_put_calls;
    test_last_put_priority = msg_prio;
    test_last_put_timeout = timeout;
    if (queue_id != &test_queue || msg_ptr == NULL) {
        return osErrorParameter;
    }
    if (test_next_put_status != osOK) {
        const osStatus_t status = test_next_put_status;
        test_next_put_status = osOK;
        return status;
    }
    if (test_queue.count >= test_queue.capacity) {
        return osErrorResource;
    }
    test_queue.messages[test_queue.count] = *(const CommandDTO_t *)msg_ptr;
    ++test_queue.count;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t queue_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout) {
    (void)msg_prio;
    ++test_get_calls;
    test_last_get_timeout = timeout;
    if (queue_id != &test_queue || msg_ptr == NULL) {
        return osErrorParameter;
    }
    if (test_next_get_status != osOK) {
        const osStatus_t status = test_next_get_status;
        test_next_get_status = osOK;
        return status;
    }
    if (test_queue.count == 0U) {
        return osErrorResource;
    }
    *(CommandDTO_t *)msg_ptr = test_queue.messages[0];
    --test_queue.count;
    if (test_queue.count > 0U) {
        memmove(&test_queue.messages[0], &test_queue.messages[1], test_queue.count * sizeof(CommandDTO_t));
    }
    return osOK;
}

uint32_t osMessageQueueGetCount(osMessageQueueId_t queue_id) {
    (void)queue_id;
    return test_queue.count;
}


uint32_t osMessageQueueGetSpace(osMessageQueueId_t queue_id) {
    (void)queue_id;
    return test_queue.capacity - test_queue.count;
}

static CommandDTO_t make_command(uint32_t sequence) {
    return (CommandDTO_t){
        .sequence = sequence,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation = {
            .operation_flags = (uint8_t)(sequence & 0x07U),
            .x = (int32_t)(sequence * 10U),
            .y = -(int32_t)(sequence * 20U),
            .z = (int32_t)(sequence * 30U),
            .speed = sequence * 100U,
            .e_delta = -(int32_t)sequence,
            .e_speed = sequence * 5U,
            .spindle_pwm = (uint16_t)(sequence * 2U),
            .laser_pwm = (uint16_t)(sequence * 3U),
        },
    };
}

static void test_command_queue_initialization_and_arguments(void) {
    CommandDTO_t command = make_command(1U);
    CommandQueueDiagnostics_t diagnostics = {0};
    expect_u32(command_queue_try_send(&command), COMMAND_QUEUE_PUT_ERROR, "send before initialization fails");
    expect_u32(command_queue_receive(&command, 0U), (uint32_t)osErrorResource, "receive before initialization fails");
    expect_u32(command_queue_try_send(NULL), COMMAND_QUEUE_PUT_ERROR, "null command cannot be queued");
    expect_u32(command_queue_receive(NULL, 0U), (uint32_t)osErrorParameter, "null receive output is rejected");
    expect_u32(command_queue_free_slots(), 0U, "uninitialized queue has no free slots");
    command_queue_get_diagnostics(NULL);
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.queued, 0U, "uninitialized queue reports no queued commands");
    expect_u32(diagnostics.enqueue_errors, 2U, "pre-initialization and null sends are counted as errors");

    test_create_fails = 1U;
    expect_u32(command_queue_init(), (uint32_t)osErrorNoMemory, "allocation failure is reported");
    expect_u32(command_queue_init(), (uint32_t)osOK, "queue initializes after allocation recovers");
    expect_u32(test_new_calls, 2U, "queue creation is attempted for each initialization attempt");
    expect_u32(test_queue.capacity, COMMAND_QUEUE_CAPACITY, "queue uses configured capacity");
    expect_u32(test_queue.message_size, sizeof(CommandDTO_t), "queue stores complete command DTOs");
    expect_u32(test_attributes_were_null, 1U, "queue uses CMSIS default allocation attributes");
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.queued, 0U, "new queue has no queued commands");
    expect_u32(command_queue_init(), (uint32_t)osErrorResource, "queue cannot be initialized twice");
}

static void test_command_queue_space_listener(void) {
    CommandDTO_t input = make_command(300U);
    CommandDTO_t output = {0};
    while (command_queue_receive(&output, 0U) == osOK) {
    }
    const uint32_t free_before = command_queue_free_slots();
    expect_u32(free_before, COMMAND_QUEUE_CAPACITY, "drained queue has every slot free");

    command_queue_set_space_listener(test_space_listener);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "command is queued for listener checks");
    expect_u32(command_queue_free_slots(), free_before - 1U, "queued command takes one free slot");
    expect_u32(test_space_listener_calls, 0U, "queuing a command does not report free space");
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osOK, "queued command is received for listener checks");
    expect_u32(test_space_listener_calls, 1U, "taking a command reports free space");
    expect_u32(command_queue_free_slots(), free_before, "received command frees its slot");
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osErrorResource, "empty queue has nothing to take");
    expect_u32(test_space_listener_calls, 1U, "failed receive does not report free space");

    command_queue_set_space_listener(NULL);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "command is queued without a listener");
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osOK, "command is received without a listener");
    expect_u32(test_space_listener_calls, 1U, "removed listener is not called");
}

static void test_command_queue_fifo_full_and_copy_semantics(void) {
    CommandDTO_t input;
    for (uint32_t sequence = 1U; sequence <= COMMAND_QUEUE_CAPACITY; ++sequence) {
        input = make_command(sequence);
        expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "command is queued while space remains");
        input.sequence = UINT32_MAX;
    }
    expect_u32(test_last_put_priority, 0U, "commands use FIFO message priority");
    expect_u32(test_last_put_timeout, 0U, "producer never waits for queue capacity");

    input = make_command(COMMAND_QUEUE_CAPACITY + 1U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_FULL, "full queue rejects the next command");
    test_next_put_status = osErrorParameter;
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_ERROR, "unexpected RTOS send error is reported");

    CommandQueueDiagnostics_t diagnostics = {0};
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.queued, COMMAND_QUEUE_CAPACITY, "diagnostics report current queue occupancy");
    expect_u32(diagnostics.max_queued, COMMAND_QUEUE_CAPACITY, "diagnostics record maximum occupancy");
    expect_u32(diagnostics.enqueued_total, COMMAND_QUEUE_CAPACITY, "diagnostics count accepted commands");
    expect_u32(diagnostics.full_rejections, 1U, "diagnostics count full queue rejections");
    expect_u32(diagnostics.enqueue_errors, 3U, "diagnostics count enqueue errors");

    for (uint32_t sequence = 1U; sequence <= COMMAND_QUEUE_CAPACITY; ++sequence) {
        CommandDTO_t output = {0};
        expect_u32(command_queue_receive(&output, osWaitForever), (uint32_t)osOK, "queued command is received");
        expect_u32(output.sequence, sequence, "queue preserves FIFO order and copies the DTO");
        expect_u32(output.code, COMMAND_CODE_MOTION_OPERATION, "queue preserves command code");
        expect_u32(
            (uint32_t)output.parameters.motion_operation.x,
            sequence * 10U,
            "queue preserves command parameters"
        );
    }
    expect_u32(test_last_get_timeout, osWaitForever, "consumer timeout is passed to CMSIS queue");

    CommandDTO_t output = {0};
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osErrorResource, "empty queue reports no command");
    test_next_get_status = osErrorParameter;
    expect_u32(command_queue_receive(&output, 23U), (uint32_t)osErrorParameter, "unexpected RTOS receive error is propagated");
    expect_u32(test_last_get_timeout, 23U, "consumer timeout is preserved");

    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.queued, 0U, "diagnostics report drained queue");
    expect_u32(diagnostics.max_queued, COMMAND_QUEUE_CAPACITY, "maximum occupancy survives queue draining");

    input = make_command(200U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "queue accepts command after draining");
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.queued, 1U, "diagnostics report one command after queue reuse");
    expect_u32(diagnostics.max_queued, COMMAND_QUEUE_CAPACITY, "maximum occupancy remains at its historical peak");
    expect_u32(diagnostics.enqueued_total, COMMAND_QUEUE_CAPACITY + 1U, "accepted command counter includes queue reuse");
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osOK, "reused queue command can be received");

    command_queue_test_seed_counters(UINT32_MAX, UINT32_MAX, UINT32_MAX);
    input = make_command(201U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "queue accepts command after counter seeding");
    for (uint32_t sequence = 1U; sequence < COMMAND_QUEUE_CAPACITY; ++sequence) {
        input = make_command(sequence);
        expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "queue fills for saturation checks");
    }
    input = make_command(COMMAND_QUEUE_CAPACITY + 1U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_FULL, "saturated full counter does not alter full result");
    test_next_put_status = osErrorParameter;
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_ERROR, "saturated error counter does not alter send result");
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.enqueued_total, UINT32_MAX, "accepted counter saturates");
    expect_u32(diagnostics.full_rejections, UINT32_MAX, "full rejection counter saturates");
    expect_u32(diagnostics.enqueue_errors, UINT32_MAX, "enqueue error counter saturates");
}

static uint32_t test_put_listener_calls;
static uint32_t test_put_listener_free_slots;

static void test_put_listener(void) {
    ++test_put_listener_calls;
    test_put_listener_free_slots = command_queue_free_slots();
}

static void test_command_queue_put_listener(void) {
    CommandDTO_t output = {0};
    while (command_queue_receive(&output, 0U) == osOK) {
    }
    expect_u32(command_queue_free_slots(), COMMAND_QUEUE_CAPACITY, "drained queue has every slot free");

    command_queue_set_put_listener(test_put_listener);
    CommandDTO_t input = make_command(400U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "command is queued for the put listener");
    expect_u32(test_put_listener_calls, 1U, "a queued command calls the put listener once");
    expect_u32(
        test_put_listener_free_slots,
        COMMAND_QUEUE_CAPACITY - 1U,
        "the listener is called after the command is in the queue"
    );
    input = make_command(401U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "a second command is queued");
    expect_u32(test_put_listener_calls, 2U, "each queued command calls the listener");
    expect_u32(test_put_listener_free_slots, COMMAND_QUEUE_CAPACITY - 2U, "the listener sees the second command queued");

    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osOK, "a command is received");
    expect_u32(test_put_listener_calls, 2U, "taking a command does not call the put listener");

    /* Fill the queue: every accepted command, and only those, calls the listener. */
    uint32_t accepted = 2U;
    for (uint32_t sequence = 402U; command_queue_free_slots() != 0U; ++sequence) {
        input = make_command(sequence);
        expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "command is queued while space remains");
        ++accepted;
    }
    expect_u32(test_put_listener_calls, accepted, "the listener was called once per accepted command");
    expect_u32(test_put_listener_free_slots, 0U, "the last accepted command filled the queue before the call");

    const uint32_t calls_when_full = test_put_listener_calls;
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_FULL, "a full queue rejects the command");
    expect_u32(test_put_listener_calls, calls_when_full, "a rejected command does not call the listener");
    test_next_put_status = osErrorParameter;
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_ERROR, "an RTOS send error is reported");
    expect_u32(test_put_listener_calls, calls_when_full, "a failed send does not call the listener");
    expect_u32(command_queue_try_send(NULL), COMMAND_QUEUE_PUT_ERROR, "a null command is rejected");
    expect_u32(test_put_listener_calls, calls_when_full, "a null command does not call the listener");

    /* Without a listener nothing is called. */
    command_queue_set_put_listener(NULL);
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osOK, "a command is received to make room");
    input = make_command(900U);
    expect_u32(command_queue_try_send(&input), COMMAND_QUEUE_PUT_OK, "a command is queued without a listener");
    expect_u32(test_put_listener_calls, calls_when_full, "a removed listener is not called");
}

static uint32_t test_epoch_probe_calls;
static uint32_t test_epoch_probe_epoch;
static uint32_t test_epoch_probe_free_slots;
static uint32_t test_epoch_probe_wrong_epoch_calls;
static uint32_t test_epoch_probe_expected_epoch;

/* A space listener that looks at the epoch while the queue is being drained. */
static void test_epoch_probe_listener(void) {
    ++test_epoch_probe_calls;
    test_epoch_probe_epoch = command_queue_epoch();
    test_epoch_probe_free_slots = command_queue_free_slots();
    if (test_epoch_probe_epoch != test_epoch_probe_expected_epoch) {
        ++test_epoch_probe_wrong_epoch_calls;
    }
}

static void drain_command_queue(void) {
    CommandDTO_t drained = {0};
    while (command_queue_receive(&drained, 0U) == osOK) {
    }
}

static void send_commands(uint32_t count) {
    for (uint32_t sequence = 1U; sequence <= count; ++sequence) {
        const CommandDTO_t command = make_command(sequence);
        expect_u32(command_queue_try_send(&command), COMMAND_QUEUE_PUT_OK, "a command is queued for the flush tests");
    }
}

/* The module starts with the epoch 0. Before the queue exists there is nothing to drain: discarding changes no epoch,
 * and a flush only starts the next one. */
static void test_epoch_starts_at_zero_and_works_before_initialization(void) {
    expect_u32(command_queue_epoch(), 0U, "the epoch of a new module is 0");
    command_queue_discard_pending();
    expect_u32(command_queue_epoch(), 0U, "dropping the commands of a queue that does not exist changes no epoch");
    command_queue_flush();
    expect_u32(command_queue_epoch(), 1U, "a flush before the queue exists starts the next epoch");
    expect_u32(command_queue_free_slots(), 0U, "the flush created no queue");
}

static void test_discard_pending_drops_the_commands_and_keeps_the_epoch(void) {
    drain_command_queue();
    command_queue_set_space_listener(NULL);
    const uint32_t epoch = command_queue_epoch();
    send_commands(5U);
    expect_u32(command_queue_free_slots(), COMMAND_QUEUE_CAPACITY - 5U, "five commands wait in the queue");

    test_space_listener_calls = 0U;
    command_queue_set_space_listener(test_space_listener);
    command_queue_discard_pending();
    expect_u32(command_queue_free_slots(), COMMAND_QUEUE_CAPACITY, "every waiting command is dropped");
    expect_u32(test_space_listener_calls, 5U, "each dropped command frees its place for the producer");
    expect_u32(command_queue_epoch(), epoch, "dropping the commands does not change the epoch");
    CommandDTO_t output = {0};
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osErrorResource, "no command survives");

    command_queue_discard_pending();
    expect_u32(test_space_listener_calls, 5U, "dropping from an empty queue frees nothing");
    expect_u32(command_queue_epoch(), epoch, "dropping from an empty queue does not change the epoch either");

    send_commands(COMMAND_QUEUE_CAPACITY);
    expect_u32(command_queue_free_slots(), 0U, "the queue is full");
    command_queue_discard_pending();
    expect_u32(command_queue_free_slots(), COMMAND_QUEUE_CAPACITY, "a full queue is dropped completely");
    expect_u32(test_space_listener_calls, 5U + COMMAND_QUEUE_CAPACITY, "every command of the full queue was taken out");
    command_queue_set_space_listener(NULL);
}

static void test_flush_drops_the_commands_and_starts_the_next_epoch(void) {
    drain_command_queue();
    command_queue_set_space_listener(NULL);
    const uint32_t epoch = command_queue_epoch();
    send_commands(5U);

    command_queue_flush();
    expect_u32(command_queue_free_slots(), COMMAND_QUEUE_CAPACITY, "the flush drops every waiting command");
    expect_u32(command_queue_epoch(), epoch + 1U, "the flush raises the epoch by one");
    CommandDTO_t output = {0};
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osErrorResource, "no command survives the flush");

    command_queue_flush();
    expect_u32(command_queue_epoch(), epoch + 2U, "a flush of an empty queue starts an epoch as well");
    CommandDTO_t command = make_command(77U);
    expect_u32(command_queue_try_send(&command), COMMAND_QUEUE_PUT_OK, "the queue takes commands after a flush");
    expect_u32(command_queue_receive(&output, 0U), (uint32_t)osOK, "the command after the flush is received");
    expect_u32(output.sequence, 77U, "the command after the flush is the one sent");
    expect_u32(command_queue_epoch(), epoch + 2U, "sending and receiving do not change the epoch");
}

/* A consumer reads the epoch before it takes a command. The epoch must rise only after the drain: a command that the
 * flush takes out, or that the consumer took just before, is then older than the epoch. */
static void test_flush_raises_the_epoch_after_the_queue_is_drained(void) {
    drain_command_queue();
    const uint32_t epoch = command_queue_epoch();
    send_commands(3U);
    test_epoch_probe_calls = 0U;
    test_epoch_probe_wrong_epoch_calls = 0U;
    test_epoch_probe_epoch = UINT32_MAX;
    test_epoch_probe_free_slots = UINT32_MAX;
    test_epoch_probe_expected_epoch = epoch;
    command_queue_set_space_listener(test_epoch_probe_listener);

    command_queue_flush();
    expect_u32(test_epoch_probe_calls, 3U, "the flush took the three commands out one by one");
    expect_u32(test_epoch_probe_wrong_epoch_calls, 0U, "every command left the queue while the epoch was still the old one");
    expect_u32(test_epoch_probe_epoch, epoch, "the last drained command saw the old epoch");
    expect_u32(test_epoch_probe_free_slots, COMMAND_QUEUE_CAPACITY, "the queue was empty at the last call");
    expect_u32(command_queue_epoch(), epoch + 1U, "the epoch rose once the queue was empty");

    /* Dropping without a flush: the probe sees the same epoch before and after. */
    send_commands(2U);
    test_epoch_probe_calls = 0U;
    test_epoch_probe_expected_epoch = epoch + 1U;
    command_queue_discard_pending();
    expect_u32(test_epoch_probe_calls, 2U, "the drop took the two commands out");
    expect_u32(test_epoch_probe_wrong_epoch_calls, 0U, "the epoch stayed during the drop");
    expect_u32(command_queue_epoch(), epoch + 1U, "and after it");
    command_queue_set_space_listener(NULL);
}

int main(void) {
    test_epoch_starts_at_zero_and_works_before_initialization();
    test_command_queue_initialization_and_arguments();
    test_command_queue_fifo_full_and_copy_semantics();
    test_command_queue_space_listener();
    test_command_queue_put_listener();
    test_discard_pending_drops_the_commands_and_keeps_the_epoch();
    test_flush_drops_the_commands_and_starts_the_next_epoch();
    test_flush_raises_the_epoch_after_the_queue_is_drained();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command queue unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command queue unit tests passed.\n");
    return 0;
}

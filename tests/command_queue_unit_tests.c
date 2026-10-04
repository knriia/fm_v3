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
    command_queue_get_diagnostics(NULL);
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.initialized, 0U, "diagnostics report queue is not initialized");
    expect_u32(diagnostics.capacity, COMMAND_QUEUE_CAPACITY, "diagnostics report configured capacity");
    expect_u32(diagnostics.queued, 0U, "uninitialized queue reports no queued commands");
    expect_u32(diagnostics.available, 0U, "uninitialized queue reports no available slots");
    expect_u32(diagnostics.enqueue_errors, 2U, "pre-initialization and null sends are counted as errors");

    test_create_fails = 1U;
    expect_u32(command_queue_init(), (uint32_t)osErrorNoMemory, "allocation failure is reported");
    expect_u32(command_queue_init(), (uint32_t)osOK, "queue initializes after allocation recovers");
    expect_u32(test_new_calls, 2U, "queue creation is attempted for each initialization attempt");
    expect_u32(test_queue.capacity, COMMAND_QUEUE_CAPACITY, "queue uses configured capacity");
    expect_u32(test_queue.message_size, sizeof(CommandDTO_t), "queue stores complete command DTOs");
    expect_u32(test_attributes_were_null, 1U, "queue uses CMSIS default allocation attributes");
    command_queue_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.initialized, 1U, "diagnostics report initialized queue");
    expect_u32(diagnostics.queued, 0U, "new queue has no queued commands");
    expect_u32(diagnostics.available, COMMAND_QUEUE_CAPACITY, "new queue has all slots available");
    expect_u32(command_queue_init(), (uint32_t)osErrorResource, "queue cannot be initialized twice");
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
    expect_u32(diagnostics.capacity, COMMAND_QUEUE_CAPACITY, "diagnostics report queue capacity");
    expect_u32(diagnostics.queued, COMMAND_QUEUE_CAPACITY, "diagnostics report current queue occupancy");
    expect_u32(diagnostics.available, 0U, "full queue reports no free slots");
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
    expect_u32(diagnostics.available, COMMAND_QUEUE_CAPACITY, "drained queue reports all slots available");
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

int main(void) {
    test_command_queue_initialization_and_arguments();
    test_command_queue_fifo_full_and_copy_semantics();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command queue unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command queue unit tests passed.\n");
    return 0;
}

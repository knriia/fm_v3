#include "command/command_result_queue.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t capacity;
    uint32_t message_size;
    uint32_t count;
    CommandResult_t messages[COMMAND_RESULT_QUEUE_CAPACITY];
} TestMessageQueue_t;

static uint32_t test_failures;
static uint8_t test_create_fails;
static osStatus_t test_next_put_status;
static osStatus_t test_next_get_status;
static uint32_t test_new_calls;
static uint32_t test_put_calls;
static uint32_t test_get_calls;
static uint32_t test_reset_calls;
static uint32_t test_last_put_timeout;
static uint32_t test_last_get_timeout;
static uint8_t test_last_put_priority;
static uint8_t test_attributes_were_null;
static TestMessageQueue_t test_queue;
static uint32_t test_listener_calls;
static uint32_t test_listener_queued_at_call;

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

static void test_listener(void) {
    ++test_listener_calls;
    test_listener_queued_at_call = test_queue.count;
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
    test_queue.messages[test_queue.count] = *(const CommandResult_t *)msg_ptr;
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
    *(CommandResult_t *)msg_ptr = test_queue.messages[0];
    --test_queue.count;
    if (test_queue.count > 0U) {
        memmove(&test_queue.messages[0], &test_queue.messages[1], test_queue.count * sizeof(CommandResult_t));
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

osStatus_t osMessageQueueReset(osMessageQueueId_t queue_id) {
    ++test_reset_calls;
    if (queue_id == &test_queue) {
        test_queue.count = 0U;
    }
    return osOK;
}

/* The module state is static and the module is linked as a normal source, so the tests run in the order of its
 * lifecycle: uninitialized first, then initialization, then the initialized module. */
static void reset_counters(void) {
    test_create_fails = 0U;
    test_next_put_status = osOK;
    test_next_get_status = osOK;
    test_new_calls = 0U;
    test_put_calls = 0U;
    test_get_calls = 0U;
    test_reset_calls = 0U;
    test_last_put_timeout = UINT32_MAX;
    test_last_get_timeout = UINT32_MAX;
    test_last_put_priority = UINT8_MAX;
    test_attributes_were_null = 0U;
    test_listener_calls = 0U;
    test_listener_queued_at_call = UINT32_MAX;
}

/* For tests of the initialized module: empties the queue and removes the listener. */
static void reset_initialized_fixture(void) {
    test_queue.count = 0U;
    reset_counters();
    command_result_queue_set_listener(NULL);
}

static void test_before_initialization_nothing_is_queued(void) {
    reset_counters();
    CommandResult_t result = {.sequence = 0xAAAAAAAAU, .response_type = 0xAAU, .code = 0xAAAAU};
    command_result_queue_set_listener(test_listener);

    command_result_queue_post(1U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_put_calls, 0U, "nothing is put into a queue that does not exist");
    expect_u32(test_listener_calls, 0U, "a result that was not queued does not wake the consumer");

    expect_u32(
        (uint32_t)command_result_queue_try_receive(&result),
        (uint32_t)osErrorResource,
        "receiving before initialization finds no result"
    );
    expect_u32(result.sequence, 0xAAAAAAAAU, "a failed receive leaves the output untouched");
    expect_u32(test_get_calls, 0U, "no queue is read before initialization");
    expect_u32(
        (uint32_t)command_result_queue_try_receive(NULL),
        (uint32_t)osErrorParameter,
        "a null output is rejected before initialization"
    );

    command_result_queue_discard_all();
    expect_u32(test_reset_calls, 0U, "there is nothing to discard before initialization");
    command_result_queue_set_listener(NULL);
}

static void test_initialization(void) {
    reset_counters();
    test_create_fails = 1U;
    expect_u32((uint32_t)command_result_queue_init(), (uint32_t)osErrorNoMemory, "allocation failure is reported");
    command_result_queue_post(2U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_put_calls, 0U, "a failed allocation leaves the module uninitialized");

    expect_u32((uint32_t)command_result_queue_init(), (uint32_t)osOK, "the queue initializes after allocation recovers");
    expect_u32(test_new_calls, 2U, "creation is attempted on each initialization");
    /* 100 queued commands + 128 moves that wait for their final result + 1. */
    expect_u32(test_queue.capacity, 229U, "there is room for the results of every queued command and waiting move");
    expect_u32(test_queue.capacity, COMMAND_RESULT_QUEUE_CAPACITY, "the queue is created with the declared capacity");
    expect_u32(
        test_queue.message_size,
        12U,
        "a message holds a 32-bit sequence, an 8-bit type, a 16-bit code and three 8-bit fields, padded to 12 bytes"
    );
    expect_u32(test_attributes_were_null, 1U, "the queue uses the default CMSIS allocation");

    expect_u32((uint32_t)command_result_queue_init(), (uint32_t)osErrorResource, "the queue cannot be initialized twice");
    expect_u32(test_new_calls, 2U, "a repeated initialization creates no second queue");
    expect_u32(
        (uint32_t)command_result_queue_try_receive(NULL),
        (uint32_t)osErrorParameter,
        "a null output is rejected after initialization"
    );
    expect_u32(test_get_calls, 0U, "a null output does not read the queue");
}

static void test_results_keep_their_order_and_values(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    command_result_queue_set_listener(test_listener);

    command_result_queue_post(10U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post(11U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
    command_result_queue_post(0xFFFFFFFFU, COMMAND_MESSAGE_TYPE_ERROR, 0xFFFFU);
    expect_u32(test_listener_calls, 3U, "the consumer is woken for every queued result");
    expect_u32(test_listener_queued_at_call, 3U, "the consumer is woken after the result is in the queue");
    expect_u32(test_last_put_timeout, 0U, "the producer never waits for room");
    expect_u32(test_last_put_priority, 0U, "results use FIFO message priority");

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the first result is received");
    expect_u32(result.sequence, 10U, "the first sequence comes first");
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_DONE, "the first type is kept");
    expect_u32(result.code, 0U, "the first code is kept");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the second result is received");
    expect_u32(result.sequence, 11U, "the second sequence follows");
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_DONE, "the second type is kept");
    expect_u32(result.code, 1U, "the second code is kept");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the third result is received");
    expect_u32(result.sequence, 0xFFFFFFFFU, "the largest sequence is kept");
    expect_u32(result.response_type, 0xFFU, "an error type is kept");
    expect_u32(result.code, 0xFFFFU, "the largest code is kept");
    expect_u32(test_last_get_timeout, 0U, "the consumer never waits for a result");

    result = (CommandResult_t){.sequence = 0x55U, .response_type = 0x55U, .code = 0x55U};
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osErrorResource, "an empty queue has no result");
    expect_u32(result.sequence, 0x55U, "an empty receive leaves the output untouched");

    test_next_get_status = osErrorParameter;
    expect_u32(
        (uint32_t)command_result_queue_try_receive(&result),
        (uint32_t)osErrorParameter,
        "an unexpected RTOS error is passed on"
    );
}

static void test_plain_post_has_no_stop_details(void) {
    CommandResult_t result = {.cause = 0x55U, .axis = 0x55U, .detail = 0x55U};
    reset_initialized_fixture();

    command_result_queue_post(40U, COMMAND_MESSAGE_TYPE_ERROR, 0x0123U);
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the result is received");
    expect_u32(result.sequence, 40U, "the sequence is kept");
    expect_u32(result.response_type, 0xFFU, "the type is kept");
    expect_u32(result.code, 0x0123U, "the code is kept");
    expect_u32(result.cause, 0U, "a plain result has cause 0");
    expect_u32(result.axis, 0xFFU, "a plain result blames no axis");
    expect_u32(result.detail, 0U, "a plain result has detail 0");
}

static void test_detailed_post_keeps_the_stop_details(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    command_result_queue_set_listener(test_listener);

    command_result_queue_post_detailed(41U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    command_result_queue_post_detailed(42U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 0xA1U, 0U, 0xC3U);
    expect_u32(test_listener_calls, 2U, "a detailed result wakes the consumer like a plain one");
    expect_u32(test_listener_queued_at_call, 2U, "the consumer is woken after the result is in the queue");

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the first result is received");
    expect_u32(result.sequence, 41U, "the first sequence is kept");
    expect_u32(result.response_type, 0xFFU, "the first type is kept");
    expect_u32(result.code, 0x0128U, "the first code is kept");
    expect_u32(result.cause, 2U, "the first cause is kept");
    expect_u32(result.axis, 1U, "the first axis is kept");
    expect_u32(result.detail, 3U, "the first detail is kept");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the second result is received");
    expect_u32(result.sequence, 42U, "the second sequence is kept");
    expect_u32(result.cause, 0xA1U, "the second cause is kept");
    expect_u32(result.axis, 0U, "axis X stays 0 and is not turned into no axis");
    expect_u32(result.detail, 0xC3U, "the second detail is kept");

    /* A detailed result is dropped like a plain one when the queue does not take it. */
    test_next_put_status = osErrorResource;
    command_result_queue_post_detailed(43U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    expect_u32(test_listener_calls, 2U, "a detailed result that was not queued does not wake the consumer");
    expect_u32(test_queue.count, 0U, "a detailed result that was not queued adds nothing");
}

static void test_detailed_post_before_initialization_queues_nothing(void) {
    reset_counters();
    command_result_queue_set_listener(test_listener);
    command_result_queue_post_detailed(44U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    expect_u32(test_put_calls, 0U, "nothing is put into a queue that does not exist");
    expect_u32(test_listener_calls, 0U, "a result that was not queued does not wake the consumer");
    command_result_queue_set_listener(NULL);
}

static void test_full_queue_drops_new_results_and_keeps_old_ones(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    command_result_queue_set_listener(test_listener);

    for (uint32_t sequence = 1U; sequence <= COMMAND_RESULT_QUEUE_CAPACITY; ++sequence) {
        command_result_queue_post(sequence, COMMAND_MESSAGE_TYPE_ERROR, (uint16_t)(sequence + 1000U));
    }
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY, "every result that fit woke the consumer");

    command_result_queue_post(999U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY, "a result that does not fit does not wake the consumer");
    expect_u32(test_queue.count, COMMAND_RESULT_QUEUE_CAPACITY, "the full queue is unchanged");

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the oldest result is received");
    expect_u32(result.sequence, 1U, "the oldest result was not replaced");
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_ERROR, "the oldest type was not replaced");
    expect_u32(result.code, 1001U, "the oldest code was not replaced");

    /* A queue that has room but fails the put for another reason also drops the result. */
    test_next_put_status = osErrorParameter;
    command_result_queue_post(888U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY, "a failed put does not wake the consumer");
    expect_u32(test_queue.count, COMMAND_RESULT_QUEUE_CAPACITY - 1U, "a failed put adds nothing");

    command_result_queue_post(777U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY + 1U, "the result that fit wakes the consumer");

    for (uint32_t sequence = 2U; sequence <= COMMAND_RESULT_QUEUE_CAPACITY; ++sequence) {
        expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "a queued result is received");
        expect_u32(result.sequence, sequence, "results come out in the order they went in");
        expect_u32(result.code, sequence + 1000U, "each result keeps its own code");
    }
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the late result is received");
    expect_u32(result.sequence, 777U, "the late result comes last");
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_DONE, "the late result keeps its type");
    expect_u32(result.code, COMMAND_DONE_ALREADY_AT_TARGET, "the late result keeps its code");
    expect_u32(
        (uint32_t)command_result_queue_try_receive(&result),
        (uint32_t)osErrorResource,
        "neither the overflowing result nor the failed put was queued"
    );
}

static void test_discard_drops_every_queued_result(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();

    command_result_queue_post(20U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post(21U, COMMAND_MESSAGE_TYPE_ERROR, 0x0125U);
    command_result_queue_discard_all();
    expect_u32(test_reset_calls, 1U, "discard resets the RTOS queue");
    expect_u32(test_queue.count, 0U, "discard leaves no result");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osErrorResource, "no result survives a discard");

    command_result_queue_post(22U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the queue works after a discard");
    expect_u32(result.sequence, 22U, "only the result posted after the discard is received");
}

static void test_listener_can_be_removed(void) {
    reset_initialized_fixture();

    command_result_queue_post(30U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 0U, "a result without a listener is queued quietly");
    expect_u32(test_queue.count, 1U, "the result without a listener is queued");

    command_result_queue_set_listener(test_listener);
    command_result_queue_post(31U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 1U, "the listener is called after it is set");

    command_result_queue_set_listener(NULL);
    command_result_queue_post(32U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 1U, "a removed listener is not called");
    expect_u32(test_queue.count, 3U, "all three results are queued");
}

int main(void) {
    test_before_initialization_nothing_is_queued();
    test_detailed_post_before_initialization_queues_nothing();
    test_initialization();
    test_plain_post_has_no_stop_details();
    test_detailed_post_keeps_the_stop_details();
    test_results_keep_their_order_and_values();
    test_full_queue_drops_new_results_and_keeps_old_ones();
    test_discard_drops_every_queued_result();
    test_listener_can_be_removed();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command result queue unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command result queue unit tests passed.\n");
    return 0;
}

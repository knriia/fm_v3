#include "command/command_result_queue.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t capacity;
    uint32_t message_size;
    uint32_t count;
    CommandResult_t messages[COMMAND_RESULT_QUEUE_CAPACITY];
} TestMessageQueue_t;

/* The queue copies whole results: the fields are 4+1+2+1+1+1+4 = 14 bytes, so a result that grows past 20 bytes has got
 * a field nobody planned for, and the queue memory grows by CAPACITY times that. */
_Static_assert(sizeof(CommandResult_t) >= 14U && sizeof(CommandResult_t) <= 20U, "CommandResult_t has an unexpected size");

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

/* The epoch of the command stream belongs to the command queue; here the test sets it, like a STOP does. */
static uint32_t test_epoch;
uint32_t command_queue_epoch(void) { return test_epoch; }

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
    test_epoch = 0U;
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

    command_result_queue_post_in_epoch(0U,1U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
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
    command_result_queue_post_in_epoch(0U,2U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_put_calls, 0U, "a failed allocation leaves the module uninitialized");

    expect_u32((uint32_t)command_result_queue_init(), (uint32_t)osOK, "the queue initializes after allocation recovers");
    expect_u32(test_new_calls, 2U, "creation is attempted on each initialization");
    /* 100 queued commands + 128 moves that wait for their final result + 1. */
    expect_u32(test_queue.capacity, 229U, "there is room for the results of every queued command and waiting move");
    expect_u32(test_queue.capacity, COMMAND_RESULT_QUEUE_CAPACITY, "the queue is created with the declared capacity");
    expect_u32(
        test_queue.message_size,
        (uint32_t)sizeof(CommandResult_t),
        "the queue is created with messages of the size of a result"
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

    command_result_queue_post_in_epoch(0U,10U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(0U,11U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
    command_result_queue_post_in_epoch(0U,0xFFFFFFFFU, COMMAND_MESSAGE_TYPE_ERROR, 0xFFFFU);
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

    command_result_queue_post_in_epoch(0U,40U, COMMAND_MESSAGE_TYPE_ERROR, 0x0123U);
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

    command_result_queue_post_detailed_in_epoch(0U,41U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    command_result_queue_post_detailed_in_epoch(0U,42U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 0xA1U, 0U, 0xC3U);
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
    command_result_queue_post_detailed_in_epoch(0U,43U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    expect_u32(test_listener_calls, 2U, "a detailed result that was not queued does not wake the consumer");
    expect_u32(test_queue.count, 0U, "a detailed result that was not queued adds nothing");
}

static void test_detailed_post_before_initialization_queues_nothing(void) {
    reset_counters();
    command_result_queue_set_listener(test_listener);
    command_result_queue_post_detailed_in_epoch(0U,44U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    expect_u32(test_put_calls, 0U, "nothing is put into a queue that does not exist");
    expect_u32(test_listener_calls, 0U, "a result that was not queued does not wake the consumer");
    command_result_queue_set_listener(NULL);
}

static void test_full_queue_drops_new_results_and_keeps_old_ones(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    command_result_queue_set_listener(test_listener);

    for (uint32_t sequence = 1U; sequence <= COMMAND_RESULT_QUEUE_CAPACITY; ++sequence) {
        command_result_queue_post_in_epoch(0U,sequence, COMMAND_MESSAGE_TYPE_ERROR, (uint16_t)(sequence + 1000U));
    }
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY, "every result that fit woke the consumer");

    command_result_queue_post_in_epoch(0U,999U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY, "a result that does not fit does not wake the consumer");
    expect_u32(test_queue.count, COMMAND_RESULT_QUEUE_CAPACITY, "the full queue is unchanged");

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the oldest result is received");
    expect_u32(result.sequence, 1U, "the oldest result was not replaced");
    expect_u32(result.response_type, COMMAND_MESSAGE_TYPE_ERROR, "the oldest type was not replaced");
    expect_u32(result.code, 1001U, "the oldest code was not replaced");

    /* A queue that has room but fails the put for another reason also drops the result. */
    test_next_put_status = osErrorParameter;
    command_result_queue_post_in_epoch(0U,888U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, COMMAND_RESULT_QUEUE_CAPACITY, "a failed put does not wake the consumer");
    expect_u32(test_queue.count, COMMAND_RESULT_QUEUE_CAPACITY - 1U, "a failed put adds nothing");

    command_result_queue_post_in_epoch(0U,777U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
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

    command_result_queue_post_in_epoch(0U,20U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(0U,21U, COMMAND_MESSAGE_TYPE_ERROR, 0x0125U);
    command_result_queue_discard_all();
    expect_u32(test_reset_calls, 1U, "discard resets the RTOS queue");
    expect_u32(test_queue.count, 0U, "discard leaves no result");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osErrorResource, "no result survives a discard");

    command_result_queue_post_in_epoch(0U,22U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the queue works after a discard");
    expect_u32(result.sequence, 22U, "only the result posted after the discard is received");
}

static void test_listener_can_be_removed(void) {
    reset_initialized_fixture();

    command_result_queue_post_in_epoch(0U,30U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 0U, "a result without a listener is queued quietly");
    expect_u32(test_queue.count, 1U, "the result without a listener is queued");

    command_result_queue_set_listener(test_listener);
    command_result_queue_post_in_epoch(0U,31U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 1U, "the listener is called after it is set");

    command_result_queue_set_listener(NULL);
    command_result_queue_post_in_epoch(0U,32U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 1U, "a removed listener is not called");
    expect_u32(test_queue.count, 3U, "all three results are queued");
}

static void test_post_in_epoch_before_initialization_queues_nothing(void) {
    reset_counters();
    command_result_queue_set_listener(test_listener);
    command_result_queue_post_in_epoch(0U, 45U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_put_calls, 0U, "nothing is put into a queue that does not exist");
    expect_u32(test_listener_calls, 0U, "a result that was not queued does not wake the consumer");
    command_result_queue_set_listener(NULL);
}

static void test_every_post_stamps_the_epoch_of_the_result(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    test_epoch = 5U;

    command_result_queue_post_in_epoch(5U, 50U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_detailed_in_epoch(5U, 51U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    command_result_queue_post_in_epoch(5U, 52U, COMMAND_MESSAGE_TYPE_ERROR, 0x0124U);
    command_result_queue_post_in_epoch(4U, 53U, COMMAND_MESSAGE_TYPE_ERROR, 0x0124U);
    command_result_queue_post_detailed_in_epoch(4U, 54U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 2U, 1U, 3U);
    expect_u32(test_queue.count, 5U, "all five results are queued, the stale ones as well");
    /* What the queue holds, before the receiver filters it. */
    expect_u32(test_queue.messages[0].epoch, 5U, "a plain post carries the epoch it was given");
    expect_u32(test_queue.messages[1].epoch, 5U, "a detailed post carries the epoch it was given");
    expect_u32(test_queue.messages[2].epoch, 5U, "a second plain post carries its epoch");
    expect_u32(test_queue.messages[3].epoch, 4U, "a plain post in an older epoch carries the older epoch");
    expect_u32(test_queue.messages[4].epoch, 4U, "a detailed post in an older epoch carries the older epoch");

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the plain result is received");
    expect_u32(result.sequence, 50U, "the plain result comes first");
    expect_u32(result.epoch, 5U, "the received plain result has the current epoch");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the detailed result is received");
    expect_u32(result.sequence, 51U, "the detailed result comes next");
    expect_u32(result.epoch, 5U, "the received detailed result has the current epoch");
    expect_u32(result.cause, 2U, "the details are kept with the epoch");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the result of the current epoch is received");
    expect_u32(result.sequence, 52U, "the result posted in the current epoch comes next");
    expect_u32(result.response_type, 0xFFU, "its type is kept");
    expect_u32(result.code, 0x0124U, "its code is kept");
    expect_u32(result.cause, 0U, "a result posted in an epoch has cause 0");
    expect_u32(result.axis, 0xFFU, "a result posted in an epoch blames no axis");
    expect_u32(result.detail, 0U, "a result posted in an epoch has detail 0");
    expect_u32(result.epoch, 5U, "the received result keeps its epoch");
    expect_u32(
        (uint32_t)command_result_queue_try_receive(&result),
        (uint32_t)osErrorResource,
        "the result of the older epoch is not handed out"
    );
}

static void test_post_in_epoch_wakes_the_consumer_and_drops_on_a_full_queue(void) {
    reset_initialized_fixture();
    command_result_queue_set_listener(test_listener);
    command_result_queue_post_in_epoch(0U, 60U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 1U, "a queued result in an epoch wakes the consumer");
    expect_u32(test_listener_queued_at_call, 1U, "the consumer is woken after the result is in the queue");
    expect_u32(test_last_put_timeout, 0U, "the producer never waits for room");

    test_next_put_status = osErrorResource;
    command_result_queue_post_in_epoch(0U, 61U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(test_listener_calls, 1U, "a result that was not queued does not wake the consumer");
    expect_u32(test_queue.count, 1U, "a result that was not queued adds nothing");
    command_result_queue_set_listener(NULL);
}

static void test_receive_skips_the_results_of_a_stopped_epoch(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    test_epoch = 2U;
    command_result_queue_post_in_epoch(1U, 10U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(2U, 11U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
    command_result_queue_post_in_epoch(0U, 12U, COMMAND_MESSAGE_TYPE_ERROR, 0x0124U);
    command_result_queue_post_in_epoch(2U, 13U, COMMAND_MESSAGE_TYPE_ERROR, 0x0127U);

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "a result of the current epoch is found");
    expect_u32(result.sequence, 11U, "the stale result in front of it is skipped");
    expect_u32(result.code, COMMAND_DONE_ALREADY_AT_TARGET, "the result found keeps its code");
    expect_u32(test_get_calls, 2U, "one stale result was read and dropped, then the current one taken");
    expect_u32(test_queue.count, 2U, "the stale result is gone from the queue, the rest waits");

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the next call goes on");
    expect_u32(result.sequence, 13U, "the stale result between them is skipped as well");
    expect_u32(result.response_type, 0xFFU, "the result found keeps its type");
    expect_u32(result.code, 0x0127U, "the result found keeps its code");
    expect_u32(test_queue.count, 0U, "nothing is left");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osErrorResource, "the queue is empty now");
}

static void test_receive_finds_nothing_when_every_result_is_stale(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    test_epoch = 3U;
    command_result_queue_post_in_epoch(0U, 20U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(1U, 21U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(2U, 22U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);

    expect_u32(
        (uint32_t)command_result_queue_try_receive(&result),
        (uint32_t)osErrorResource,
        "a queue of stale results has no result to hand out"
    );
    expect_u32(test_queue.count, 0U, "every stale result was dropped");
    expect_u32(test_get_calls, 4U, "the three stale results and the end of the queue were read");

    command_result_queue_post_in_epoch(3U, 23U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the next result of the epoch is handed out");
    expect_u32(result.sequence, 23U, "it is the one posted after the stale ones");
}

static void test_receive_keeps_the_order_of_the_results_of_the_current_epoch(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    test_epoch = 4U;
    command_result_queue_post_in_epoch(4U, 1U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(3U, 2U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(4U, 3U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(2U, 4U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(4U, 5U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);

    const uint32_t expected_sequences[3] = {1U, 3U, 5U};
    for (uint32_t index = 0U; index < 3U; ++index) {
        expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "a current result is handed out");
        expect_u32(result.sequence, expected_sequences[index], "the results of the epoch keep their order");
    }
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osErrorResource, "no stale result follows");
}

/* A STOP raises the epoch: what was posted before it is void, what is posted after it is delivered. */
static void test_a_new_epoch_voids_the_results_posted_before_it(void) {
    CommandResult_t result = {0};
    reset_initialized_fixture();
    command_result_queue_post_in_epoch(0U, 30U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    command_result_queue_post_in_epoch(0U, 31U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    test_epoch = 1U;
    command_result_queue_post_in_epoch(1U, 32U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ALREADY_AT_TARGET);
    command_result_queue_post_detailed_in_epoch(1U, 33U, COMMAND_MESSAGE_TYPE_ERROR, 0x0128U, 4U, 2U, 7U);

    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "a result of the new epoch is handed out");
    expect_u32(result.sequence, 32U, "the results posted before the STOP are skipped");
    expect_u32(result.epoch, 1U, "the result carries the new epoch");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osOK, "the detailed result of the new epoch follows");
    expect_u32(result.sequence, 33U, "it comes after the plain one");
    expect_u32(result.detail, 7U, "its details are kept");
    expect_u32((uint32_t)command_result_queue_try_receive(&result), (uint32_t)osErrorResource, "the voided results are gone");

    /* A result posted for the old epoch after the raise is void as well. */
    command_result_queue_post_in_epoch(0U, 34U, COMMAND_MESSAGE_TYPE_DONE, COMMAND_DONE_ACCEPTED);
    expect_u32(
        (uint32_t)command_result_queue_try_receive(&result),
        (uint32_t)osErrorResource,
        "a late result of the old epoch is not handed out"
    );
}

int main(void) {
    test_before_initialization_nothing_is_queued();
    test_detailed_post_before_initialization_queues_nothing();
    test_post_in_epoch_before_initialization_queues_nothing();
    test_initialization();
    test_plain_post_has_no_stop_details();
    test_detailed_post_keeps_the_stop_details();
    test_results_keep_their_order_and_values();
    test_full_queue_drops_new_results_and_keeps_old_ones();
    test_discard_drops_every_queued_result();
    test_listener_can_be_removed();
    test_every_post_stamps_the_epoch_of_the_result();
    test_post_in_epoch_wakes_the_consumer_and_drops_on_a_full_queue();
    test_receive_skips_the_results_of_a_stopped_epoch();
    test_receive_finds_nothing_when_every_result_is_stale();
    test_receive_keeps_the_order_of_the_results_of_the_current_epoch();
    test_a_new_epoch_voids_the_results_posted_before_it();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Command result queue unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Command result queue unit tests passed.\n");
    return 0;
}

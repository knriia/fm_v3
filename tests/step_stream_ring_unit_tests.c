#include "motion/step_stream_ring.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static void fill_both(StepStreamRing_t *ring) {
    assert(step_stream_ring_mark_filled(ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_mark_filled(ring, 1U) == STEP_STREAM_RING_STATUS_OK);
}

static void test_start_requires_both_halves_filled(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_NOT_STARTED);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_NOT_READY);
    assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_NOT_READY);
    assert(step_stream_ring_mark_filled(&ring, 1U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_RUNNING);
    assert(ring.half_states[0] == STEP_STREAM_RING_HALF_PLAYING);
    assert(ring.half_states[1] == STEP_STREAM_RING_HALF_FULL);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_INVALID_STATE);
    assert(step_stream_ring_get_half_fill_count(&ring) == 2U);
}

static void test_many_cycles_without_underrun(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    fill_both(&ring);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);

    for (uint32_t cycle = 0U; cycle < 1000U; ++cycle) {
        uint8_t half = 0xFFU;
        assert(step_stream_ring_next_fill_request(&ring, &half) == STEP_STREAM_RING_STATUS_NO_FILL_REQUEST);

        assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_OK);
        assert(ring.half_states[1] == STEP_STREAM_RING_HALF_PLAYING);
        assert(step_stream_ring_next_fill_request(&ring, &half) == STEP_STREAM_RING_STATUS_OK);
        assert(half == 0U);
        assert(step_stream_ring_mark_filled(&ring, half) == STEP_STREAM_RING_STATUS_OK);

        assert(step_stream_ring_on_transfer_complete(&ring) == STEP_STREAM_RING_STATUS_OK);
        assert(ring.half_states[0] == STEP_STREAM_RING_HALF_PLAYING);
        assert(step_stream_ring_next_fill_request(&ring, &half) == STEP_STREAM_RING_STATUS_OK);
        assert(half == 1U);
        assert(step_stream_ring_mark_filled(&ring, half) == STEP_STREAM_RING_STATUS_OK);
    }
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_RUNNING);
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);
    assert(step_stream_ring_get_half_transfer_count(&ring) == 2000U);
    assert(step_stream_ring_get_half_fill_count(&ring) == 2002U);
}

static void test_late_generator_is_a_sticky_underrun(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    fill_both(&ring);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_OK);

    /* Half 0 was not refilled when the DMA wraps into it. */
    assert(step_stream_ring_on_transfer_complete(&ring) == STEP_STREAM_RING_STATUS_UNDERRUN);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_UNDERRUN);
    assert(step_stream_ring_get_underrun_count(&ring) == 1U);
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_UNDERRUN);
    assert(step_stream_ring_on_transfer_complete(&ring) == STEP_STREAM_RING_STATUS_UNDERRUN);
    assert(step_stream_ring_get_underrun_count(&ring) == 1U);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_INVALID_STATE);

    step_stream_ring_init(&ring);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_NOT_STARTED);
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);
}

static void test_late_generator_on_half_transfer(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    fill_both(&ring);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);

    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_on_transfer_complete(&ring) == STEP_STREAM_RING_STATUS_OK);
    /* Half 1 was not refilled when the DMA enters it again. */
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_UNDERRUN);
    assert(step_stream_ring_get_underrun_count(&ring) == 1U);
}

static void test_events_out_of_order_are_a_sticky_sequence_error(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    fill_both(&ring);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);

    /* Half 0 plays, so TC cannot come first. */
    assert(step_stream_ring_on_transfer_complete(&ring) == STEP_STREAM_RING_STATUS_SEQUENCE_ERROR);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_SEQUENCE_ERROR);
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_SEQUENCE_ERROR);
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);

    step_stream_ring_init(&ring);
    fill_both(&ring);
    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    /* Half 1 plays, so a second HT is out of order. */
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_SEQUENCE_ERROR);
}

static void test_events_before_start_are_rejected(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    assert(step_stream_ring_on_half_transfer(&ring) == STEP_STREAM_RING_STATUS_INVALID_STATE);
    assert(step_stream_ring_on_transfer_complete(&ring) == STEP_STREAM_RING_STATUS_INVALID_STATE);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_NOT_STARTED);
    assert(step_stream_ring_get_half_transfer_count(&ring) == 0U);
}

static void test_mark_filled_and_fill_request_guards(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    uint8_t half = 0xFFU;

    assert(step_stream_ring_next_fill_request(&ring, &half) == STEP_STREAM_RING_STATUS_OK && half == 0U);
    assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_next_fill_request(&ring, &half) == STEP_STREAM_RING_STATUS_OK && half == 1U);
    assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_INVALID_STATE);
    assert(step_stream_ring_mark_filled(&ring, 1U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_next_fill_request(&ring, &half) == STEP_STREAM_RING_STATUS_NO_FILL_REQUEST);

    assert(step_stream_ring_start(&ring) == STEP_STREAM_RING_STATUS_OK);
    /* A half the DMA plays cannot be filled. */
    assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_INVALID_STATE);
}

static void test_invalid_arguments(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    uint8_t half = 0U;

    assert(step_stream_ring_mark_filled(NULL, 0U) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
    assert(step_stream_ring_mark_filled(&ring, STEP_STREAM_RING_HALF_COUNT) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
    assert(step_stream_ring_start(NULL) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
    assert(step_stream_ring_on_half_transfer(NULL) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
    assert(step_stream_ring_on_transfer_complete(NULL) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
    assert(step_stream_ring_next_fill_request(NULL, &half) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
    assert(step_stream_ring_next_fill_request(&ring, NULL) == STEP_STREAM_RING_STATUS_INVALID_ARGUMENT);
}

int main(void) {
    test_start_requires_both_halves_filled();
    test_many_cycles_without_underrun();
    test_late_generator_is_a_sticky_underrun();
    test_late_generator_on_half_transfer();
    test_events_out_of_order_are_a_sticky_sequence_error();
    test_events_before_start_are_rejected();
    test_mark_filled_and_fill_request_guards();
    test_invalid_arguments();
    puts("step_stream_ring_unit_tests: PASS");
    return 0;
}

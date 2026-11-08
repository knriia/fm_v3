#include "step_stream_axis_logic.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* 10 records: 5 per half, 30 DMA transfers in all (3 registers per record). */
#define TEST_RECORD_COUNT 10U

static uint32_t fake_pending_script[16];
static uint32_t fake_pending_count;
static uint32_t fake_pending_calls;
static uint32_t fake_ndtr_value;
static uint32_t fake_ndtr_calls;

static uint32_t fake_pending(void) {
    const uint32_t index = fake_pending_calls < fake_pending_count ? fake_pending_calls : fake_pending_count - 1U;
    ++fake_pending_calls;
    return fake_pending_script[index];
}

static uint32_t fake_ndtr(void) {
    ++fake_ndtr_calls;
    return fake_ndtr_value;
}

static const StepStreamAxisSnapshotSource_t fake_source = {fake_pending, fake_ndtr};

static void set_pending_script(const uint32_t *values, uint32_t count) {
    for (uint32_t i = 0U; i < count; ++i) {
        fake_pending_script[i] = values[i];
    }
    fake_pending_count = count;
    fake_pending_calls = 0U;
    fake_ndtr_calls = 0U;
}

static void test_prepare_stand_ring(void) {
    StepStreamRing_t ring;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_RUNNING);
    assert(ring.half_states[0] == STEP_STREAM_RING_HALF_PLAYING);
    assert(ring.half_states[1] == STEP_STREAM_RING_HALF_FULL);

    /* A ring left in a fault by the previous run is cleared by the preparation. */
    ring.state = STEP_STREAM_RING_STATE_UNDERRUN;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_RUNNING);
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);
}

static void test_stand_mode_refills_every_half(void) {
    StepStreamRing_t ring;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    for (uint32_t cycle = 0U; cycle < 100U; ++cycle) {
        assert(step_stream_axis_logic_half_event(&ring, 1U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
        assert(ring.half_states[0] == STEP_STREAM_RING_HALF_FULL);
        assert(step_stream_axis_logic_half_event(&ring, 0U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
        assert(ring.half_states[1] == STEP_STREAM_RING_HALF_FULL);
    }
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);
}

static void test_stream_mode_leaves_half_empty_and_underruns(void) {
    StepStreamRing_t ring;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    assert(step_stream_axis_logic_half_event(&ring, 1U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
    assert(ring.half_states[0] == STEP_STREAM_RING_HALF_EMPTY);
    /* The task refilled nothing yet; half 0 is still EMPTY when TC makes the DMA enter it. */
    assert(step_stream_axis_logic_half_event(&ring, 0U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN);
    assert(step_stream_ring_get_underrun_count(&ring) == 1U);
    /* The fault is sticky, so a late repeated event is still an underrun. */
    assert(step_stream_axis_logic_half_event(&ring, 1U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN);
}

static void test_stream_mode_task_keeps_up(void) {
    StepStreamRing_t ring;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    for (uint32_t cycle = 0U; cycle < 50U; ++cycle) {
        assert(step_stream_axis_logic_half_event(&ring, 1U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
        assert(step_stream_ring_mark_filled(&ring, 0U) == STEP_STREAM_RING_STATUS_OK);
        assert(step_stream_axis_logic_half_event(&ring, 0U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
        assert(step_stream_ring_mark_filled(&ring, 1U) == STEP_STREAM_RING_STATUS_OK);
    }
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);
}

static void test_sequence_faults(void) {
    StepStreamRing_t ring;
    step_stream_ring_init(&ring);
    /* Events before the start are a sequence defect. */
    assert(step_stream_axis_logic_half_event(&ring, 1U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_SEQUENCE);

    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    /* TC arrives while half 0 is still the playing one: HT was lost or the order is broken. */
    assert(step_stream_axis_logic_half_event(&ring, 0U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_SEQUENCE);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_SEQUENCE_ERROR);
    assert(step_stream_axis_logic_half_event(&ring, 1U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_SEQUENCE);
}

static void test_simultaneous_ht_and_tc(void) {
    StepStreamRing_t ring;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    /* The interrupt ran late and both flags were set: HT is handled first, then TC. */
    assert(step_stream_axis_logic_half_event(&ring, 1U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
    assert(step_stream_axis_logic_half_event(&ring, 0U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
    assert(ring.current_half == 0U);
    /* The reversed order is detected instead of being miscounted. */
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    assert(step_stream_axis_logic_half_event(&ring, 0U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_SEQUENCE);
}

static void test_restart_after_fault_clears_state(void) {
    StepStreamRing_t ring;
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    assert(step_stream_axis_logic_half_event(&ring, 1U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
    assert(step_stream_axis_logic_half_event(&ring, 0U, 0U) == STEP_STREAM_AXIS_LOGIC_ACTION_FAULT_UNDERRUN);
    assert(step_stream_axis_logic_prepare_stand_ring(&ring) == 0U);
    assert(step_stream_ring_get_state(&ring) == STEP_STREAM_RING_STATE_RUNNING);
    assert(step_stream_ring_get_underrun_count(&ring) == 0U);
    assert(step_stream_axis_logic_half_event(&ring, 1U, 1U) == STEP_STREAM_AXIS_LOGIC_ACTION_RELEASED);
}

static void test_read_steps_stable_snapshot(void) {
    const uint32_t stable[] = {0U, 0U};
    set_pending_script(stable, 2U);
    fake_ndtr_value = 20U;
    /* 10 transfers of 30 are played: 3 records, one of them still in the preload. */
    assert(step_stream_axis_logic_read_steps(&fake_source, 0U, TEST_RECORD_COUNT) == 2U);
    assert(fake_pending_calls == 2U);
    assert(fake_ndtr_calls == 1U);
}

static void test_read_steps_retries_when_a_flag_appears(void) {
    /* TC lands between the two flag reads of the first attempt; the second attempt is stable. */
    const uint32_t changing[] = {0U, 1U, 1U, 1U};
    set_pending_script(changing, 4U);
    fake_ndtr_value = 15U;
    /* One completed half (the pending event) at its boundary: 5 records, minus the preload. */
    assert(step_stream_axis_logic_read_steps(&fake_source, 0U, TEST_RECORD_COUNT) == 4U);
    assert(fake_pending_calls == 4U);
    assert(fake_ndtr_calls == 2U);
}

static void test_read_steps_gives_up_after_the_attempt_limit(void) {
    /* Every attempt differs, so the retries end at the limit and the last values are used. */
    const uint32_t endless[] = {0U, 1U, 0U, 1U, 0U, 1U, 0U, 1U};
    set_pending_script(endless, 8U);
    fake_ndtr_value = 15U;
    assert(step_stream_axis_logic_read_steps(&fake_source, 0U, TEST_RECORD_COUNT) == 4U);
    assert(fake_pending_calls == 8U);
    assert(fake_ndtr_calls == 4U);
}

static void test_boundary_steps(void) {
    assert(step_stream_axis_logic_boundary_steps(0U, TEST_RECORD_COUNT) == 0U);
    assert(step_stream_axis_logic_boundary_steps(1U, TEST_RECORD_COUNT) == 4U);
    assert(step_stream_axis_logic_boundary_steps(2U, TEST_RECORD_COUNT) == 9U);
    assert(step_stream_axis_logic_boundary_steps(3U, TEST_RECORD_COUNT) == 14U);
}

static void test_stale_steps(void) {
    assert(step_stream_axis_logic_stale_steps(12U, 9U) == 3U);
    assert(step_stream_axis_logic_stale_steps(9U, 9U) == 0U);
    assert(step_stream_axis_logic_stale_steps(7U, 9U) == 0U);
}

static uint32_t poll_calls_until_done;
static uint32_t poll_calls;

static uint8_t poll_is_done(void) { return ++poll_calls >= poll_calls_until_done ? 1U : 0U; }

static void test_poll(void) {
    poll_calls = 0U;
    poll_calls_until_done = 1U;
    assert(step_stream_axis_logic_poll(poll_is_done, 5U) == 1U);
    assert(poll_calls == 1U);

    poll_calls = 0U;
    poll_calls_until_done = 5U;
    assert(step_stream_axis_logic_poll(poll_is_done, 5U) == 1U);
    assert(poll_calls == 5U);

    poll_calls = 0U;
    poll_calls_until_done = 6U;
    assert(step_stream_axis_logic_poll(poll_is_done, 5U) == 0U);
    assert(poll_calls == 5U);

    poll_calls = 0U;
    assert(step_stream_axis_logic_poll(poll_is_done, 0U) == 0U);
    assert(poll_calls == 0U);
}

int main(void) {
    test_prepare_stand_ring();
    test_stand_mode_refills_every_half();
    test_stream_mode_leaves_half_empty_and_underruns();
    test_stream_mode_task_keeps_up();
    test_sequence_faults();
    test_simultaneous_ht_and_tc();
    test_restart_after_fault_clears_state();
    test_read_steps_stable_snapshot();
    test_read_steps_retries_when_a_flag_appears();
    test_read_steps_gives_up_after_the_attempt_limit();
    test_boundary_steps();
    test_stale_steps();
    test_poll();
    (void)printf("Step stream axis logic unit tests passed.\n");
    return 0;
}

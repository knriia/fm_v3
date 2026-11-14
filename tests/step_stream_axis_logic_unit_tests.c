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
static int fake_context_marker;
static uint32_t fake_wrong_context_calls;

static void check_source_context(const void *context) {
    if (context != &fake_context_marker) {
        ++fake_wrong_context_calls;
    }
}

static uint32_t fake_pending(const void *context) {
    check_source_context(context);
    const uint32_t index = fake_pending_calls < fake_pending_count ? fake_pending_calls : fake_pending_count - 1U;
    ++fake_pending_calls;
    return fake_pending_script[index];
}

static uint32_t fake_ndtr(const void *context) {
    check_source_context(context);
    ++fake_ndtr_calls;
    return fake_ndtr_value;
}

static const StepStreamAxisSnapshotSource_t fake_source = {fake_pending, fake_ndtr, &fake_context_marker};

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
    fake_wrong_context_calls = 0U;
    set_pending_script(stable, 2U);
    fake_ndtr_value = 20U;
    /* 10 transfers of 30 are played: 3 records, one of them still in the preload. */
    assert(step_stream_axis_logic_read_steps(&fake_source, 0U, TEST_RECORD_COUNT) == 2U);
    assert(fake_pending_calls == 2U);
    assert(fake_ndtr_calls == 1U);
    assert(fake_wrong_context_calls == 0U);
}

static void test_read_steps_retries_when_a_flag_appears(void) {
    /* TC lands between the two flag reads of the first attempt; the second attempt is stable. */
    const uint32_t changing[] = {0U, 1U, 1U, 1U};
    fake_wrong_context_calls = 0U;
    set_pending_script(changing, 4U);
    fake_ndtr_value = 15U;
    /* One completed half (the pending event) at its boundary: 5 records, minus the preload. */
    assert(step_stream_axis_logic_read_steps(&fake_source, 0U, TEST_RECORD_COUNT) == 4U);
    assert(fake_pending_calls == 4U);
    assert(fake_ndtr_calls == 2U);
    assert(fake_wrong_context_calls == 0U);
}

static void test_read_steps_gives_up_after_the_attempt_limit(void) {
    /* Every attempt differs, so the retries end at the limit and the last values are used. */
    const uint32_t endless[] = {0U, 1U, 0U, 1U, 0U, 1U, 0U, 1U};
    fake_wrong_context_calls = 0U;
    set_pending_script(endless, 8U);
    fake_ndtr_value = 15U;
    assert(step_stream_axis_logic_read_steps(&fake_source, 0U, TEST_RECORD_COUNT) == 4U);
    assert(fake_pending_calls == 8U);
    assert(fake_ndtr_calls == 4U);
    assert(fake_wrong_context_calls == 0U);
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
static int poll_context_marker;
static uint32_t poll_wrong_context_calls;

static uint8_t poll_is_done(const void *context) {
    if (context != &poll_context_marker) {
        ++poll_wrong_context_calls;
    }
    return ++poll_calls >= poll_calls_until_done ? 1U : 0U;
}

static void test_poll(void) {
    poll_wrong_context_calls = 0U;
    poll_calls = 0U;
    poll_calls_until_done = 1U;
    assert(step_stream_axis_logic_poll(poll_is_done, &poll_context_marker, 5U) == 1U);
    assert(poll_calls == 1U);

    poll_calls = 0U;
    poll_calls_until_done = 5U;
    assert(step_stream_axis_logic_poll(poll_is_done, &poll_context_marker, 5U) == 1U);
    assert(poll_calls == 5U);

    poll_calls = 0U;
    poll_calls_until_done = 6U;
    assert(step_stream_axis_logic_poll(poll_is_done, &poll_context_marker, 5U) == 0U);
    assert(poll_calls == 5U);

    poll_calls = 0U;
    assert(step_stream_axis_logic_poll(poll_is_done, &poll_context_marker, 0U) == 0U);
    assert(poll_calls == 0U);
    assert(poll_wrong_context_calls == 0U);
}

/* ---- Braking on a limit switch ---- */

#define BRAKE_RECORD_COUNT 40U
#define BRAKE_HALF_RECORDS (BRAKE_RECORD_COUNT / 2U)
#define BRAKE_MARGIN 3U
#define BRAKE_WORDS 3U

static uint16_t brake_table_storage[64];
static StepStreamBrake_t brake_table;
static uint16_t brake_buffer[BRAKE_RECORD_COUNT * BRAKE_WORDS];
static StepStreamRing_t brake_ring;
static StepStreamAxisBrake_t brake_state;
/* The EMPTY half that the task holds when the braking begins. */
static uint8_t brake_held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;

static uint16_t brake_cruise_arr(void) { return brake_table.arr[0]; }

static void brake_fixture(void) {
    const StepStreamBrakeConfig_t config = {
        .timer_clock_hz = 275000000U,
        .pulse_ticks = 1375U,
        .min_period_ticks = 2063U,
        .idle_record_ticks = 2750U,
        .deceleration_steps_per_s2 = 200000U,
        .max_step_rate_hz = 5000U,
    };
    assert(step_stream_brake_init(&brake_table, &config, brake_table_storage, 64U) == STEP_STREAM_BRAKE_STATUS_OK);
    assert(brake_table.count > 12U && brake_table.count < 32U);

    /* The old stream cruises at the top of the ramp: every record is a step. */
    for (uint32_t record = 0U; record < BRAKE_RECORD_COUNT; ++record) {
        brake_buffer[record * BRAKE_WORDS] = brake_cruise_arr();
        brake_buffer[record * BRAKE_WORDS + 1U] = 0U;
        brake_buffer[record * BRAKE_WORDS + 2U] = 1375U;
    }

    /* Stream mode: both halves FULL, half 0 PLAYING. */
    step_stream_ring_init(&brake_ring);
    assert(step_stream_ring_mark_filled(&brake_ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_mark_filled(&brake_ring, 1U) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_start(&brake_ring) == STEP_STREAM_RING_STATUS_OK);
    step_stream_axis_logic_brake_reset(&brake_state);
    brake_held_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
}

static StepStreamAxisBrakeBegin_t brake_begin(uint32_t steps_done) {
    return step_stream_axis_logic_brake_begin(
        &brake_state,
        &brake_table,
        &brake_ring,
        brake_buffer,
        BRAKE_RECORD_COUNT,
        steps_done,
        BRAKE_MARGIN,
        brake_held_half
    );
}

static void test_brake_reset(void) {
    step_stream_axis_logic_brake_reset(NULL);
    brake_fixture();
    brake_state.active = 1U;
    brake_state.half_has_steps[0] = 1U;
    brake_state.deferred_half = 1U;
    step_stream_axis_logic_brake_reset(&brake_state);
    assert(brake_state.active == 0U);
    assert(brake_state.half_has_steps[0] == 0U && brake_state.half_has_steps[1] == 0U);
    assert(brake_state.deferred_half == STEP_STREAM_AXIS_LOGIC_NO_HALF);
    assert(brake_state.cursor.brake == NULL && brake_state.cursor.next == 0U);
}

static void test_brake_in_the_playing_half(void) {
    brake_fixture();
    /* steps_done 4: five records are with the timer, the ramp starts after three more, at record 8. */
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_state.active == 1U);
    assert(brake_state.deferred_half == STEP_STREAM_AXIS_LOGIC_NO_HALF);
    assert(brake_state.half_has_steps[0] == 1U && brake_state.half_has_steps[1] == 1U);

    /* Records before the ramp are untouched; the ramp follows without a gap into the other half. */
    for (uint32_t record = 0U; record < 8U; ++record) {
        assert(brake_buffer[record * BRAKE_WORDS] == brake_cruise_arr());
    }
    for (uint32_t index = 0U; index < brake_table.count; ++index) {
        const uint32_t record = 8U + index;
        assert(brake_buffer[record * BRAKE_WORDS] == brake_table.arr[index]);
        assert(brake_buffer[record * BRAKE_WORDS + 2U] == 1375U);
    }
    for (uint32_t record = 8U + brake_table.count; record < BRAKE_RECORD_COUNT; ++record) {
        assert(brake_buffer[record * BRAKE_WORDS] == 2749U);
        assert(brake_buffer[record * BRAKE_WORDS + 2U] == 0U);
    }
    assert(brake_state.cursor.next == brake_table.count);

    /* HT: half 0 is played, the ring releases it, the interrupt refills it with filler; half 1 still has the ramp. */
    assert(step_stream_ring_on_half_transfer(&brake_ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_CONTINUE);
    assert(brake_ring.half_states[0] == STEP_STREAM_RING_HALF_FULL);
    assert(brake_state.half_has_steps[0] == 0U);
    assert(brake_buffer[0] == 2749U && brake_buffer[2] == 0U);

    /* TC: half 1 is played too; no half carries a step any more. */
    assert(step_stream_ring_on_transfer_complete(&brake_ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 1U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_COMPLETE);
}

static void test_brake_ramp_longer_than_one_refill(void) {
    brake_fixture();
    /* Record 8 on: 12 records in half 0, 20 in half 1, so the first 32 ramp entries are placed at once. A longer ramp
     * is continued by the refills. */
    StepStreamBrakeConfig_t config = {
        .timer_clock_hz = 275000000U,
        .pulse_ticks = 1375U,
        .min_period_ticks = 2063U,
        .idle_record_ticks = 2750U,
        .deceleration_steps_per_s2 = 100000U,
        .max_step_rate_hz = 5000U,
    };
    assert(step_stream_brake_init(&brake_table, &config, brake_table_storage, 64U) == STEP_STREAM_BRAKE_STATUS_OK);
    assert(brake_table.count > 32U);
    for (uint32_t record = 0U; record < BRAKE_RECORD_COUNT; ++record) {
        brake_buffer[record * BRAKE_WORDS] = brake_cruise_arr();
        brake_buffer[record * BRAKE_WORDS + 2U] = 1375U;
    }

    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_state.cursor.next == 32U);

    assert(step_stream_ring_on_half_transfer(&brake_ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_CONTINUE);
    /* The rest of the ramp continues in the refilled half 0, right after the entry that ended half 1. */
    assert(brake_buffer[0] == brake_table.arr[32]);
    assert(brake_state.half_has_steps[0] == 1U);
    assert(brake_state.cursor.next == brake_table.count);
}

static void test_brake_defers_a_half_the_task_may_hold(void) {
    brake_fixture();
    /* The task holds half 1 and has not committed it yet. */
    brake_ring.half_states[1] = STEP_STREAM_RING_HALF_EMPTY;
    brake_held_half = 1U;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_state.deferred_half == 1U);
    assert(brake_state.cursor.next == 12U);
    assert(brake_ring.half_states[1] == STEP_STREAM_RING_HALF_EMPTY);

    /* Whatever the task wrote into half 1 is replaced by the rest of the ramp at its commit. */
    for (uint32_t word = 0U; word < BRAKE_HALF_RECORDS * BRAKE_WORDS; ++word) {
        brake_buffer[BRAKE_HALF_RECORDS * BRAKE_WORDS + word] = 0x5A5AU;
    }
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) == 0U);
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 1U) == 1U);
    assert(brake_ring.half_states[1] == STEP_STREAM_RING_HALF_FULL);
    assert(brake_state.deferred_half == STEP_STREAM_AXIS_LOGIC_NO_HALF);
    assert(brake_state.half_has_steps[1] == 1U);
    assert(brake_buffer[BRAKE_HALF_RECORDS * BRAKE_WORDS] == brake_table.arr[12]);
    assert(brake_buffer[BRAKE_HALF_RECORDS * BRAKE_WORDS + 2U] == 1375U);
    /* A second commit of the same half is ignored. */
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 1U) == 0U);
}

static void test_brake_fills_an_empty_half_nobody_holds(void) {
    brake_fixture();
    /* Half 1 was released and the task has not taken it yet: no commit will come for it, so the braking fills it. */
    brake_ring.half_states[1] = STEP_STREAM_RING_HALF_EMPTY;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_state.deferred_half == STEP_STREAM_AXIS_LOGIC_NO_HALF);
    assert(brake_ring.half_states[1] == STEP_STREAM_RING_HALF_FULL);
    assert(brake_state.half_has_steps[1] == 1U);
    assert(brake_buffer[BRAKE_HALF_RECORDS * BRAKE_WORDS] == brake_table.arr[12]);

    /* The task holds the other half, not this one: this one is still filled here. */
    brake_fixture();
    brake_ring.half_states[1] = STEP_STREAM_RING_HALF_EMPTY;
    brake_held_half = 0U;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_state.deferred_half == STEP_STREAM_AXIS_LOGIC_NO_HALF);
    assert(brake_ring.half_states[1] == STEP_STREAM_RING_HALF_FULL);
}

static void test_brake_commit_rejects_what_is_not_deferred(void) {
    brake_fixture();
    assert(step_stream_axis_logic_brake_commit(NULL, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 1U) == 0U);
    assert(step_stream_axis_logic_brake_commit(&brake_state, NULL, brake_buffer, BRAKE_RECORD_COUNT, 1U) == 0U);
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, NULL, BRAKE_RECORD_COUNT, 1U) == 0U);
    /* Not braking. */
    brake_state.deferred_half = 1U;
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 1U) == 0U);

    /* Braking with no deferred half, and a half number outside the ring. */
    brake_state.active = 1U;
    brake_state.deferred_half = STEP_STREAM_AXIS_LOGIC_NO_HALF;
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) == 0U);
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0xFFU) == 0U);

    /* A deferred half that the ring no longer has EMPTY is not announced FULL. */
    step_stream_brake_cursor_init(&brake_state.cursor, &brake_table, 0U);
    brake_state.deferred_half = 0U;
    assert(brake_ring.half_states[0] == STEP_STREAM_RING_HALF_PLAYING);
    assert(step_stream_axis_logic_brake_commit(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) == 0U);
}

static void test_brake_in_the_other_half(void) {
    brake_fixture();
    /* steps_done 17: 18 records are with the timer, the ramp starts at record 21 in half 1. */
    assert(brake_begin(17U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_state.deferred_half == STEP_STREAM_AXIS_LOGIC_NO_HALF);
    assert(brake_state.half_has_steps[1] == 1U);
    assert(brake_buffer[21U * BRAKE_WORDS] == brake_table.arr[0]);
    /* Half 0 keeps its old records: the DMA plays them before it reaches the ramp. */
    assert(brake_buffer[0] == brake_cruise_arr());
    assert(brake_buffer[2] == 1375U);
}

static void test_brake_wraps_into_the_first_half(void) {
    brake_fixture();
    assert(step_stream_ring_on_half_transfer(&brake_ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_mark_filled(&brake_ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    /* steps_done 38: 39 records are with the timer, half 1 plays, the ramp starts at record 2 of half 0. */
    assert(brake_begin(38U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(brake_buffer[2U * BRAKE_WORDS] == brake_table.arr[0]);
    assert(brake_state.half_has_steps[0] == 1U);
    /* A stream that has gone around more than once reaches the same position. */
    brake_fixture();
    assert(step_stream_ring_on_half_transfer(&brake_ring) == STEP_STREAM_RING_STATUS_OK);
    assert(step_stream_ring_mark_filled(&brake_ring, 0U) == STEP_STREAM_RING_STATUS_OK);
    assert(brake_begin(38U + 3U * BRAKE_RECORD_COUNT) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
}

static void test_brake_is_refused_when_nothing_can_be_placed(void) {
    brake_fixture();
    /* The ramp would start in half 1, which the task has not filled. */
    brake_ring.half_states[1] = STEP_STREAM_RING_HALF_EMPTY;
    assert(brake_begin(17U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW);
    assert(brake_state.active == 0U);

    /* The step before the ramp is a filler: the axis does not step there. */
    brake_fixture();
    brake_buffer[7U * BRAKE_WORDS + 2U] = 0U;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW);

    /* The step before the ramp is already slower than the whole ramp. */
    brake_fixture();
    brake_buffer[7U * BRAKE_WORDS] = UINT16_MAX;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STOP_NOW);
}

static void test_brake_waits_for_an_unhandled_half_event(void) {
    brake_fixture();
    /* 20 records are with the timer, so the DMA is in half 1 while the ring still has half 0 playing. */
    assert(brake_begin(19U) == STEP_STREAM_AXIS_LOGIC_BRAKE_RETRY);
    assert(brake_state.active == 0U);
}

static void test_brake_rejects_inconsistent_input(void) {
    brake_fixture();
    assert(step_stream_axis_logic_brake_begin(
               NULL, &brake_table, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 4U, BRAKE_MARGIN, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(
               &brake_state, NULL, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 4U, BRAKE_MARGIN, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    StepStreamBrake_t empty = {0};
    assert(step_stream_axis_logic_brake_begin(
               &brake_state, &empty, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 4U, BRAKE_MARGIN, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(
               &brake_state, &brake_table, NULL, brake_buffer, BRAKE_RECORD_COUNT, 4U, BRAKE_MARGIN, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(
               &brake_state, &brake_table, &brake_ring, NULL, BRAKE_RECORD_COUNT, 4U, BRAKE_MARGIN, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(&brake_state, &brake_table, &brake_ring, brake_buffer, 2U, 4U, 1U, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(&brake_state, &brake_table, &brake_ring, brake_buffer, 41U, 4U, BRAKE_MARGIN, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(
               &brake_state, &brake_table, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 4U, 0U, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
    assert(step_stream_axis_logic_brake_begin(
               &brake_state, &brake_table, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 4U, BRAKE_HALF_RECORDS, STEP_STREAM_AXIS_LOGIC_NO_HALF) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);

    /* A ring that is not running. */
    step_stream_ring_init(&brake_ring);
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);

    /* The half the ring calls playing is not PLAYING. */
    brake_fixture();
    brake_ring.half_states[0] = STEP_STREAM_RING_HALF_EMPTY;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);

    /* The other half is PLAYING too. */
    brake_fixture();
    brake_ring.half_states[1] = STEP_STREAM_RING_HALF_PLAYING;
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_FAULT);
}

static void test_brake_half_release_rejects_inconsistent_input(void) {
    brake_fixture();
    assert(brake_begin(4U) == STEP_STREAM_AXIS_LOGIC_BRAKE_STARTED);
    assert(step_stream_ring_on_half_transfer(&brake_ring) == STEP_STREAM_RING_STATUS_OK);

    assert(step_stream_axis_logic_brake_half_released(NULL, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, NULL, brake_buffer, BRAKE_RECORD_COUNT, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, NULL, BRAKE_RECORD_COUNT, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 2U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, 2U, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, 41U, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);

    /* Not braking. */
    StepStreamAxisBrake_t idle;
    step_stream_axis_logic_brake_reset(&idle);
    assert(step_stream_axis_logic_brake_half_released(&idle, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 0U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);

    /* The released half is not EMPTY in the ring: the half is playing, not released. */
    assert(step_stream_axis_logic_brake_half_released(&brake_state, &brake_ring, brake_buffer, BRAKE_RECORD_COUNT, 1U) ==
           STEP_STREAM_AXIS_LOGIC_BRAKE_EVENT_FAULT);
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
    test_brake_reset();
    test_brake_in_the_playing_half();
    test_brake_ramp_longer_than_one_refill();
    test_brake_defers_a_half_the_task_may_hold();
    test_brake_fills_an_empty_half_nobody_holds();
    test_brake_commit_rejects_what_is_not_deferred();
    test_brake_in_the_other_half();
    test_brake_wraps_into_the_first_half();
    test_brake_is_refused_when_nothing_can_be_placed();
    test_brake_waits_for_an_unhandled_half_event();
    test_brake_rejects_inconsistent_input();
    test_brake_half_release_rejects_inconsistent_input();
    (void)printf("Step stream axis logic unit tests passed.\n");
    return 0;
}

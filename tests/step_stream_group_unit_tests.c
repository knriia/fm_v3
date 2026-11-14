#include "motion/step_stream_group.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_MEMBERS 3U
#define TEST_STARTUP_NS 2500U
#define TEST_CALL_LOG_SIZE 64U

typedef enum {
    CALL_PREPARE = 1,
    CALL_ARM,
    CALL_TRIGGER,
    CALL_STOP,
    CALL_BRAKE,
    CALL_LISTENER_STOPPING,
    CALL_LISTENER_FAULT
} CallKind_t;

typedef struct {
    CallKind_t kind;
    uint32_t member;
    uint32_t value;
} Call_t;

static Call_t call_log[TEST_CALL_LOG_SIZE];
static uint32_t call_count;
static uint8_t member_active[TEST_MEMBERS];
static uint8_t prepare_fails_at_member;
static uint8_t arm_fails_at_member;
static uint32_t critical_depth;
static uint32_t critical_enters;
static uint32_t critical_exits;
static uint32_t critical_violations;
static StepStreamGroupState_t state_seen_at_trigger;
static StepStreamGroup_t group;
static int ports_context_marker;
static int listener_context_marker;
static uint32_t wrong_context_calls;
/* Runs once inside the next arm, stop or is_active call: lets a test play an interrupt or another task that comes in
 * the middle of an operation of the group. */
static void (*port_hook)(void);

#define NO_FAILURE 0xFFU
#define CRITICAL_TOKEN_BASE 0x1000U

static void log_call(CallKind_t kind, uint32_t member, uint32_t value) {
    assert(call_count < TEST_CALL_LOG_SIZE);
    call_log[call_count++] = (Call_t){kind, member, value};
}

static void check_ports_context(void *context) {
    if (context != &ports_context_marker) {
        ++wrong_context_calls;
    }
}

/* Runs inside the arm of one member: another task or an interrupt gets in while the group is arming. */
static void (*arm_hook)(void);
static uint32_t arm_hook_member;

static void run_port_hook(void) {
    if (port_hook != NULL) {
        void (*const hook)(void) = port_hook;
        port_hook = NULL;
        hook();
    }
}

/* The ports must be called with the section left: they take their own, and an interrupt must not be blocked longer
 * than the state change needs. */
static void check_outside_critical(void) {
    if (critical_depth != 0U) {
        ++critical_violations;
    }
}

static StepStreamGroupPortResult_t port_prepare(void *context, uint32_t member) {
    check_ports_context(context);
    check_outside_critical();
    log_call(CALL_PREPARE, member, 0U);
    return member == prepare_fails_at_member ? STEP_STREAM_GROUP_PORT_ERROR : STEP_STREAM_GROUP_PORT_OK;
}

static StepStreamGroupPortResult_t port_arm(void *context, uint32_t member, uint32_t startup_period_ns) {
    check_ports_context(context);
    check_outside_critical();
    run_port_hook();
    if (arm_hook != NULL && member == arm_hook_member) {
        void (*const hook)(void) = arm_hook;
        arm_hook = NULL;
        hook();
    }
    log_call(CALL_ARM, member, startup_period_ns);
    if (member == arm_fails_at_member) {
        return STEP_STREAM_GROUP_PORT_ERROR;
    }
    member_active[member] = 1U;
    return STEP_STREAM_GROUP_PORT_OK;
}

static void port_trigger(void *context) {
    check_ports_context(context);
    check_outside_critical();
    state_seen_at_trigger = step_stream_group_get_state(&group);
    log_call(CALL_TRIGGER, 0U, 0U);
}

static void port_stop(void *context, uint32_t member) {
    check_ports_context(context);
    check_outside_critical();
    run_port_hook();
    log_call(CALL_STOP, member, 0U);
    member_active[member] = 0U;
}

static void port_brake(void *context, uint32_t member) {
    check_ports_context(context);
    check_outside_critical();
    log_call(CALL_BRAKE, member, 0U);
}

static uint8_t port_is_active(void *context, uint32_t member) {
    check_ports_context(context);
    check_outside_critical();
    run_port_hook();
    return member_active[member];
}

static uint32_t port_enter_critical(void *context) {
    check_ports_context(context);
    ++critical_depth;
    ++critical_enters;
    return CRITICAL_TOKEN_BASE + critical_enters;
}

static void port_exit_critical(void *context, uint32_t saved) {
    check_ports_context(context);
    /* Exit must hand back what the matching enter returned. */
    if (critical_depth == 0U || saved != CRITICAL_TOKEN_BASE + critical_enters) {
        ++critical_violations;
    }
    if (critical_depth > 0U) {
        --critical_depth;
    }
    ++critical_exits;
}

static void listener_stopping(void *context, uint32_t member) {
    if (context != &listener_context_marker) {
        ++wrong_context_calls;
    }
    log_call(CALL_LISTENER_STOPPING, member, 0U);
}

static void listener_fault(void *context, uint32_t member, uint32_t fault) {
    if (context != &listener_context_marker) {
        ++wrong_context_calls;
    }
    log_call(CALL_LISTENER_FAULT, member, fault);
}

static StepStreamGroupPorts_t make_ports(void) {
    return (StepStreamGroupPorts_t){
        .prepare = port_prepare,
        .arm = port_arm,
        .trigger = port_trigger,
        .stop = port_stop,
        .brake = port_brake,
        .is_active = port_is_active,
        .enter_critical = port_enter_critical,
        .exit_critical = port_exit_critical,
        .context = &ports_context_marker,
    };
}

static void reset_fixture(void) {
    memset(call_log, 0, sizeof(call_log));
    memset(member_active, 0, sizeof(member_active));
    call_count = 0U;
    prepare_fails_at_member = NO_FAILURE;
    arm_fails_at_member = NO_FAILURE;
    critical_depth = 0U;
    critical_enters = 0U;
    critical_exits = 0U;
    critical_violations = 0U;
    state_seen_at_trigger = STEP_STREAM_GROUP_STATE_IDLE;
    wrong_context_calls = 0U;
    port_hook = NULL;
    arm_hook = NULL;
    arm_hook_member = 0U;
    const StepStreamGroupPorts_t ports = make_ports();
    const StepStreamGroupListener_t listener = {listener_stopping, listener_fault, &listener_context_marker};
    assert(step_stream_group_init(&group, TEST_MEMBERS, &ports, &listener) == STEP_STREAM_GROUP_STATUS_OK);
}

static void expect_call(uint32_t index, CallKind_t kind, uint32_t member, uint32_t value) {
    assert(index < call_count);
    assert(call_log[index].kind == kind);
    assert(call_log[index].member == member);
    assert(call_log[index].value == value);
}

static uint32_t count_calls(CallKind_t kind) {
    uint32_t count = 0U;
    for (uint32_t index = 0U; index < call_count; ++index) {
        count += call_log[index].kind == kind ? 1U : 0U;
    }
    return count;
}

/* Brings the group to RUNNING with every member active, then forgets the calls of getting there. */
static void run_group(void) {
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
    call_count = 0U;
}

static void test_init_rejects_bad_arguments(void) {
    StepStreamGroup_t local;
    const StepStreamGroupPorts_t ports = make_ports();
    assert(step_stream_group_init(NULL, 1U, &ports, NULL) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_init(&local, 1U, NULL, NULL) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_init(&local, 0U, &ports, NULL) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(
        step_stream_group_init(&local, STEP_STREAM_GROUP_MAX_MEMBERS + 1U, &ports, NULL) ==
        STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT
    );

    StepStreamGroupPorts_t broken[8];
    for (uint32_t index = 0U; index < 8U; ++index) {
        broken[index] = ports;
    }
    broken[0].prepare = NULL;
    broken[1].arm = NULL;
    broken[2].trigger = NULL;
    broken[3].stop = NULL;
    broken[4].brake = NULL;
    broken[5].is_active = NULL;
    broken[6].enter_critical = NULL;
    broken[7].exit_critical = NULL;
    for (uint32_t index = 0U; index < 8U; ++index) {
        assert(step_stream_group_init(&local, 1U, &broken[index], NULL) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    }

    assert(step_stream_group_init(&local, 1U, &ports, NULL) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_get_state(&local) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(local.mode == STEP_STREAM_GROUP_MODE_PLANNED);
    assert(step_stream_group_init(&local, STEP_STREAM_GROUP_MAX_MEMBERS, &ports, NULL) == STEP_STREAM_GROUP_STATUS_OK);
}

static void test_null_group_is_harmless(void) {
    /* A zeroed group is one that was never initialized: no ports, so no call may reach them. */
    StepStreamGroup_t blank;
    memset(&blank, 0, sizeof(blank));
    assert(
        step_stream_group_set_mode(&blank, STEP_STREAM_GROUP_MODE_PLANNED) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT
    );
    assert(step_stream_group_prepare(&blank) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_start(&blank, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_reset(&blank) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_service(&blank) == STEP_STREAM_GROUP_STATE_IDLE);
    step_stream_group_stop(&blank);
    step_stream_group_on_limit(&blank, 0U);
    step_stream_group_on_member_braking(&blank, 0U);
    step_stream_group_on_member_fault(&blank, 0U, 1U);
    assert(step_stream_group_get_state(&blank) == STEP_STREAM_GROUP_STATE_IDLE);

    assert(
        step_stream_group_set_mode(NULL, STEP_STREAM_GROUP_MODE_PLANNED) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT
    );
    assert(step_stream_group_prepare(NULL) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_start(NULL, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_reset(NULL) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(step_stream_group_service(NULL) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_get_state(NULL) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_get_fault(NULL) == 0U);
    assert(step_stream_group_get_fault_member(NULL) == 0U);
    step_stream_group_stop(NULL);
    step_stream_group_on_limit(NULL, 0U);
    step_stream_group_on_member_braking(NULL, 0U);
    step_stream_group_on_member_fault(NULL, 0U, 1U);
}

static void test_mode_changes_only_while_idle(void) {
    reset_fixture();
    assert(step_stream_group_set_mode(&group, STEP_STREAM_GROUP_MODE_INDEPENDENT) == STEP_STREAM_GROUP_STATUS_OK);
    assert(group.mode == STEP_STREAM_GROUP_MODE_INDEPENDENT);
    assert(step_stream_group_set_mode(&group, (StepStreamGroupMode_t)7) == STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT);
    assert(group.mode == STEP_STREAM_GROUP_MODE_INDEPENDENT);

    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_set_mode(&group, STEP_STREAM_GROUP_MODE_PLANNED) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    assert(group.mode == STEP_STREAM_GROUP_MODE_INDEPENDENT);
    assert(critical_violations == 0U);
    assert(critical_depth == 0U);
}

static void test_prepare_resets_every_ring(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(call_count == TEST_MEMBERS);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        expect_call(member, CALL_PREPARE, member, 0U);
    }
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_PRIMING);

    call_count = 0U;
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    assert(call_count == 0U);
    assert(wrong_context_calls == 0U);
}

static void test_prepare_failure_leaves_the_group_idle(void) {
    reset_fixture();
    prepare_fails_at_member = 1U;
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_MEMBER_ERROR);
    /* The members after the failed one are not touched. */
    assert(call_count == 2U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);
}

static void test_start_arms_every_member_then_triggers_once(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;

    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_OK);
    assert(call_count == TEST_MEMBERS + 1U);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        expect_call(member, CALL_ARM, member, TEST_STARTUP_NS);
    }
    expect_call(TEST_MEMBERS, CALL_TRIGGER, 0U, 0U);
    /* The state is already RUNNING when the channels start: an interrupt right after the trigger finds it so. */
    assert(state_seen_at_trigger == STEP_STREAM_GROUP_STATE_RUNNING);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
    assert(critical_violations == 0U);
    assert(critical_depth == 0U);
}

static void test_start_needs_a_prepared_group(void) {
    reset_fixture();
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    assert(call_count == 0U);
    run_group();
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    assert(call_count == 0U);
}

static void test_start_failure_stops_the_armed_members(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;
    arm_fails_at_member = 1U;

    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_MEMBER_ERROR);
    expect_call(0U, CALL_ARM, 0U, TEST_STARTUP_NS);
    expect_call(1U, CALL_ARM, 1U, TEST_STARTUP_NS);
    assert(count_calls(CALL_TRIGGER) == 0U);
    assert(count_calls(CALL_STOP) == TEST_MEMBERS);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        assert(member_active[member] == 0U);
    }
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);
}

static void test_stop_stops_every_member_and_goes_idle(void) {
    reset_fixture();
    run_group();
    step_stream_group_stop(&group);
    assert(call_count == TEST_MEMBERS);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        expect_call(member, CALL_STOP, member, 0U);
        assert(member_active[member] == 0U);
    }
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);

    /* From PRIMING too: a stop before the start leaves the group ready to prepare again. */
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    step_stream_group_stop(&group);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(critical_violations == 0U);
}

static void test_stop_keeps_a_fault(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_member_fault(&group, 2U, 5U);
    step_stream_group_stop(&group);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault(&group) == 5U);
}

static void test_planned_limit_brakes_every_member(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_limit(&group, 1U);
    assert(call_count == TEST_MEMBERS);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        expect_call(member, CALL_BRAKE, member, 0U);
    }
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_STOPPING);

    /* The second switch of the same move brakes nothing again. */
    call_count = 0U;
    step_stream_group_on_limit(&group, 2U);
    assert(call_count == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_STOPPING);
    assert(critical_violations == 0U);
    assert(critical_depth == 0U);
}

static void test_independent_limit_brakes_only_its_member(void) {
    reset_fixture();
    assert(step_stream_group_set_mode(&group, STEP_STREAM_GROUP_MODE_INDEPENDENT) == STEP_STREAM_GROUP_STATUS_OK);
    run_group();
    step_stream_group_on_limit(&group, 1U);
    assert(call_count == 1U);
    expect_call(0U, CALL_BRAKE, 1U, 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);

    /* The others reach their switches later and brake the same way. */
    step_stream_group_on_limit(&group, 0U);
    assert(call_count == 2U);
    expect_call(1U, CALL_BRAKE, 0U, 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
}

static void test_limit_is_ignored_unless_running(void) {
    reset_fixture();
    step_stream_group_on_limit(&group, 0U);
    assert(call_count == 0U);

    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;
    step_stream_group_on_limit(&group, 0U);
    assert(call_count == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_PRIMING);

    step_stream_group_stop(&group);
    run_group();
    step_stream_group_on_limit(&group, TEST_MEMBERS);
    assert(call_count == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);

    step_stream_group_on_member_fault(&group, 0U, 1U);
    call_count = 0U;
    step_stream_group_on_limit(&group, 1U);
    assert(call_count == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
}

static void test_braking_is_forwarded_to_the_listener(void) {
    reset_fixture();
    step_stream_group_on_member_braking(&group, 2U);
    assert(call_count == 1U);
    expect_call(0U, CALL_LISTENER_STOPPING, 2U, 0U);
    assert(wrong_context_calls == 0U);

    call_count = 0U;
    step_stream_group_on_member_braking(&group, TEST_MEMBERS);
    assert(call_count == 0U);

    StepStreamGroup_t quiet;
    const StepStreamGroupPorts_t ports = make_ports();
    assert(step_stream_group_init(&quiet, 1U, &ports, NULL) == STEP_STREAM_GROUP_STATUS_OK);
    step_stream_group_on_member_braking(&quiet, 0U);
    assert(call_count == 0U);
}

static void test_fault_stops_the_others_and_is_reported_once(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_member_fault(&group, 1U, 7U);

    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault_member(&group) == 1U);
    assert(step_stream_group_get_fault(&group) == 7U);
    /* The failed channel stopped itself, so only the others get a stop. */
    assert(call_count == TEST_MEMBERS);
    expect_call(0U, CALL_STOP, 0U, 0U);
    expect_call(1U, CALL_STOP, 2U, 0U);
    expect_call(2U, CALL_LISTENER_FAULT, 1U, 7U);
    assert(wrong_context_calls == 0U);

    /* A second failure, e.g. of a channel that was stopped meanwhile, keeps the first. */
    call_count = 0U;
    step_stream_group_on_member_fault(&group, 2U, 9U);
    assert(call_count == 0U);
    assert(step_stream_group_get_fault_member(&group) == 1U);
    assert(step_stream_group_get_fault(&group) == 7U);
    assert(critical_violations == 0U);
    assert(critical_depth == 0U);
}

static void test_fault_while_braking_wins(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_limit(&group, 0U);
    call_count = 0U;
    step_stream_group_on_member_fault(&group, 2U, 3U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault_member(&group) == 2U);
    assert(step_stream_group_get_fault(&group) == 3U);
    assert(call_count == 3U);
    expect_call(0U, CALL_STOP, 0U, 0U);
    expect_call(1U, CALL_STOP, 1U, 0U);
    expect_call(2U, CALL_LISTENER_FAULT, 2U, 3U);
}

static void test_fault_is_ignored_when_idle_and_for_unknown_members(void) {
    reset_fixture();
    step_stream_group_on_member_fault(&group, 0U, 1U);
    assert(call_count == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);

    run_group();
    step_stream_group_on_member_fault(&group, TEST_MEMBERS, 1U);
    assert(call_count == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
}

static void test_fault_while_priming_stops_the_group(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;
    step_stream_group_on_member_fault(&group, 0U, 2U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(count_calls(CALL_STOP) == TEST_MEMBERS - 1U);
}

static void test_fault_without_a_listener(void) {
    StepStreamGroup_t quiet;
    reset_fixture();
    const StepStreamGroupPorts_t ports = make_ports();
    assert(step_stream_group_init(&quiet, 2U, &ports, NULL) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_prepare(&quiet) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_start(&quiet, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_OK);
    step_stream_group_on_member_fault(&quiet, 0U, 4U);
    assert(step_stream_group_get_state(&quiet) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault(&quiet) == 4U);
}

static void test_service_waits_for_the_members(void) {
    reset_fixture();
    run_group();
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_RUNNING);

    member_active[0] = 0U;
    member_active[1] = 0U;
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_RUNNING);

    member_active[2] = 0U;
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);
}

static void fault_of_member_zero(void) { step_stream_group_on_member_fault(&group, 0U, 8U); }

static void test_service_keeps_a_fault_that_comes_meanwhile(void) {
    reset_fixture();
    run_group();
    memset(member_active, 0, sizeof(member_active));
    /* The fault arrives after service has seen the group RUNNING and before it has looked at the members. */
    port_hook = fault_of_member_zero;
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault(&group) == 8U);
    assert(critical_violations == 0U);
}

static void test_service_ends_a_braking(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_limit(&group, 0U);
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_STOPPING);
    memset(member_active, 0, sizeof(member_active));
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_IDLE);
}

static void test_service_leaves_other_states_alone(void) {
    reset_fixture();
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_PRIMING);

    step_stream_group_stop(&group);
    run_group();
    step_stream_group_on_member_fault(&group, 0U, 1U);
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(critical_violations == 0U);
}

static void test_reset_clears_a_fault_once_the_members_stopped(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_member_fault(&group, 1U, 6U);
    /* The failed channel stopped itself before it reported. */
    member_active[1] = 0U;
    member_active[0] = 1U;
    assert(step_stream_group_reset(&group) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault(&group) == 6U);

    member_active[0] = 0U;
    assert(step_stream_group_reset(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_get_fault(&group) == 0U);
    assert(step_stream_group_get_fault_member(&group) == 0U);
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
}

static void stop_the_group(void) { step_stream_group_stop(&group); }

static void fault_of_member_one(void) { step_stream_group_on_member_fault(&group, 1U, 4U); }

static void test_stop_during_start_wins(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;
    /* Another task stops the group while the second channel is being armed. */
    arm_hook_member = 1U;
    arm_hook = stop_the_group;
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    arm_hook = NULL;
    assert(count_calls(CALL_TRIGGER) == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        assert(member_active[member] == 0U);
    }
    assert(critical_violations == 0U);
}

static void test_fault_during_start_wins(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;
    arm_hook_member = 2U;
    arm_hook = fault_of_member_one;
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    arm_hook = NULL;
    assert(count_calls(CALL_TRIGGER) == 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault_member(&group) == 1U);
    assert(step_stream_group_get_fault(&group) == 4U);
    assert(count_calls(CALL_LISTENER_FAULT) == 1U);
}

static void test_failed_arm_does_not_overwrite_a_fault(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    arm_fails_at_member = 2U;
    arm_hook_member = 2U;
    arm_hook = fault_of_member_one;
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_MEMBER_ERROR);
    arm_hook = NULL;
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
}

static void test_stop_does_not_overwrite_a_fault_that_comes_meanwhile(void) {
    reset_fixture();
    run_group();
    port_hook = fault_of_member_one;
    step_stream_group_stop(&group);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_FAULT);
    assert(step_stream_group_get_fault(&group) == 4U);
    assert(critical_violations == 0U);
}

static void test_limit_before_running_is_applied_after_the_start(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    call_count = 0U;
    /* The switch is reached while the channels are armed but the group is still PRIMING. */
    step_stream_group_on_limit(&group, 1U);
    assert(call_count == 0U);

    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_OK);
    /* Arm x3, trigger, then the switch as if it had just come: every channel brakes (planned move). */
    assert(call_count == TEST_MEMBERS + 1U + TEST_MEMBERS);
    expect_call(TEST_MEMBERS, CALL_TRIGGER, 0U, 0U);
    for (uint32_t member = 0U; member < TEST_MEMBERS; ++member) {
        expect_call(TEST_MEMBERS + 1U + member, CALL_BRAKE, member, 0U);
    }
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_STOPPING);
}

static void test_independent_limit_before_running_brakes_only_its_member(void) {
    reset_fixture();
    assert(step_stream_group_set_mode(&group, STEP_STREAM_GROUP_MODE_INDEPENDENT) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    step_stream_group_on_limit(&group, 2U);
    call_count = 0U;
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_OK);
    assert(count_calls(CALL_BRAKE) == 1U);
    expect_call(TEST_MEMBERS + 1U, CALL_BRAKE, 2U, 0U);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
}

static void test_a_pending_limit_does_not_outlive_its_stream(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    step_stream_group_on_limit(&group, 0U);
    step_stream_group_stop(&group);
    run_group();
    assert(count_calls(CALL_BRAKE) == 0U);

    /* Prepare forgets it too, even without a stop in between (e.g. after a reset). */
    step_stream_group_stop(&group);
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    step_stream_group_on_limit(&group, 1U);
    assert(step_stream_group_reset(&group) == STEP_STREAM_GROUP_STATUS_OK);
    run_group();
    assert(count_calls(CALL_BRAKE) == 0U);
}

static void test_reset_is_valid_in_every_state_without_active_members(void) {
    reset_fixture();
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_reset(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);

    run_group();
    memset(member_active, 0, sizeof(member_active));
    assert(step_stream_group_reset(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_IDLE);

    run_group();
    assert(step_stream_group_reset(&group) == STEP_STREAM_GROUP_STATUS_BAD_STATE);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
}

static void test_restart_after_a_braking(void) {
    reset_fixture();
    run_group();
    step_stream_group_on_limit(&group, 0U);
    memset(member_active, 0, sizeof(member_active));
    assert(step_stream_group_service(&group) == STEP_STREAM_GROUP_STATE_IDLE);
    assert(step_stream_group_prepare(&group) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_start(&group, TEST_STARTUP_NS) == STEP_STREAM_GROUP_STATUS_OK);
    assert(step_stream_group_get_state(&group) == STEP_STREAM_GROUP_STATE_RUNNING);
}

int main(void) {
    test_init_rejects_bad_arguments();
    test_null_group_is_harmless();
    test_mode_changes_only_while_idle();
    test_prepare_resets_every_ring();
    test_prepare_failure_leaves_the_group_idle();
    test_start_arms_every_member_then_triggers_once();
    test_start_needs_a_prepared_group();
    test_start_failure_stops_the_armed_members();
    test_stop_stops_every_member_and_goes_idle();
    test_stop_keeps_a_fault();
    test_planned_limit_brakes_every_member();
    test_independent_limit_brakes_only_its_member();
    test_limit_is_ignored_unless_running();
    test_braking_is_forwarded_to_the_listener();
    test_fault_stops_the_others_and_is_reported_once();
    test_fault_while_braking_wins();
    test_fault_is_ignored_when_idle_and_for_unknown_members();
    test_fault_while_priming_stops_the_group();
    test_fault_without_a_listener();
    test_service_waits_for_the_members();
    test_service_keeps_a_fault_that_comes_meanwhile();
    test_service_ends_a_braking();
    test_service_leaves_other_states_alone();
    test_reset_clears_a_fault_once_the_members_stopped();
    test_stop_during_start_wins();
    test_fault_during_start_wins();
    test_failed_arm_does_not_overwrite_a_fault();
    test_stop_does_not_overwrite_a_fault_that_comes_meanwhile();
    test_limit_before_running_is_applied_after_the_start();
    test_independent_limit_before_running_brakes_only_its_member();
    test_a_pending_limit_does_not_outlive_its_stream();
    test_reset_is_valid_in_every_state_without_active_members();
    test_restart_after_a_braking();
    (void)printf("step_stream_group tests passed\n");
    return 0;
}

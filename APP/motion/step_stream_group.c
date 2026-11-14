#include "motion/step_stream_group.h"

#include <stddef.h>

static uint8_t step_stream_group_ports_are_complete(const StepStreamGroupPorts_t *ports) {
    return (uint8_t)(ports->prepare != NULL && ports->arm != NULL && ports->trigger != NULL && ports->stop != NULL &&
                     ports->brake != NULL && ports->is_active != NULL && ports->enter_critical != NULL &&
                     ports->exit_critical != NULL);
}

StepStreamGroupStatus_t step_stream_group_init(
    StepStreamGroup_t *group,
    uint32_t member_count,
    const StepStreamGroupPorts_t *ports,
    const StepStreamGroupListener_t *listener
) {
    if (group == NULL || ports == NULL || member_count == 0U || member_count > STEP_STREAM_GROUP_MAX_MEMBERS ||
        step_stream_group_ports_are_complete(ports) == 0U) {
        return STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT;
    }
    *group = (StepStreamGroup_t){
        .ports = *ports,
        .listener = listener != NULL ? *listener : (StepStreamGroupListener_t){0},
        .member_count = member_count,
        .mode = STEP_STREAM_GROUP_MODE_PLANNED,
        .state = STEP_STREAM_GROUP_STATE_IDLE,
    };
    return STEP_STREAM_GROUP_STATUS_OK;
}

/* A group that was never initialized (zeroed static) has no ports: every call is then a no-op or refused. */
static uint8_t step_stream_group_is_ready(const StepStreamGroup_t *group) {
    return (uint8_t)(group != NULL && group->member_count != 0U);
}

static uint32_t step_stream_group_enter(const StepStreamGroup_t *group) {
    return group->ports.enter_critical(group->ports.context);
}

static void step_stream_group_exit(const StepStreamGroup_t *group, uint32_t saved) {
    group->ports.exit_critical(group->ports.context, saved);
}

static void step_stream_group_stop_members(const StepStreamGroup_t *group, uint32_t except_member) {
    for (uint32_t member = 0U; member < group->member_count; ++member) {
        if (member != except_member) {
            group->ports.stop(group->ports.context, member);
        }
    }
}

static uint8_t step_stream_group_any_active(const StepStreamGroup_t *group) {
    for (uint32_t member = 0U; member < group->member_count; ++member) {
        if (group->ports.is_active(group->ports.context, member) != 0U) {
            return 1U;
        }
    }
    return 0U;
}

StepStreamGroupStatus_t step_stream_group_set_mode(StepStreamGroup_t *group, StepStreamGroupMode_t mode) {
    if (step_stream_group_is_ready(group) == 0U ||
        (mode != STEP_STREAM_GROUP_MODE_PLANNED && mode != STEP_STREAM_GROUP_MODE_INDEPENDENT)) {
        return STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT;
    }
    const uint32_t saved = step_stream_group_enter(group);
    StepStreamGroupStatus_t status = STEP_STREAM_GROUP_STATUS_BAD_STATE;
    if (group->state == STEP_STREAM_GROUP_STATE_IDLE) {
        group->mode = mode;
        status = STEP_STREAM_GROUP_STATUS_OK;
    }
    step_stream_group_exit(group, saved);
    return status;
}

StepStreamGroupStatus_t step_stream_group_prepare(StepStreamGroup_t *group) {
    if (step_stream_group_is_ready(group) == 0U) {
        return STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT;
    }
    if (group->state != STEP_STREAM_GROUP_STATE_IDLE) {
        return STEP_STREAM_GROUP_STATUS_BAD_STATE;
    }
    /* IDLE: no channel runs and no interrupt of the group fires, so the rings can be reset outside the section. */
    for (uint32_t member = 0U; member < group->member_count; ++member) {
        if (group->ports.prepare(group->ports.context, member) != STEP_STREAM_GROUP_PORT_OK) {
            return STEP_STREAM_GROUP_STATUS_MEMBER_ERROR;
        }
    }
    const uint32_t saved = step_stream_group_enter(group);
    group->pending_limits = 0U;
    group->state = STEP_STREAM_GROUP_STATE_PRIMING;
    step_stream_group_exit(group, saved);
    return STEP_STREAM_GROUP_STATUS_OK;
}

StepStreamGroupStatus_t step_stream_group_start(StepStreamGroup_t *group, uint32_t startup_period_ns) {
    if (step_stream_group_is_ready(group) == 0U) {
        return STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT;
    }
    if (group->state != STEP_STREAM_GROUP_STATE_PRIMING) {
        return STEP_STREAM_GROUP_STATUS_BAD_STATE;
    }
    for (uint32_t member = 0U; member < group->member_count; ++member) {
        if (group->ports.arm(group->ports.context, member, startup_period_ns) != STEP_STREAM_GROUP_PORT_OK) {
            /* Channels armed so far must not be left waiting for a trigger that never comes. */
            step_stream_group_stop_members(group, group->member_count);
            const uint32_t saved = step_stream_group_enter(group);
            /* A stop or a fault that came meanwhile has already settled the state. */
            if (group->state == STEP_STREAM_GROUP_STATE_PRIMING) {
                group->state = STEP_STREAM_GROUP_STATE_IDLE;
            }
            step_stream_group_exit(group, saved);
            return STEP_STREAM_GROUP_STATUS_MEMBER_ERROR;
        }
    }
    /* RUNNING before the trigger: a limit switch that is already active is seen by the first interrupt after it. */
    uint32_t saved = step_stream_group_enter(group);
    const uint8_t still_priming = group->state == STEP_STREAM_GROUP_STATE_PRIMING ? 1U : 0U;
    uint32_t pending_limits = 0U;
    if (still_priming != 0U) {
        group->state = STEP_STREAM_GROUP_STATE_RUNNING;
        pending_limits = group->pending_limits;
        group->pending_limits = 0U;
    }
    step_stream_group_exit(group, saved);
    if (still_priming == 0U) {
        /* A stop or a fault came while the channels were armed: they must not wait for a trigger. */
        step_stream_group_stop_members(group, group->member_count);
        return STEP_STREAM_GROUP_STATUS_BAD_STATE;
    }
    group->ports.trigger(group->ports.context);
    /* A switch reached between arm and RUNNING is handled as if it had come now. */
    for (uint32_t member = 0U; member < group->member_count; ++member) {
        if ((pending_limits & (1UL << member)) != 0U) {
            step_stream_group_on_limit(group, member);
        }
    }
    return STEP_STREAM_GROUP_STATUS_OK;
}

void step_stream_group_stop(StepStreamGroup_t *group) {
    if (step_stream_group_is_ready(group) == 0U) {
        return;
    }
    step_stream_group_stop_members(group, group->member_count);
    const uint32_t saved = step_stream_group_enter(group);
    group->pending_limits = 0U;
    if (group->state != STEP_STREAM_GROUP_STATE_FAULT) {
        group->state = STEP_STREAM_GROUP_STATE_IDLE;
    }
    step_stream_group_exit(group, saved);
}

StepStreamGroupState_t step_stream_group_service(StepStreamGroup_t *group) {
    if (step_stream_group_is_ready(group) == 0U) {
        return STEP_STREAM_GROUP_STATE_IDLE;
    }
    const StepStreamGroupState_t state = group->state;
    if ((state == STEP_STREAM_GROUP_STATE_RUNNING || state == STEP_STREAM_GROUP_STATE_STOPPING) &&
        step_stream_group_any_active(group) == 0U) {
        const uint32_t saved = step_stream_group_enter(group);
        /* A fault that came meanwhile stays. */
        if (group->state == state) {
            group->state = STEP_STREAM_GROUP_STATE_IDLE;
        }
        step_stream_group_exit(group, saved);
    }
    return group->state;
}

StepStreamGroupStatus_t step_stream_group_reset(StepStreamGroup_t *group) {
    if (step_stream_group_is_ready(group) == 0U) {
        return STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT;
    }
    if (step_stream_group_any_active(group) != 0U) {
        return STEP_STREAM_GROUP_STATUS_BAD_STATE;
    }
    const uint32_t saved = step_stream_group_enter(group);
    group->state = STEP_STREAM_GROUP_STATE_IDLE;
    group->pending_limits = 0U;
    group->fault_member = 0U;
    group->fault = 0U;
    step_stream_group_exit(group, saved);
    return STEP_STREAM_GROUP_STATUS_OK;
}

StepStreamGroupState_t step_stream_group_get_state(const StepStreamGroup_t *group) {
    return group != NULL ? group->state : STEP_STREAM_GROUP_STATE_IDLE;
}

uint32_t step_stream_group_get_fault_member(const StepStreamGroup_t *group) {
    return group != NULL ? group->fault_member : 0U;
}

uint32_t step_stream_group_get_fault(const StepStreamGroup_t *group) { return group != NULL ? group->fault : 0U; }

void step_stream_group_on_limit(StepStreamGroup_t *group, uint32_t member) {
    if (step_stream_group_is_ready(group) == 0U || member >= group->member_count) {
        return;
    }
    const uint32_t saved = step_stream_group_enter(group);
    if (group->state == STEP_STREAM_GROUP_STATE_PRIMING) {
        /* The channels may already be armed: the switch is applied when the group starts running. */
        group->pending_limits |= 1UL << member;
    }
    const uint8_t running = group->state == STEP_STREAM_GROUP_STATE_RUNNING ? 1U : 0U;
    const uint8_t planned = group->mode == STEP_STREAM_GROUP_MODE_PLANNED ? 1U : 0U;
    if (running != 0U && planned != 0U) {
        group->state = STEP_STREAM_GROUP_STATE_STOPPING;
    }
    step_stream_group_exit(group, saved);
    if (running == 0U) {
        return;
    }
    if (planned == 0U) {
        group->ports.brake(group->ports.context, member);
        return;
    }
    for (uint32_t other = 0U; other < group->member_count; ++other) {
        group->ports.brake(group->ports.context, other);
    }
}

void step_stream_group_on_member_braking(StepStreamGroup_t *group, uint32_t member) {
    if (step_stream_group_is_ready(group) == 0U || member >= group->member_count ||
        group->listener.on_member_stopping == NULL) {
        return;
    }
    group->listener.on_member_stopping(group->listener.context, member);
}

void step_stream_group_on_member_fault(StepStreamGroup_t *group, uint32_t member, uint32_t fault) {
    if (step_stream_group_is_ready(group) == 0U || member >= group->member_count) {
        return;
    }
    const uint32_t saved = step_stream_group_enter(group);
    /* Nothing runs in IDLE, and the first fault of a group is the one that counts. */
    const uint8_t first =
        group->state != STEP_STREAM_GROUP_STATE_IDLE && group->state != STEP_STREAM_GROUP_STATE_FAULT ? 1U : 0U;
    if (first != 0U) {
        group->state = STEP_STREAM_GROUP_STATE_FAULT;
        group->fault_member = member;
        group->fault = fault;
    }
    step_stream_group_exit(group, saved);
    if (first == 0U) {
        return;
    }
    /* The failed channel has stopped itself; the others must not go on along a path that has lost a part. */
    step_stream_group_stop_members(group, member);
    if (group->listener.on_fault != NULL) {
        group->listener.on_fault(group->listener.context, member, fault);
    }
}

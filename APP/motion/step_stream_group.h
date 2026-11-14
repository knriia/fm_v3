#ifndef FM_V3_STEP_STREAM_GROUP_H
#define FM_V3_STEP_STREAM_GROUP_H

#include <stdint.h>

/* Owner of the life cycle of the stream channels that move together: prepare, common start, stop, and what a fault or a
 * limit switch of one channel does to the others. Pure logic: the channels, the common start and the critical section
 * are reached only through the ports below, so the module does not depend on the BSP.
 *
 * Who calls what. The owner task calls prepare, start, stop, service and reset. The channels report from the DMA
 * interrupt (on_member_fault, on_member_braking) and the limit switch interrupt reports on_limit; none of these block.
 * The state of the group is changed only inside the critical section of the ports, and a fault is never overwritten
 * by anything but reset. */

#define STEP_STREAM_GROUP_MAX_MEMBERS 4U

typedef enum {
    STEP_STREAM_GROUP_STATUS_OK = 0,
    STEP_STREAM_GROUP_STATUS_INVALID_ARGUMENT,
    /* The call does not fit the state of the group. */
    STEP_STREAM_GROUP_STATUS_BAD_STATE,
    /* A channel refused the operation; the group has stopped every channel and is IDLE (prepare: still IDLE). */
    STEP_STREAM_GROUP_STATUS_MEMBER_ERROR
} StepStreamGroupStatus_t;

typedef enum { STEP_STREAM_GROUP_PORT_OK = 0, STEP_STREAM_GROUP_PORT_ERROR } StepStreamGroupPortResult_t;

/* What a limit switch of one channel does. */
typedef enum {
    /* A planned move: one switch stops every channel at once, each by its own braking ramp. */
    STEP_STREAM_GROUP_MODE_PLANNED = 0,
    /* Homing and calibration: the channels reach their switches at different times, so a switch brakes only its own
     * channel and the others go on. */
    STEP_STREAM_GROUP_MODE_INDEPENDENT
} StepStreamGroupMode_t;

typedef enum {
    /* No channel runs. */
    STEP_STREAM_GROUP_STATE_IDLE = 0,
    /* Rings are prepared and being filled; nothing has started. */
    STEP_STREAM_GROUP_STATE_PRIMING,
    STEP_STREAM_GROUP_STATE_RUNNING,
    /* A switch has begun the braking of every channel; the group is IDLE again once service sees them stopped. */
    STEP_STREAM_GROUP_STATE_STOPPING,
    /* A channel failed and every channel was stopped. Sticky until reset. */
    STEP_STREAM_GROUP_STATE_FAULT
} StepStreamGroupState_t;

/* A channel is addressed by its index in the group, 0..member_count-1. */
typedef struct {
    /* Resets the ring of an IDLE channel. */
    StepStreamGroupPortResult_t (*prepare)(void *context, uint32_t member);
    /* Sets the channel up with both halves FULL; nothing moves until trigger. */
    StepStreamGroupPortResult_t (*arm)(void *context, uint32_t member, uint32_t startup_period_ns);
    /* Starts every armed channel at the same moment. */
    void (*trigger)(void *context);
    /* Stops the channel at once; harmless on an idle one. Callable from an interrupt. */
    void (*stop)(void *context, uint32_t member);
    /* Starts the braking of the channel, or stops it at once when it has no ramp. Callable from an interrupt. */
    void (*brake)(void *context, uint32_t member);
    /* 1 while the channel streams or brakes. */
    uint8_t (*is_active)(void *context, uint32_t member);
    /* Masks the interrupts that call the group; returns what exit_critical needs. */
    uint32_t (*enter_critical)(void *context);
    void (*exit_critical)(void *context, uint32_t saved);
    void *context;
} StepStreamGroupPorts_t;

/* All callbacks run in an interrupt and must not block. */
typedef struct {
    /* A channel has begun to brake, or was stopped at once because it could not: feed it no more. */
    void (*on_member_stopping)(void *context, uint32_t member);
    /* A channel failed with `fault` (its StepStreamAxisFault_t value) and the group has stopped every channel. */
    void (*on_fault)(void *context, uint32_t member, uint32_t fault);
    void *context;
} StepStreamGroupListener_t;

typedef struct {
    StepStreamGroupPorts_t ports;
    StepStreamGroupListener_t listener;
    uint32_t member_count;
    volatile StepStreamGroupMode_t mode;
    volatile StepStreamGroupState_t state;
    volatile uint32_t fault_member;
    volatile uint32_t fault;
    /* Bit per member: a switch was reached while the channels were armed but the group was not yet RUNNING. start
     * hands it to on_limit once the channels run. */
    volatile uint32_t pending_limits;
} StepStreamGroup_t;

/* A group in static storage can be defined without init: set .ports, .listener and .member_count in its initializer and
 * leave the rest zero (IDLE, PLANNED). It is then complete before any task or interrupt runs, so calls that come
 * before the owner task starts need no special case. */

/* member_count channels (1..STEP_STREAM_GROUP_MAX_MEMBERS); the listener may be NULL. */
StepStreamGroupStatus_t step_stream_group_init(
    StepStreamGroup_t *group,
    uint32_t member_count,
    const StepStreamGroupPorts_t *ports,
    const StepStreamGroupListener_t *listener
);

/* Only while IDLE. The mode stays until changed. */
StepStreamGroupStatus_t step_stream_group_set_mode(StepStreamGroup_t *group, StepStreamGroupMode_t mode);

/* IDLE to PRIMING: resets every ring. */
StepStreamGroupStatus_t step_stream_group_prepare(StepStreamGroup_t *group);
/* PRIMING to RUNNING: arms every channel, then starts them together. A channel that refuses stops them all
 * (MEMBER_ERROR). A stop or a fault that comes while the channels are being armed wins: BAD_STATE, nothing starts. A
 * switch reported meanwhile is applied right after the start. */
StepStreamGroupStatus_t step_stream_group_start(StepStreamGroup_t *group, uint32_t startup_period_ns);
/* Stops every channel at once; any task may call it. A fault stays; anything else becomes IDLE. */
void step_stream_group_stop(StepStreamGroup_t *group);
/* Owner task: RUNNING or STOPPING with no active channel becomes IDLE. Returns the state. Without calls (e.g. while the
 * owner does not feed after a stop) STOPPING stays until stop or reset, which is as good: both accept it. */
StepStreamGroupState_t step_stream_group_service(StepStreamGroup_t *group);
/* Clears a fault once no channel is active (BAD_STATE otherwise). Also valid in any other state with no active
 * channel; the group is IDLE afterwards. */
StepStreamGroupStatus_t step_stream_group_reset(StepStreamGroup_t *group);

StepStreamGroupState_t step_stream_group_get_state(const StepStreamGroup_t *group);
/* Channel and fault of the last FAULT; meaningful in FAULT only. */
uint32_t step_stream_group_get_fault_member(const StepStreamGroup_t *group);
uint32_t step_stream_group_get_fault(const StepStreamGroup_t *group);

/* Limit switch interrupt: the channel `member` reached a switch it moves toward. */
void step_stream_group_on_limit(StepStreamGroup_t *group, uint32_t member);
/* Channel interrupt: the channel has begun to brake, or stopped at once. */
void step_stream_group_on_member_braking(StepStreamGroup_t *group, uint32_t member);
/* Channel interrupt: the channel has stopped itself after `fault`. */
void step_stream_group_on_member_fault(StepStreamGroup_t *group, uint32_t member, uint32_t fault);

#endif /* FM_V3_STEP_STREAM_GROUP_H */

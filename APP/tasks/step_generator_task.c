#include "step_generator_task.h"

#include "motion/plan_buffer.h"
#include "motion/step_stream_feeder.h"
#include "motion/step_stream_group.h"
#include "motion_axis_hardware.h"
#include "step_stream_axis.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include <stddef.h>

/* X (TIM4 CH1, DMA1 stream 0), Y (TIM1 CH1, DMA1 stream 1) and Z (TIM3 CH1, DMA1 stream 2) have a planned stream in
 * step_stream_axis. X is the reference of the timing: the members must share its timer kernel clock. */
#define STEP_GENERATOR_TASK_AXIS MOTION_AXIS_X
/* The timer idles this long before the DMA delivers the first record of a stream. */
#define STEP_GENERATOR_TASK_STARTUP_PERIOD_NS STEP_STREAM_AXIS_MIN_STARTUP_NS
/* A record is never shorter than this: the DMA must finish a burst before the next timer update. */
#define STEP_GENERATOR_TASK_MIN_RECORD_NS STEP_STREAM_AXIS_MIN_STARTUP_NS
/* Filler record of an empty stretch. A half of them takes about 14 ms, which is how long a new block can wait behind
 * an idle half; shorter records would only load the DMA interrupt more. */
#define STEP_GENERATOR_TASK_IDLE_RECORD_NS 10000U
/* Longest record of a wait; 0 is the format maximum (238 us). With several axes it must be short enough that a ring
 * half of an axis that mostly waits spans about the time of a half of an axis that steps: the plan has to hold the
 * blocks of the longest half. Still 0 with three axes: it is tuned on the hardware (kanban card 4e.3). */
#define STEP_GENERATOR_TASK_MAX_WAIT_RECORD_NS 0U
/* The braking ramp starts at this step rate: the fast seek of homing. A faster axis is brought down to it at once. */
#define STEP_GENERATOR_TASK_BRAKE_MAX_RATE_HZ 50000U
/* DIR setup before the first STEP is at least this long; the shortest RTOS delay covers it. */
#define STEP_GENERATOR_TASK_DIRECTION_SETTLE_MS 1U

#define STEP_GENERATOR_TASK_FLAG_BLOCK_READY (1UL << 0)
#define STEP_GENERATOR_TASK_FLAG_STOP_RESET (1UL << 1)
#define STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED (1UL << 2)
#define STEP_GENERATOR_TASK_FLAGS_ALL                                                                                  \
    (STEP_GENERATOR_TASK_FLAG_BLOCK_READY | STEP_GENERATOR_TASK_FLAG_STOP_RESET |                                      \
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED)

/* The channels that move together: member 0 is X, member 1 is Y, member 2 is Z. */
#define STEP_GENERATOR_TASK_MEMBER_COUNT 3U
_Static_assert(STEP_GENERATOR_TASK_MEMBER_COUNT == STEP_GENERATOR_TASK_STREAM_MEMBERS, "diagnostics list every member");

static const MotionAxis_t step_generator_task_member_axes[STEP_GENERATOR_TASK_MEMBER_COUNT] = {
    MOTION_AXIS_X,
    MOTION_AXIS_Y,
    MOTION_AXIS_Z,
};
/* The planner axis behind each member, in the order of the members. */
static const uint32_t step_generator_task_member_planner_axes[STEP_GENERATOR_TASK_MEMBER_COUNT] = {0U, 1U, 2U};

struct StepGeneratorInstance;

/* What an interrupt of a channel knows about it: the instance it belongs to and its index in the instance's group. */
typedef struct {
    struct StepGeneratorInstance *instance;
    uint32_t member;
} StepGeneratorMemberContext_t;

/* Everything one stream owns: its feeder, the group of its channels, the halt state and the diagnostics. Every port and
 * every listener of the stream gets a pointer to it as its context, so the file keeps no other stream state. The
 * instance is defined with its initializer below, so it exists before the task starts and a STOP that comes earlier
 * still reaches the axes. */
typedef struct StepGeneratorInstance {
    volatile StepGeneratorTaskDiagnostics diagnostics;
    /* Owned by the task: only StepGeneratorTask touches it. */
    StepStreamFeeder_t feeder;
    StepStreamGroup_t group;
    /* The queue of plan blocks this stream plays; set once at task start, NULL before it. */
    PlanBuffer_t *plan_buffer;
    const MotionAxis_t *member_axes;
    const uint32_t *member_planner_axes;
    uint32_t member_count;
    StepGeneratorMemberContext_t member_contexts[STEP_GENERATOR_TASK_MEMBER_COUNT];
    StepStreamAxisListener_t stream_listeners[STEP_GENERATOR_TASK_MEMBER_COUNT];
    /* 1 once the feeder is built; an instance that is not ready only stays halted. */
    uint8_t ready;
    /* 1 while this instance holds the channels of its members. Only StepGeneratorTask touches it. */
    volatile uint8_t channels_held;
    /* Set by step_generator_instance_resume_after_stop, cleared by the task when it resets this instance. */
    volatile uint8_t reset_requested;
    /* From the moment a block leaves the plan buffer until the stream has ended; STOP resets it. */
    volatile uint8_t stream_busy;
    /* Set by STOP in another task or by a ring fault in the DMA interrupt; cleared by this task once it is reset. */
    volatile uint8_t halted;
    /* Set once at start when the axis refused the braking ramp; counted into generator_errors with the feeder's
     * errors. */
    volatile uint8_t brake_setup_failed;
    /* Why the feeding stopped: the first cause of the latest halt, kept after the reset that lifts it. Written inside a
     * critical section where halted is set, read only for diagnostics. */
    volatile uint32_t halt_cause;
    volatile uint32_t halt_member;
    volatile uint32_t halt_fault;
    volatile uint32_t halts_total;
    /* Written by the generator task only. */
    volatile uint32_t feeder_status;
    /* Resets after STOP that left a channel running; counted into axis_errors with the feeder's errors. */
    volatile uint32_t reset_failures;
    /* Counted inside a critical section: the switch interrupt and the poll at the stream start both report. */
    volatile uint32_t limit_trips[STEP_GENERATOR_TASK_MEMBER_COUNT];
} StepGeneratorInstance_t;

/* The printing stream of X, Y and Z. */
static StepGeneratorInstance_t step_generator_task_instance;
/* The streams of HOME and CALIBRATE: one lane on one axis each, indexed by MotionAxis_t, so the axes of a command run
 * independently. */
static StepGeneratorInstance_t step_generator_task_axis_instances[STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT];
_Static_assert(
    STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT == (uint32_t)MOTION_AXIS_HARDWARE_COUNT,
    "one instance per axis"
);
/* Every instance the task serves; STOP and the service loop walk it. */
static StepGeneratorInstance_t *const step_generator_task_instances[] = {
    &step_generator_task_instance,
    &step_generator_task_axis_instances[MOTION_AXIS_X],
    &step_generator_task_axis_instances[MOTION_AXIS_Y],
    &step_generator_task_axis_instances[MOTION_AXIS_Z],
};
#define STEP_GENERATOR_TASK_INSTANCE_COUNT                                                                             \
    (sizeof(step_generator_task_instances) / sizeof(step_generator_task_instances[0]))
/* Written once at task start and read by MotionTask and the DMA interrupt to wake the task. */
static osThreadId_t volatile step_generator_task_thread_id;

/* The stream channel of the axis; the instance lives for the whole run, so this is a lookup and no state. */
static StepStreamAxis_t *step_generator_task_axis(void) { return step_stream_axis_get(STEP_GENERATOR_TASK_AXIS); }

/* The group passes only members below its count, so the index is always inside the table. */
static StepStreamAxis_t *step_generator_task_member_axis(const StepGeneratorInstance_t *self, uint32_t member) {
    return step_stream_axis_get(self->member_axes[member]);
}

static void step_generator_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

/* The only place that sets halted, so every halt leaves its cause behind. Callable from tasks and interrupts. Only the
 * first halt since the last reset records its cause: a later one (MotionTask ending the move after a switch stopped it)
 * would hide what stopped the stream. Every halt is counted. */
static void
step_generator_task_set_halted(StepGeneratorInstance_t *self, uint32_t cause, uint32_t member, uint32_t fault) {
    const uint32_t primask = step_stream_axis_enter_critical();
    if (self->halted == 0U) {
        self->halt_cause = cause;
        self->halt_member = member;
        self->halt_fault = fault;
    }
    step_generator_counter_increment(&self->halts_total);
    self->halted = 1U;
    step_stream_axis_exit_critical(primask);
}

static void step_generator_task_signal(uint32_t flags) {
    const osThreadId_t thread_id = step_generator_task_thread_id;
    if (thread_id == NULL) {
        return;
    }
    (void)osThreadFlagsSet(thread_id, flags);
}

void step_generator_task_notify_block_ready(void) { step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_BLOCK_READY); }

void step_generator_task_notify_ring_half_released(void) {
    step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED);
}

StepGeneratorInstance_t *step_generator_task_get_print_instance(void) { return &step_generator_task_instance; }

StepGeneratorInstance_t *step_generator_task_get_axis_instance(MotionAxis_t axis) {
    return (uint32_t)axis < STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT ? &step_generator_task_axis_instances[axis] : NULL;
}

PlanBuffer_t *step_generator_instance_get_plan_buffer(const StepGeneratorInstance_t *instance) {
    return instance == NULL ? NULL : instance->plan_buffer;
}

/* Stops the channels of the instance's members, but only while it holds them: otherwise they belong to another instance
 * that may be streaming on them, and stopping them here would cut its stream. */
static void step_generator_task_stop_own_channels(StepGeneratorInstance_t *self) {
    if (self->channels_held != 0U) {
        step_stream_group_stop(&self->group);
    }
}

void step_generator_instance_halt(StepGeneratorInstance_t *instance) {
    if (instance == NULL) {
        return;
    }
    step_generator_task_set_halted(
        instance,
        STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL,
        STEP_GENERATOR_TASK_NO_MEMBER,
        0U
    );
    /* The axes stop themselves even if the task is in the middle of a fill: the start port checks the flag with task
     * switching paused, so a stream cannot begin after this stop. */
    step_generator_task_stop_own_channels(instance);
}

/* The list walks take the table as an argument, so a test can walk a table of two instances. */
static void step_generator_task_halt_list(StepGeneratorInstance_t *const *list, uint32_t count) {
    for (uint32_t index = 0U; index < count; ++index) {
        step_generator_instance_halt(list[index]);
    }
}

void step_generator_task_halt_all(void) {
    step_generator_task_halt_list(step_generator_task_instances, STEP_GENERATOR_TASK_INSTANCE_COUNT);
}

uint8_t step_generator_instance_is_busy(const StepGeneratorInstance_t *instance) {
    if (instance == NULL) {
        return 0U;
    }
    PlanBufferDiagnostics_t plan = {0};
    /* Only a null pointer (no queue yet) makes this fail, and the zeroed struct then reads as an empty plan. */
    (void)plan_buffer_get_diagnostics(instance->plan_buffer, &plan);
    return (uint8_t)(instance->stream_busy != 0U || plan.queued != 0U);
}

uint8_t step_generator_instance_is_halted(const StepGeneratorInstance_t *instance) {
    return instance == NULL ? 0U : instance->halted;
}

uint8_t step_generator_instance_is_ready(const StepGeneratorInstance_t *instance) {
    return instance == NULL ? 0U : instance->ready;
}

uint32_t step_generator_instance_reset_count(const StepGeneratorInstance_t *instance) {
    return instance == NULL ? 0U : instance->diagnostics.stop_resets;
}

void step_generator_instance_resume_after_stop(StepGeneratorInstance_t *instance) {
    /* Before the task starts nothing runs yet and the task has not set the instance up, so there is nothing to reset; a
     * request kept from then would reset the instance on the first pass and lift a STOP that came in between. */
    if (instance == NULL || step_generator_task_thread_id == NULL) {
        return;
    }
    instance->reset_requested = 1U;
    step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_STOP_RESET);
}

static void step_generator_task_resume_list(StepGeneratorInstance_t *const *list, uint32_t count) {
    for (uint32_t index = 0U; index < count; ++index) {
        step_generator_instance_resume_after_stop(list[index]);
    }
}

void step_generator_task_resume_all_after_stop(void) {
    step_generator_task_resume_list(step_generator_task_instances, STEP_GENERATOR_TASK_INSTANCE_COUNT);
}

/* The ports below adapt the feeder to the stream axis and the plan buffer. */

static StepStreamFeederAxisResult_t step_generator_task_port_result(StepStreamAxisStatus_t status) {
    return status == STEP_STREAM_AXIS_STATUS_OK ? STEP_STREAM_FEEDER_AXIS_OK : STEP_STREAM_FEEDER_AXIS_ERROR;
}

static StepStreamFeederAxisResult_t step_generator_task_group_result(StepStreamGroupStatus_t status) {
    return status == STEP_STREAM_GROUP_STATUS_OK ? STEP_STREAM_FEEDER_AXIS_OK : STEP_STREAM_FEEDER_AXIS_ERROR;
}

/* Called while the feeder opens a stream, after it has decided which axes the blocks move. An axis that only idles in
 * the stream may stand on its switch; its limit must not stop the axes that move. */
static void step_generator_task_watch_moving_axes(StepGeneratorInstance_t *self) {
    const uint8_t moving_planner_axes = step_stream_feeder_get_moving_axes(&self->feeder);
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        const uint8_t moves = (uint8_t)((moving_planner_axes >> self->member_planner_axes[member]) & 1U);
        motion_axis_hardware_set_stream_axis_watched(self->member_axes[member], moves);
    }
}

/* No stream runs: the axes of the instance are watched again until the next stream narrows them to those it moves. */
static void step_generator_task_watch_all_axes(const StepGeneratorInstance_t *self) {
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        motion_axis_hardware_set_stream_axis_watched(self->member_axes[member], 1U);
    }
}

static uint8_t step_generator_task_claim_channels(StepGeneratorInstance_t *self);
static void step_generator_task_release_channels(StepGeneratorInstance_t *self);

static StepStreamFeederAxisResult_t step_generator_task_port_prepare(void *context) {
    StepGeneratorInstance_t *const self = context;
    /* The channels belong to the instance only while it streams: the printing instance and the single-axis ones use the
     * same timers, and a second owner of a channel must be refused, not take it over. */
    if (step_generator_task_claim_channels(self) == 0U) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    step_generator_task_watch_moving_axes(self);
    return step_generator_task_group_result(step_stream_group_prepare(&self->group));
}

static StepStreamFeederAxisResult_t
step_generator_task_port_acquire_half(void *context, uint32_t lane, uint8_t *half, uint16_t **records) {
    const StepGeneratorInstance_t *const self = context;
    const StepStreamAxisStatus_t status =
        step_stream_axis_acquire_half(step_generator_task_member_axis(self, lane), half, records);
    if (status == STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST) {
        return STEP_STREAM_FEEDER_AXIS_NO_REQUEST;
    }
    return step_generator_task_port_result(status);
}

static StepStreamFeederAxisResult_t step_generator_task_port_commit_half(void *context, uint32_t lane, uint8_t half) {
    const StepGeneratorInstance_t *const self = context;
    return step_generator_task_port_result(
        step_stream_axis_commit_half(step_generator_task_member_axis(self, lane), half)
    );
}

static StepStreamFeederAxisResult_t
step_generator_task_port_set_direction(void *context, uint32_t lane, uint8_t positive) {
    const StepGeneratorInstance_t *const self = context;
    /* With task switching paused, STOP and MotionTask cannot touch the axis between the flag check and the write. */
    MotionAxisHardwareStatus_t status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    (void)osKernelLock();
    if (self->halted == 0U) {
        /* The feeder passes lanes below the member count only; this port needs the motion axis, not the channel. */
        status = motion_axis_hardware_set_stream_outputs(self->member_axes[lane], positive);
    }
    (void)osKernelUnlock();
    /* A refusal returns the status of a busy or faulted axis; a write leaves a status the axis can start from. */
    if (status != MOTION_AXIS_HARDWARE_STATUS_READY && status != MOTION_AXIS_HARDWARE_STATUS_COMPLETE &&
        status != MOTION_AXIS_HARDWARE_STATUS_STOPPED) {
        return STEP_STREAM_FEEDER_AXIS_ERROR;
    }
    (void)osDelay(STEP_GENERATOR_TASK_DIRECTION_SETTLE_MS);
    return STEP_STREAM_FEEDER_AXIS_OK;
}

static StepStreamFeederAxisResult_t step_generator_task_port_start(void *context, uint32_t startup_period_ns) {
    StepGeneratorInstance_t *const self = context;
    StepStreamGroupStatus_t status = STEP_STREAM_GROUP_STATUS_BAD_STATE;
    (void)osKernelLock();
    if (self->halted == 0U) {
        status = step_stream_group_start(&self->group, startup_period_ns);
    }
    (void)osKernelUnlock();
    return step_generator_task_group_result(status);
}

static void step_generator_task_port_stop(void *context) {
    StepGeneratorInstance_t *const self = context;
    step_generator_task_stop_own_channels(self);
    /* The channels are at rest; a refusal is tried again by the task when it has served the instance. */
    step_generator_task_release_channels(self);
}

static StepStreamFeederPlanResult_t step_generator_task_plan_result(PlanBufferStatus_t status) {
    switch (status) {
    case PLAN_BUFFER_STATUS_OK:
        return STEP_STREAM_FEEDER_PLAN_OK;
    case PLAN_BUFFER_STATUS_EMPTY:
        return STEP_STREAM_FEEDER_PLAN_NONE;
    case PLAN_BUFFER_STATUS_STALE:
        return STEP_STREAM_FEEDER_PLAN_STALE;
    default:
        return STEP_STREAM_FEEDER_PLAN_ERROR;
    }
}

static uint32_t step_generator_task_port_plan_epoch(void *context) {
    const StepGeneratorInstance_t *const self = context;
    return plan_buffer_get_epoch(self->plan_buffer);
}

static StepStreamFeederPlanResult_t
step_generator_task_port_peek_block(void *context, uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block) {
    StepGeneratorInstance_t *const self = context;
    return step_generator_task_plan_result(plan_buffer_try_peek(self->plan_buffer, epoch, offset, block));
}

static StepStreamFeederPlanResult_t
step_generator_task_port_discard_blocks(void *context, uint32_t epoch, uint32_t count) {
    StepGeneratorInstance_t *const self = context;
    /* Set before the blocks are out of sight, so HOME never sees an empty plan buffer and an idle stream in between. */
    self->stream_busy = 1U;
    return step_generator_task_plan_result(plan_buffer_discard(self->plan_buffer, epoch, count));
}

/* DMA interrupt: a half of the ring has been played. */
static void step_generator_task_notify_from_ring(void *context) {
    (void)context;
    step_generator_task_notify_ring_half_released();
}

/* DMA interrupt: a channel has stopped itself after a ring fault; the group tells the others and the owner. */
static void step_generator_task_channel_fault(void *context, StepStreamAxisFault_t fault) {
    const StepGeneratorMemberContext_t *const member = context;
    step_stream_group_on_member_fault(&member->instance->group, member->member, (uint32_t)fault);
}

/* DMA interrupt: a channel brakes on a limit switch and refills its ring itself, so the feeder must not write. */
static void step_generator_task_channel_braking(void *context) {
    const StepGeneratorMemberContext_t *const member = context;
    step_stream_group_on_member_braking(&member->instance->group, member->member);
}

/* Limit switch interrupt: the group decides which channels brake. */
static void step_generator_task_limit_reached(void *context, MotionAxis_t axis) {
    StepGeneratorInstance_t *const self = context;
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        if (self->member_axes[member] == axis) {
            const uint32_t primask = step_stream_axis_enter_critical();
            step_generator_counter_increment(&self->limit_trips[member]);
            step_stream_axis_exit_critical(primask);
            step_stream_group_on_limit(&self->group, member);
            return;
        }
    }
}

/* Group, in an interrupt: the stream stopped for a limit switch or a fault, so stop feeding until STOP resets us. */
static void step_generator_task_group_member_stopping(void *context, uint32_t member) {
    step_generator_task_set_halted(context, STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, member, 0U);
}

static void step_generator_task_group_fault(void *context, uint32_t member, uint32_t fault) {
    step_generator_task_set_halted(context, STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, member, fault);
}

/* The group reaches the channels only through these. */
static StepStreamGroupPortResult_t step_generator_task_member_prepare(void *context, uint32_t member) {
    const StepGeneratorInstance_t *const self = context;
    return step_stream_axis_prepare_stream(step_generator_task_member_axis(self, member)) == STEP_STREAM_AXIS_STATUS_OK
               ? STEP_STREAM_GROUP_PORT_OK
               : STEP_STREAM_GROUP_PORT_ERROR;
}

static StepStreamGroupPortResult_t
step_generator_task_member_arm(void *context, uint32_t member, uint32_t startup_period_ns) {
    const StepGeneratorInstance_t *const self = context;
    return step_stream_axis_arm_stream(step_generator_task_member_axis(self, member), startup_period_ns) ==
                   STEP_STREAM_AXIS_STATUS_OK
               ? STEP_STREAM_GROUP_PORT_OK
               : STEP_STREAM_GROUP_PORT_ERROR;
}

static void step_generator_task_member_trigger(void *context) {
    const StepGeneratorInstance_t *const self = context;
    StepStreamAxis_t *axes[STEP_GENERATOR_TASK_MEMBER_COUNT];
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        axes[member] = step_generator_task_member_axis(self, member);
    }
    step_stream_axis_trigger(axes, self->member_count);
}

static void step_generator_task_member_stop(void *context, uint32_t member) {
    const StepGeneratorInstance_t *const self = context;
    step_stream_axis_stop(step_generator_task_member_axis(self, member));
}

static void step_generator_task_member_brake(void *context, uint32_t member) {
    const StepGeneratorInstance_t *const self = context;
    step_stream_axis_request_brake(step_generator_task_member_axis(self, member));
}

static uint8_t step_generator_task_member_is_active(void *context, uint32_t member) {
    const StepGeneratorInstance_t *const self = context;
    return step_stream_axis_get_state(step_generator_task_member_axis(self, member)) == STEP_STREAM_AXIS_STATE_STREAMING
               ? 1U
               : 0U;
}

static uint32_t step_generator_task_critical_enter(void *context) {
    (void)context;
    return step_stream_axis_enter_critical();
}

static void step_generator_task_critical_exit(void *context, uint32_t saved) {
    (void)context;
    step_stream_axis_exit_critical(saved);
}

#define STEP_GENERATOR_TASK_MEMBER_CONTEXT(instance, index) {&(instance), (index)}
#define STEP_GENERATOR_TASK_MEMBER_LISTENER(instance, index)                                                           \
    {                                                                                                                  \
        .on_half_released = step_generator_task_notify_from_ring,                                                      \
        .on_fault = step_generator_task_channel_fault,                                                                 \
        .on_brake = step_generator_task_channel_braking,                                                               \
        .context = &(instance).member_contexts[(index)],                                                               \
    }

/* `instance` is the object being initialized, `axes` and `planner_axes` point into the member tables, `count` is the
 * number of members in the group. The context tables are filled for every slot; the unused ones are never read. */
#define STEP_GENERATOR_TASK_INSTANCE_INITIAL(instance, axes, planner_axes, count)                                      \
    {                                                                                                                  \
        .group =                                                                                                       \
            {                                                                                                          \
                .ports =                                                                                               \
                    {                                                                                                  \
                        .prepare = step_generator_task_member_prepare,                                                 \
                        .arm = step_generator_task_member_arm,                                                         \
                        .trigger = step_generator_task_member_trigger,                                                 \
                        .stop = step_generator_task_member_stop,                                                       \
                        .brake = step_generator_task_member_brake,                                                     \
                        .is_active = step_generator_task_member_is_active,                                             \
                        .enter_critical = step_generator_task_critical_enter,                                          \
                        .exit_critical = step_generator_task_critical_exit,                                            \
                        .context = &(instance),                                                                        \
                    },                                                                                                 \
                .listener =                                                                                            \
                    {                                                                                                  \
                        .on_member_stopping = step_generator_task_group_member_stopping,                               \
                        .on_fault = step_generator_task_group_fault,                                                   \
                        .context = &(instance),                                                                        \
                    },                                                                                                 \
                .member_count = (count),                                                                               \
            },                                                                                                         \
        .member_axes = (axes),                                                                                         \
        .member_planner_axes = (planner_axes),                                                                         \
        .member_count = (count),                                                                                       \
        .member_contexts =                                                                                             \
            {                                                                                                          \
                STEP_GENERATOR_TASK_MEMBER_CONTEXT(instance, 0U),                                                      \
                STEP_GENERATOR_TASK_MEMBER_CONTEXT(instance, 1U),                                                      \
                STEP_GENERATOR_TASK_MEMBER_CONTEXT(instance, 2U),                                                      \
            },                                                                                                         \
        .stream_listeners =                                                                                            \
            {                                                                                                          \
                STEP_GENERATOR_TASK_MEMBER_LISTENER(instance, 0U),                                                     \
                STEP_GENERATOR_TASK_MEMBER_LISTENER(instance, 1U),                                                     \
                STEP_GENERATOR_TASK_MEMBER_LISTENER(instance, 2U),                                                     \
            },                                                                                                         \
        .halt_member = STEP_GENERATOR_TASK_NO_MEMBER,                                                                  \
    }

#define STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(axis)                                                                \
    STEP_GENERATOR_TASK_INSTANCE_INITIAL(                                                                              \
        step_generator_task_axis_instances[axis],                                                                      \
        &step_generator_task_member_axes[axis],                                                                        \
        &step_generator_task_member_planner_axes[axis],                                                                \
        1U                                                                                                             \
    )

static StepGeneratorInstance_t step_generator_task_instance = STEP_GENERATOR_TASK_INSTANCE_INITIAL(
    step_generator_task_instance,
    step_generator_task_member_axes,
    step_generator_task_member_planner_axes,
    STEP_GENERATOR_TASK_MEMBER_COUNT
);

static StepGeneratorInstance_t step_generator_task_axis_instances[STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT] = {
    [MOTION_AXIS_X] = STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(MOTION_AXIS_X),
    [MOTION_AXIS_Y] = STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(MOTION_AXIS_Y),
    [MOTION_AXIS_Z] = STEP_GENERATOR_TASK_AXIS_INSTANCE_INITIAL(MOTION_AXIS_Z),
};

static void step_generator_task_publish_diagnostics(StepGeneratorInstance_t *self) {
    StepStreamFeederDiagnostics_t feeder_diagnostics;
    step_stream_feeder_get_diagnostics(&self->feeder, &feeder_diagnostics);
    self->diagnostics.blocks_started = feeder_diagnostics.blocks_started;
    self->diagnostics.blocks_completed = feeder_diagnostics.blocks_completed;
    self->diagnostics.halves_filled = feeder_diagnostics.halves_filled;
    self->diagnostics.generator_errors =
        self->brake_setup_failed != 0U && feeder_diagnostics.generator_errors != UINT32_MAX
            ? feeder_diagnostics.generator_errors + 1U
            : feeder_diagnostics.generator_errors;
    const uint64_t axis_errors = (uint64_t)feeder_diagnostics.axis_errors + self->reset_failures;
    self->diagnostics.axis_errors = axis_errors > UINT32_MAX ? UINT32_MAX : (uint32_t)axis_errors;
    self->diagnostics.streams_started = feeder_diagnostics.streams_started;
    self->diagnostics.streams_stopped = feeder_diagnostics.streams_stopped;
}

/* Builds the braking ramp of the axis from the same timing as the feeder. Without a deceleration setting, or with
 * one the ramp cannot be built from, the axis keeps no ramp and stops at once on a limit switch; the latter is counted
 * in generator_errors. */
static void step_generator_task_setup_brake(
    StepGeneratorInstance_t *self,
    const StepGeneratorTaskConfig_t *task_config,
    const StepStreamFeederConfig_t *feeder_config
) {
    if (task_config->limit_deceleration_steps_per_s2 == 0U) {
        return;
    }
    const uint64_t pulse_and_low = (uint64_t)feeder_config->generator.pulse_ticks + feeder_config->generator.low_ticks;
    const StepStreamBrakeConfig_t brake = {
        .timer_clock_hz = feeder_config->generator.timer_clock_hz,
        .pulse_ticks = feeder_config->generator.pulse_ticks,
        .min_period_ticks = feeder_config->generator.min_record_ticks > pulse_and_low
                                ? feeder_config->generator.min_record_ticks
                                : (uint32_t)pulse_and_low,
        .idle_record_ticks = feeder_config->idle_record_ticks,
        .deceleration_steps_per_s2 = task_config->limit_deceleration_steps_per_s2,
        .max_step_rate_hz = STEP_GENERATOR_TASK_BRAKE_MAX_RATE_HZ,
    };
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        if (step_stream_axis_configure_brake(step_generator_task_member_axis(self, member), &brake) !=
            STEP_STREAM_AXIS_STATUS_OK) {
            self->brake_setup_failed = 1U;
        }
    }
}

/* The feeder counts every record in the ticks of one clock, so the members must share the timer kernel clock. */
static uint8_t step_generator_task_member_clocks_match(const StepGeneratorInstance_t *self) {
    const uint32_t reference_hz = step_stream_axis_get_timer_clock_hz(step_generator_task_axis());
    for (uint32_t member = 1U; member < self->member_count; ++member) {
        if (step_stream_axis_get_timer_clock_hz(step_generator_task_member_axis(self, member)) != reference_hz) {
            return 0U;
        }
    }
    return 1U;
}

/* Builds the feeder from the driver timing. Returns 0 when the clock or the timing cannot be used. */
static uint8_t step_generator_task_setup_feeder(
    StepGeneratorInstance_t *self,
    const StepGeneratorTaskConfig_t *task_config,
    PlanBuffer_t *plan_buffer
) {
    if (task_config == NULL) {
        return 0U;
    }
    /* Kept even when the setup fails below, so is_busy still sees the queued blocks. */
    self->plan_buffer = plan_buffer;
    if (plan_buffer == NULL || step_generator_task_member_clocks_match(self) == 0U) {
        return 0U;
    }
    StepStreamFeederConfig_t config;
    if (step_stream_feeder_make_config(
            step_stream_axis_get_timer_clock_hz(step_generator_task_axis()),
            task_config->step_pulse_width_ns,
            task_config->step_low_time_ns,
            STEP_GENERATOR_TASK_MIN_RECORD_NS,
            STEP_GENERATOR_TASK_MAX_WAIT_RECORD_NS,
            STEP_GENERATOR_TASK_IDLE_RECORD_NS,
            STEP_GENERATOR_TASK_STARTUP_PERIOD_NS,
            STEP_STREAM_AXIS_HALF_RECORD_COUNT,
            self->member_planner_axes,
            self->member_count,
            &config
        ) != STEP_STREAM_FEEDER_STATUS_OK) {
        return 0U;
    }
    const StepStreamFeederPorts_t ports = {
        .prepare = step_generator_task_port_prepare,
        .acquire_half = step_generator_task_port_acquire_half,
        .commit_half = step_generator_task_port_commit_half,
        .set_direction = step_generator_task_port_set_direction,
        .start = step_generator_task_port_start,
        .stop = step_generator_task_port_stop,
        .plan_epoch = step_generator_task_port_plan_epoch,
        .peek_block = step_generator_task_port_peek_block,
        .discard_blocks = step_generator_task_port_discard_blocks,
        .context = self,
    };
    /* The ramp depends on the config only, so it is set up whether or not the feeder takes it. It belongs to the axis,
     * not to the instance, and the printing instance covers all of them: the single-axis instances would only build the
     * same tables again. */
    if (self == &step_generator_task_instance) {
        step_generator_task_setup_brake(self, task_config, &config);
    }
    return (uint8_t)(step_stream_feeder_init(&self->feeder, &config, &ports) == STEP_STREAM_FEEDER_STATUS_OK);
}

/* Lets go of the listeners of the members that this instance owns; the listeners of another owner stay. A channel that
 * still streams refuses and the instance keeps holding it, so the task tries again on its next pass. */
static void step_generator_task_release_channels(StepGeneratorInstance_t *self) {
    if (self->channels_held == 0U) {
        return;
    }
    uint8_t all_released = 1U;
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        const StepStreamAxisStatus_t listener_status = step_stream_axis_release_listener(
            step_generator_task_member_axis(self, member),
            &self->member_contexts[member]
        );
        const uint8_t limit_released =
            motion_axis_hardware_set_stream_limit_listener(self->member_axes[member], NULL, self);
        if (listener_status != STEP_STREAM_AXIS_STATUS_OK || limit_released == 0U) {
            all_released = 0U;
        }
    }
    if (all_released != 0U) {
        self->channels_held = 0U;
    }
}

/* Takes the channels of the members and the switch listeners of their axes for this instance. A channel that streams,
 * or that another instance holds, refuses and nothing stays taken: the members taken so far are let go. An instance
 * that holds its channels takes them again without change. */
static uint8_t step_generator_task_claim_channels(StepGeneratorInstance_t *self) {
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        const uint8_t taken = (uint8_t)(step_stream_axis_set_listener(
                                            step_generator_task_member_axis(self, member),
                                            &self->stream_listeners[member]
                                        ) == STEP_STREAM_AXIS_STATUS_OK &&
                                        motion_axis_hardware_set_stream_limit_listener(
                                            self->member_axes[member],
                                            step_generator_task_limit_reached,
                                            self
                                        ) != 0U);
        if (taken == 0U) {
            /* The release lets go of this instance's own listeners only, so the members it never took stay as they
             * are. */
            self->channels_held = 1U;
            step_generator_task_release_channels(self);
            return 0U;
        }
    }
    self->channels_held = 1U;
    return 1U;
}

/* Builds the feeder of the instance; one that cannot refuses all work instead of guessing. The channels are not taken
 * here: the instance takes them each time it opens a stream. */
static void step_generator_task_setup_instance(
    StepGeneratorInstance_t *self,
    const StepGeneratorTaskConfig_t *config,
    PlanBuffer_t *plan_buffer
) {
    self->ready = step_generator_task_setup_feeder(self, config, plan_buffer);
    if (self->ready == 0U) {
        step_generator_counter_increment(&self->diagnostics.generator_errors);
        step_generator_task_set_halted(
            self,
            STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED,
            STEP_GENERATOR_TASK_NO_MEMBER,
            0U
        );
    }
}

/* One pass over an instance: the reset requested after STOP, then the feeding. */
static void step_generator_task_run_instance(StepGeneratorInstance_t *self) {
    uint8_t reset_requested = 0U;
    if (self->ready != 0U) {
        /* Take the request and lift the halt in one step: a request that arrives meanwhile is kept for the next pass,
         * and a STOP that arrives after this stays in force. */
        const uint32_t primask = step_stream_axis_enter_critical();
        reset_requested = self->reset_requested;
        if (reset_requested != 0U) {
            self->reset_requested = 0U;
            self->halted = 0U;
        }
        step_stream_axis_exit_critical(primask);
    }
    if (reset_requested != 0U) {
        /* The axis has stopped and this task is not filling, so the feeder owns no running stream. */
        step_generator_task_watch_all_axes(self);
        step_stream_feeder_reset(&self->feeder);
        if (step_stream_group_reset(&self->group) != STEP_STREAM_GROUP_STATUS_OK) {
            /* A channel still runs after STOP: nothing may be fed until the next STOP. */
            step_generator_counter_increment(&self->reset_failures);
            step_generator_task_set_halted(
                self,
                STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED,
                STEP_GENERATOR_TASK_NO_MEMBER,
                0U
            );
        } else {
            /* Every channel is at rest: other instances may take them. */
            step_generator_task_release_channels(self);
        }
        self->stream_busy = 0U;
        step_generator_counter_increment(&self->diagnostics.stop_resets);
    }
    if (self->ready != 0U && self->halted == 0U) {
        const StepStreamFeederStatus_t feeder_status = step_stream_feeder_service(&self->feeder);
        if (feeder_status != STEP_STREAM_FEEDER_STATUS_OK) {
            /* The feeder has stopped the axis; the plan is untrustworthy until STOP. */
            self->feeder_status = (uint32_t)feeder_status;
            step_generator_task_set_halted(
                self,
                STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
                STEP_GENERATOR_TASK_NO_MEMBER,
                0U
            );
        }
        (void)step_stream_group_service(&self->group);
        if (step_stream_feeder_get_phase(&self->feeder) == STEP_STREAM_FEEDER_PHASE_IDLE) {
            self->stream_busy = 0U;
            /* The stream has ended; the release at its stop may have found a channel still braking. */
            step_generator_task_release_channels(self);
        }
    }
    /* Also while halted: the counters of the stream that was cut off must reach the diagnostics. */
    if (self->ready != 0U) {
        step_generator_task_publish_diagnostics(self);
    }
}

void StepGeneratorTask(void *argument) {
    step_generator_task_thread_id = osThreadGetId();
    const StepGeneratorTaskConfig_t *const config = argument;
    step_generator_task_setup_instance(
        &step_generator_task_instance,
        config,
        config == NULL ? NULL : config->plan_buffer
    );
    for (uint32_t axis = 0U; axis < STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT; ++axis) {
        step_generator_task_setup_instance(
            &step_generator_task_axis_instances[axis],
            config,
            config == NULL ? NULL : config->axis_plan_buffers[axis]
        );
    }

    for (;;) {
        for (uint32_t index = 0U; index < STEP_GENERATOR_TASK_INSTANCE_COUNT; ++index) {
            step_generator_task_run_instance(step_generator_task_instances[index]);
        }
        /* Flags set while filling stay pending, so a block or a released half that arrives meanwhile is not missed. */
        const uint32_t flags = osThreadFlagsWait(STEP_GENERATOR_TASK_FLAGS_ALL, osFlagsWaitAny, osWaitForever);
        configASSERT((flags & osFlagsError) == 0U);
    }
}

void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    const StepGeneratorInstance_t *const self = &step_generator_task_instance;
    diagnostics->blocks_started = self->diagnostics.blocks_started;
    diagnostics->blocks_completed = self->diagnostics.blocks_completed;
    diagnostics->halves_filled = self->diagnostics.halves_filled;
    diagnostics->generator_errors = self->diagnostics.generator_errors;
    diagnostics->axis_errors = self->diagnostics.axis_errors;
    diagnostics->streams_started = self->diagnostics.streams_started;
    diagnostics->streams_stopped = self->diagnostics.streams_stopped;
    diagnostics->stop_resets = self->diagnostics.stop_resets;
}

void step_generator_task_get_stream_diagnostics(StepGeneratorTaskStreamDiagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    const StepGeneratorInstance_t *const self = &step_generator_task_instance;
    diagnostics->halted = self->halted;
    diagnostics->halt_cause = self->halt_cause;
    diagnostics->halt_member = self->halt_member;
    diagnostics->halt_fault = self->halt_fault;
    diagnostics->halts_total = self->halts_total;
    diagnostics->feeder_status = self->feeder_status;
    diagnostics->feeder_phase = (uint32_t)step_stream_feeder_get_phase(&self->feeder);
    for (uint32_t member = 0U; member < self->member_count; ++member) {
        const StepStreamAxis_t *channel = step_generator_task_member_axis(self, member);
        const MotionAxis_t axis = self->member_axes[member];
        diagnostics->members[member] = (StepGeneratorTaskMemberDiagnostics_t){
            .channel_state = (uint32_t)step_stream_axis_get_state(channel),
            .channel_fault = (uint32_t)step_stream_axis_get_fault(channel),
            .hardware_status = (uint32_t)motion_axis_hardware_get_status(axis),
            .hardware_stop_reason = (uint32_t)motion_axis_hardware_get_last_stop_reason(axis),
            .limit_trips = self->limit_trips[member],
        };
    }
}

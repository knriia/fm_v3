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

/* Axes mask of motion_axis_hardware_set_stream_watched_axes that watches every axis. */
#define STEP_GENERATOR_TASK_ALL_AXES_WATCHED 0xFFU

#define STEP_GENERATOR_TASK_FLAG_BLOCK_READY (1UL << 0)
#define STEP_GENERATOR_TASK_FLAG_STOP_RESET (1UL << 1)
#define STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED (1UL << 2)
#define STEP_GENERATOR_TASK_FLAGS_ALL                                                                                  \
    (STEP_GENERATOR_TASK_FLAG_BLOCK_READY | STEP_GENERATOR_TASK_FLAG_STOP_RESET |                                      \
        STEP_GENERATOR_TASK_FLAG_RING_HALF_RELEASED)

static volatile StepGeneratorTaskDiagnostics step_generator_task_diagnostics;
/* Owned by the task: only StepGeneratorTask touches it. */
static StepStreamFeeder_t step_generator_task_feeder;
/* Written once at task start and read by MotionTask and the DMA interrupt to wake the task. */
static osThreadId_t volatile step_generator_task_thread_id;
/* From the moment a block leaves the plan buffer until the stream has ended; STOP resets it. */
static volatile uint8_t step_generator_task_stream_busy;
/* Set by STOP in another task or by a ring fault in the DMA interrupt; cleared by this task once it is reset. */
static volatile uint8_t step_generator_task_halted;
/* Set once at start when the axis refused the braking ramp; counted into generator_errors with the feeder's errors. */
static volatile uint8_t step_generator_task_brake_setup_failed;

/* The stream channel of the axis; the instance lives for the whole run, so this is a lookup and no state. */
static StepStreamAxis_t *step_generator_task_axis(void) { return step_stream_axis_get(STEP_GENERATOR_TASK_AXIS); }

/* The channels that move together: member 0 is X, member 1 is Y, member 2 is Z. The group is defined with its
 * initializer below, so it exists before the task starts and a STOP that comes earlier still reaches the axes. */
#define STEP_GENERATOR_TASK_MEMBER_COUNT 3U
static StepStreamGroup_t step_generator_task_group;
/* The member index that each channel reports with; the listener context of a channel points at its entry. */
static const uint32_t step_generator_task_member_ids[STEP_GENERATOR_TASK_MEMBER_COUNT] = {0U, 1U, 2U};
static const MotionAxis_t step_generator_task_member_axes[STEP_GENERATOR_TASK_MEMBER_COUNT] = {
    MOTION_AXIS_X,
    MOTION_AXIS_Y,
    MOTION_AXIS_Z,
};
/* The planner axis behind each member, in the order of the members. */
static const uint32_t step_generator_task_member_planner_axes[STEP_GENERATOR_TASK_MEMBER_COUNT] = {0U, 1U, 2U};

/* The group passes only members below its count, so the index is always inside the table. */
static StepStreamAxis_t *step_generator_task_member_axis(uint32_t member) {
    return step_stream_axis_get(step_generator_task_member_axes[member]);
}

static void step_generator_counter_increment(volatile uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

_Static_assert(STEP_GENERATOR_TASK_MEMBER_COUNT == STEP_GENERATOR_TASK_STREAM_MEMBERS, "diagnostics list every member");
/* Why the feeding stopped: the first cause of the latest halt, kept after the reset that lifts it. Written inside a
 * critical section where halted is set, read only for diagnostics. */
static volatile uint32_t step_generator_task_halt_cause;
static volatile uint32_t step_generator_task_halt_member = STEP_GENERATOR_TASK_NO_MEMBER;
static volatile uint32_t step_generator_task_halt_fault;
static volatile uint32_t step_generator_task_halts_total;
/* Written by the generator task only. */
static volatile uint32_t step_generator_task_feeder_status;
/* Resets after STOP that left a channel running; counted into axis_errors with the feeder's errors. */
static volatile uint32_t step_generator_task_reset_failures;
/* Counted inside a critical section: the switch interrupt and the poll at the stream start both report. */
static volatile uint32_t step_generator_task_limit_trips[STEP_GENERATOR_TASK_MEMBER_COUNT];

/* The only place that sets halted, so every halt leaves its cause behind. Callable from tasks and interrupts. Only the
 * first halt since the last reset records its cause: a later one (MotionTask ending the move after a switch stopped it)
 * would hide what stopped the stream. Every halt is counted. */
static void step_generator_task_set_halted(uint32_t cause, uint32_t member, uint32_t fault) {
    const uint32_t primask = step_stream_axis_enter_critical();
    if (step_generator_task_halted == 0U) {
        step_generator_task_halt_cause = cause;
        step_generator_task_halt_member = member;
        step_generator_task_halt_fault = fault;
    }
    step_generator_counter_increment(&step_generator_task_halts_total);
    step_generator_task_halted = 1U;
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

void step_generator_task_halt(void) {
    step_generator_task_set_halted(STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL, STEP_GENERATOR_TASK_NO_MEMBER, 0U);
    /* The axes stop themselves even if the task is in the middle of a fill: the start port checks the flag with task
     * switching paused, so a stream cannot begin after this stop. */
    step_stream_group_stop(&step_generator_task_group);
}

uint8_t step_generator_task_is_busy(void) {
    PlanBufferDiagnostics_t plan = {0};
    /* Only a null pointer makes this fail, and the zeroed struct then reads as an empty plan. */
    (void)plan_buffer_get_diagnostics(&plan);
    return (uint8_t)(step_generator_task_stream_busy != 0U || plan.queued != 0U);
}

uint8_t step_generator_task_is_halted(void) { return step_generator_task_halted; }

void step_generator_task_resume_after_stop(void) { step_generator_task_signal(STEP_GENERATOR_TASK_FLAG_STOP_RESET); }

/* The ports below adapt the feeder to the stream axis and the plan buffer. */

static StepStreamFeederAxisResult_t step_generator_task_port_result(StepStreamAxisStatus_t status) {
    return status == STEP_STREAM_AXIS_STATUS_OK ? STEP_STREAM_FEEDER_AXIS_OK : STEP_STREAM_FEEDER_AXIS_ERROR;
}

static StepStreamFeederAxisResult_t step_generator_task_group_result(StepStreamGroupStatus_t status) {
    return status == STEP_STREAM_GROUP_STATUS_OK ? STEP_STREAM_FEEDER_AXIS_OK : STEP_STREAM_FEEDER_AXIS_ERROR;
}

/* Called while the feeder opens a stream, after it has decided which axes the blocks move. An axis that only idles in
 * the stream may stand on its switch; its limit must not stop the axes that move. */
static void step_generator_task_watch_moving_axes(void) {
    const uint8_t moving_planner_axes = step_stream_feeder_get_moving_axes(&step_generator_task_feeder);
    uint8_t watched = 0U;
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        if (((moving_planner_axes >> step_generator_task_member_planner_axes[member]) & 1U) != 0U) {
            watched = (uint8_t)(watched | (1U << (uint32_t)step_generator_task_member_axes[member]));
        }
    }
    motion_axis_hardware_set_stream_watched_axes(watched);
}

static StepStreamFeederAxisResult_t step_generator_task_port_prepare(void) {
    step_generator_task_watch_moving_axes();
    return step_generator_task_group_result(step_stream_group_prepare(&step_generator_task_group));
}

static StepStreamFeederAxisResult_t
step_generator_task_port_acquire_half(uint32_t lane, uint8_t *half, uint16_t **records) {
    const StepStreamAxisStatus_t status =
        step_stream_axis_acquire_half(step_generator_task_member_axis(lane), half, records);
    if (status == STEP_STREAM_AXIS_STATUS_NO_FILL_REQUEST) {
        return STEP_STREAM_FEEDER_AXIS_NO_REQUEST;
    }
    return step_generator_task_port_result(status);
}

static StepStreamFeederAxisResult_t step_generator_task_port_commit_half(uint32_t lane, uint8_t half) {
    return step_generator_task_port_result(step_stream_axis_commit_half(step_generator_task_member_axis(lane), half));
}

static StepStreamFeederAxisResult_t step_generator_task_port_set_direction(uint32_t lane, uint8_t positive) {
    /* With task switching paused, STOP and MotionTask cannot touch the axis between the flag check and the write. */
    MotionAxisHardwareStatus_t status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    (void)osKernelLock();
    if (step_generator_task_halted == 0U) {
        /* The feeder passes lanes below the member count only; this port needs the motion axis, not the channel. */
        status = motion_axis_hardware_set_stream_outputs(step_generator_task_member_axes[lane], positive);
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

static StepStreamFeederAxisResult_t step_generator_task_port_start(uint32_t startup_period_ns) {
    StepStreamGroupStatus_t status = STEP_STREAM_GROUP_STATUS_BAD_STATE;
    (void)osKernelLock();
    if (step_generator_task_halted == 0U) {
        status = step_stream_group_start(&step_generator_task_group, startup_period_ns);
    }
    (void)osKernelUnlock();
    return step_generator_task_group_result(status);
}

static void step_generator_task_port_stop(void) { step_stream_group_stop(&step_generator_task_group); }

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

static uint32_t step_generator_task_port_plan_epoch(void) { return plan_buffer_get_epoch(); }

static StepStreamFeederPlanResult_t
step_generator_task_port_peek_block(uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block) {
    return step_generator_task_plan_result(plan_buffer_try_peek(epoch, offset, block));
}

static StepStreamFeederPlanResult_t step_generator_task_port_discard_blocks(uint32_t epoch, uint32_t count) {
    /* Set before the blocks are out of sight, so HOME never sees an empty plan buffer and an idle stream in between. */
    step_generator_task_stream_busy = 1U;
    return step_generator_task_plan_result(plan_buffer_discard(epoch, count));
}

/* DMA interrupt: a half of the ring has been played. */
static void step_generator_task_notify_from_ring(void *context) {
    (void)context;
    step_generator_task_notify_ring_half_released();
}

/* DMA interrupt: a channel has stopped itself after a ring fault; the group tells the others and the owner. */
static void step_generator_task_channel_fault(void *context, StepStreamAxisFault_t fault) {
    step_stream_group_on_member_fault(&step_generator_task_group, *(const uint32_t *)context, (uint32_t)fault);
}

/* DMA interrupt: a channel brakes on a limit switch and refills its ring itself, so the feeder must not write. */
static void step_generator_task_channel_braking(void *context) {
    step_stream_group_on_member_braking(&step_generator_task_group, *(const uint32_t *)context);
}

/* Limit switch interrupt: the group decides which channels brake. */
static void step_generator_task_limit_reached(void *context, MotionAxis_t axis) {
    (void)context;
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        if (step_generator_task_member_axes[member] == axis) {
            const uint32_t primask = step_stream_axis_enter_critical();
            step_generator_counter_increment(&step_generator_task_limit_trips[member]);
            step_stream_axis_exit_critical(primask);
            step_stream_group_on_limit(&step_generator_task_group, step_generator_task_member_ids[member]);
            return;
        }
    }
}

/* Group, in an interrupt: the stream stopped for a limit switch or a fault, so stop feeding until STOP resets us. */
static void step_generator_task_group_member_stopping(void *context, uint32_t member) {
    (void)context;
    step_generator_task_set_halted(STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT, member, 0U);
}

static void step_generator_task_group_fault(void *context, uint32_t member, uint32_t fault) {
    (void)context;
    step_generator_task_set_halted(STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT, member, fault);
}

/* The group reaches the channels only through these. */
static StepStreamGroupPortResult_t step_generator_task_member_prepare(void *context, uint32_t member) {
    (void)context;
    return step_stream_axis_prepare_stream(step_generator_task_member_axis(member)) == STEP_STREAM_AXIS_STATUS_OK
               ? STEP_STREAM_GROUP_PORT_OK
               : STEP_STREAM_GROUP_PORT_ERROR;
}

static StepStreamGroupPortResult_t
step_generator_task_member_arm(void *context, uint32_t member, uint32_t startup_period_ns) {
    (void)context;
    return step_stream_axis_arm_stream(step_generator_task_member_axis(member), startup_period_ns) ==
                   STEP_STREAM_AXIS_STATUS_OK
               ? STEP_STREAM_GROUP_PORT_OK
               : STEP_STREAM_GROUP_PORT_ERROR;
}

static void step_generator_task_member_trigger(void *context) {
    (void)context;
    StepStreamAxis_t *axes[STEP_GENERATOR_TASK_MEMBER_COUNT];
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        axes[member] = step_generator_task_member_axis(member);
    }
    step_stream_axis_trigger(axes, STEP_GENERATOR_TASK_MEMBER_COUNT);
}

static void step_generator_task_member_stop(void *context, uint32_t member) {
    (void)context;
    step_stream_axis_stop(step_generator_task_member_axis(member));
}

static void step_generator_task_member_brake(void *context, uint32_t member) {
    (void)context;
    step_stream_axis_request_brake(step_generator_task_member_axis(member));
}

static uint8_t step_generator_task_member_is_active(void *context, uint32_t member) {
    (void)context;
    return step_stream_axis_get_state(step_generator_task_member_axis(member)) == STEP_STREAM_AXIS_STATE_STREAMING ? 1U
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

#define STEP_GENERATOR_TASK_GROUP_INITIAL                                                                              \
    {                                                                                                                  \
        .ports =                                                                                                       \
            {                                                                                                          \
                .prepare = step_generator_task_member_prepare,                                                         \
                .arm = step_generator_task_member_arm,                                                                 \
                .trigger = step_generator_task_member_trigger,                                                         \
                .stop = step_generator_task_member_stop,                                                               \
                .brake = step_generator_task_member_brake,                                                             \
                .is_active = step_generator_task_member_is_active,                                                     \
                .enter_critical = step_generator_task_critical_enter,                                                  \
                .exit_critical = step_generator_task_critical_exit,                                                    \
            },                                                                                                         \
        .listener =                                                                                                    \
            {                                                                                                          \
                .on_member_stopping = step_generator_task_group_member_stopping,                                       \
                .on_fault = step_generator_task_group_fault,                                                           \
            },                                                                                                         \
        .member_count = STEP_GENERATOR_TASK_MEMBER_COUNT,                                                              \
    }

static StepStreamGroup_t step_generator_task_group = STEP_GENERATOR_TASK_GROUP_INITIAL;

static void step_generator_task_publish_diagnostics(void) {
    StepStreamFeederDiagnostics_t feeder_diagnostics;
    step_stream_feeder_get_diagnostics(&step_generator_task_feeder, &feeder_diagnostics);
    step_generator_task_diagnostics.blocks_started = feeder_diagnostics.blocks_started;
    step_generator_task_diagnostics.blocks_completed = feeder_diagnostics.blocks_completed;
    step_generator_task_diagnostics.halves_filled = feeder_diagnostics.halves_filled;
    step_generator_task_diagnostics.generator_errors =
        step_generator_task_brake_setup_failed != 0U && feeder_diagnostics.generator_errors != UINT32_MAX
            ? feeder_diagnostics.generator_errors + 1U
            : feeder_diagnostics.generator_errors;
    const uint64_t axis_errors = (uint64_t)feeder_diagnostics.axis_errors + step_generator_task_reset_failures;
    step_generator_task_diagnostics.axis_errors = axis_errors > UINT32_MAX ? UINT32_MAX : (uint32_t)axis_errors;
    step_generator_task_diagnostics.streams_started = feeder_diagnostics.streams_started;
    step_generator_task_diagnostics.streams_stopped = feeder_diagnostics.streams_stopped;
}

/* Builds the braking ramp of the axis from the same timing as the feeder. Without a deceleration setting, or with
 * one the ramp cannot be built from, the axis keeps no ramp and stops at once on a limit switch; the latter is counted
 * in generator_errors. */
static void step_generator_task_setup_brake(
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
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        if (step_stream_axis_configure_brake(step_generator_task_member_axis(member), &brake) !=
            STEP_STREAM_AXIS_STATUS_OK) {
            step_generator_task_brake_setup_failed = 1U;
        }
    }
}

/* The feeder counts every record in the ticks of one clock, so the members must share the timer kernel clock. */
static uint8_t step_generator_task_member_clocks_match(void) {
    const uint32_t reference_hz = step_stream_axis_get_timer_clock_hz(step_generator_task_axis());
    for (uint32_t member = 1U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        if (step_stream_axis_get_timer_clock_hz(step_generator_task_member_axis(member)) != reference_hz) {
            return 0U;
        }
    }
    return 1U;
}

/* Builds the feeder from the driver timing. Returns 0 when the clock or the timing cannot be used. */
static uint8_t step_generator_task_setup_feeder(const StepGeneratorTaskConfig_t *task_config) {
    if (task_config == NULL || step_generator_task_member_clocks_match() == 0U) {
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
            step_generator_task_member_planner_axes,
            STEP_GENERATOR_TASK_MEMBER_COUNT,
            &config
        ) != STEP_STREAM_FEEDER_STATUS_OK) {
        return 0U;
    }
    static const StepStreamFeederPorts_t ports = {
        step_generator_task_port_prepare,
        step_generator_task_port_acquire_half,
        step_generator_task_port_commit_half,
        step_generator_task_port_set_direction,
        step_generator_task_port_start,
        step_generator_task_port_stop,
        step_generator_task_port_plan_epoch,
        step_generator_task_port_peek_block,
        step_generator_task_port_discard_blocks,
    };
    /* The ramp depends on the config only, so it is set up whether or not the feeder takes it. */
    step_generator_task_setup_brake(task_config, &config);
    return (uint8_t)(step_stream_feeder_init(&step_generator_task_feeder, &config, &ports) ==
                     STEP_STREAM_FEEDER_STATUS_OK);
}

void StepGeneratorTask(void *argument) {
    step_generator_task_thread_id = osThreadGetId();
    const uint8_t feeder_ready = step_generator_task_setup_feeder(argument);
    if (feeder_ready == 0U) {
        /* No usable timing: the task cannot feed the axis, so it refuses all work instead of guessing. */
        step_generator_counter_increment(&step_generator_task_diagnostics.generator_errors);
        step_generator_task_set_halted(STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED, STEP_GENERATOR_TASK_NO_MEMBER, 0U);
    }
    static const StepStreamAxisListener_t stream_listeners[STEP_GENERATOR_TASK_MEMBER_COUNT] = {
        {
            step_generator_task_notify_from_ring,
            step_generator_task_channel_fault,
            step_generator_task_channel_braking,
            (void *)&step_generator_task_member_ids[0],
        },
        {
            step_generator_task_notify_from_ring,
            step_generator_task_channel_fault,
            step_generator_task_channel_braking,
            (void *)&step_generator_task_member_ids[1],
        },
        {
            step_generator_task_notify_from_ring,
            step_generator_task_channel_fault,
            step_generator_task_channel_braking,
            (void *)&step_generator_task_member_ids[2],
        },
    };
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        (void)step_stream_axis_set_listener(step_generator_task_member_axis(member), &stream_listeners[member]);
    }
    motion_axis_hardware_set_stream_limit_listener(step_generator_task_limit_reached, NULL);

    uint32_t flags = 0U;
    for (;;) {
        if (feeder_ready != 0U && (flags & STEP_GENERATOR_TASK_FLAG_STOP_RESET) != 0U) {
            /* The axis has stopped and this task is not filling, so the feeder owns no running stream. */
            /* Lift the halt first: a STOP that arrives during the reset then stays in force. */
            step_generator_task_halted = 0U;
            /* No stream runs: watch every axis again until the next one narrows it to the axes it moves. */
            motion_axis_hardware_set_stream_watched_axes(STEP_GENERATOR_TASK_ALL_AXES_WATCHED);
            step_stream_feeder_reset(&step_generator_task_feeder);
            if (step_stream_group_reset(&step_generator_task_group) != STEP_STREAM_GROUP_STATUS_OK) {
                /* A channel still runs after STOP: nothing may be fed until the next STOP. */
                step_generator_counter_increment(&step_generator_task_reset_failures);
                step_generator_task_set_halted(
                    STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED,
                    STEP_GENERATOR_TASK_NO_MEMBER,
                    0U
                );
            }
            step_generator_task_stream_busy = 0U;
            step_generator_counter_increment(&step_generator_task_diagnostics.stop_resets);
        }
        if (feeder_ready != 0U && step_generator_task_halted == 0U) {
            const StepStreamFeederStatus_t feeder_status = step_stream_feeder_service(&step_generator_task_feeder);
            if (feeder_status != STEP_STREAM_FEEDER_STATUS_OK) {
                /* The feeder has stopped the axis; the plan is untrustworthy until STOP. */
                step_generator_task_feeder_status = (uint32_t)feeder_status;
                step_generator_task_set_halted(
                    STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR,
                    STEP_GENERATOR_TASK_NO_MEMBER,
                    0U
                );
            }
            (void)step_stream_group_service(&step_generator_task_group);
            if (step_stream_feeder_get_phase(&step_generator_task_feeder) == STEP_STREAM_FEEDER_PHASE_IDLE) {
                step_generator_task_stream_busy = 0U;
            }
        }
        /* Also while halted: the counters of the stream that was cut off must reach the diagnostics. */
        if (feeder_ready != 0U) {
            step_generator_task_publish_diagnostics();
        }
        /* Flags set while filling stay pending, so a block or a released half that arrives meanwhile is not missed. */
        flags = osThreadFlagsWait(STEP_GENERATOR_TASK_FLAGS_ALL, osFlagsWaitAny, osWaitForever);
        configASSERT((flags & osFlagsError) == 0U);
    }
}

void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    diagnostics->blocks_started = step_generator_task_diagnostics.blocks_started;
    diagnostics->blocks_completed = step_generator_task_diagnostics.blocks_completed;
    diagnostics->halves_filled = step_generator_task_diagnostics.halves_filled;
    diagnostics->generator_errors = step_generator_task_diagnostics.generator_errors;
    diagnostics->axis_errors = step_generator_task_diagnostics.axis_errors;
    diagnostics->streams_started = step_generator_task_diagnostics.streams_started;
    diagnostics->streams_stopped = step_generator_task_diagnostics.streams_stopped;
    diagnostics->stop_resets = step_generator_task_diagnostics.stop_resets;
}

void step_generator_task_get_stream_diagnostics(StepGeneratorTaskStreamDiagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    diagnostics->halted = step_generator_task_halted;
    diagnostics->halt_cause = step_generator_task_halt_cause;
    diagnostics->halt_member = step_generator_task_halt_member;
    diagnostics->halt_fault = step_generator_task_halt_fault;
    diagnostics->halts_total = step_generator_task_halts_total;
    diagnostics->feeder_status = step_generator_task_feeder_status;
    diagnostics->feeder_phase = (uint32_t)step_stream_feeder_get_phase(&step_generator_task_feeder);
    for (uint32_t member = 0U; member < STEP_GENERATOR_TASK_MEMBER_COUNT; ++member) {
        const StepStreamAxis_t *channel = step_generator_task_member_axis(member);
        const MotionAxis_t axis = step_generator_task_member_axes[member];
        diagnostics->members[member] = (StepGeneratorTaskMemberDiagnostics_t){
            .channel_state = (uint32_t)step_stream_axis_get_state(channel),
            .channel_fault = (uint32_t)step_stream_axis_get_fault(channel),
            .hardware_status = (uint32_t)motion_axis_hardware_get_status(axis),
            .hardware_stop_reason = (uint32_t)motion_axis_hardware_get_last_stop_reason(axis),
            .limit_trips = step_generator_task_limit_trips[member],
        };
    }
}

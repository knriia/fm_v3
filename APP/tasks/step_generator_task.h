#ifndef FM_V3_STEP_GENERATOR_TASK_H
#define FM_V3_STEP_GENERATOR_TASK_H

#include "motion/plan_buffer.h"
#include "motion_axis_hardware.h"

#include <stdint.h>

#define STEP_GENERATOR_TASK_STACK_SIZE_BYTES (1024U * 2U)

/* Layout is part of the diagnostic frame: eight 32-bit counters, field positions must not change. */
typedef struct {
    uint32_t blocks_started;
    uint32_t blocks_completed;
    /* Ring halves written and handed to the axis DMA. */
    uint32_t halves_filled;
    /* A block the generator rejected, or a configuration the task could not build. */
    uint32_t generator_errors;
    /* The axis refused a ring, direction, start or half operation. */
    uint32_t axis_errors;
    uint32_t streams_started;
    uint32_t streams_stopped;
    uint32_t stop_resets;
} StepGeneratorTaskDiagnostics;

/* Number of axes that stream together (X, Y, Z); the diagnostic frame carries one entry per axis. The syringe, the
 * fourth member of the printing stream, has no entry yet: its halts show in the fields of the stream, not in a member
 * entry. */
#define STEP_GENERATOR_TASK_STREAM_MEMBERS 3U
/* Who stopped the feeding the last time; 0 until the first halt. */
#define STEP_GENERATOR_TASK_HALT_CAUSE_NONE 0U
/* step_generator_task_halt: STOP, or MotionTask ending a move or an abort. */
#define STEP_GENERATOR_TASK_HALT_CAUSE_HALT_CALL 1U
/* A channel reported a limit switch it moves toward (or the start switch release). */
#define STEP_GENERATOR_TASK_HALT_CAUSE_LIMIT 2U
/* A channel stopped itself after a ring fault. */
#define STEP_GENERATOR_TASK_HALT_CAUSE_GROUP_FAULT 3U
/* The feeder service returned an error. */
#define STEP_GENERATOR_TASK_HALT_CAUSE_FEEDER_ERROR 4U
/* The task could not build its feeder. */
#define STEP_GENERATOR_TASK_HALT_CAUSE_SETUP_FAILED 5U
/* A channel still ran after the reset that follows STOP. */
#define STEP_GENERATOR_TASK_HALT_CAUSE_RESET_FAILED 6U
/* halt_member while no channel is to blame. */
#define STEP_GENERATOR_TASK_NO_MEMBER 0xFFFFFFFFU

/* One stream channel; layout is part of the diagnostic frame. */
typedef struct {
    /* StepStreamAxisState_t: 0 idle, 1 streaming. */
    uint32_t channel_state;
    /* StepStreamAxisFault_t of the last stop of the channel. */
    uint32_t channel_fault;
    /* MotionAxisHardwareStatus_t of the axis. */
    uint32_t hardware_status;
    /* MotionAxisHardwareStopReason_t recorded by the limit switch interrupt, not yet cleared by MotionTask. */
    uint32_t hardware_stop_reason;
    /* Limit switch reports of this channel since start. */
    uint32_t limit_trips;
} StepGeneratorTaskMemberDiagnostics_t;

/* Why the stream stops; layout is part of the diagnostic frame. */
typedef struct {
    uint32_t halted;
    /* STEP_GENERATOR_TASK_HALT_CAUSE_* of the first halt since the last reset: a later halt, e.g. MotionTask ending the
     * move after a switch stopped it, does not overwrite it. It stays after the reset, so with halted 0 it is old. */
    uint32_t halt_cause;
    /* Member (0 = X, 1 = Y, 2 = Z, 3 = syringe): for a limit the one whose switch fired first since the last reset, or
     * the braking channel when no switch was reported; for a fault the faulted one; STEP_GENERATOR_TASK_NO_MEMBER when
     * no channel is to blame. */
    uint32_t halt_member;
    /* StepStreamAxisFault_t of the latest group fault. */
    uint32_t halt_fault;
    /* Every halt, also those that kept the cause of an earlier one. */
    uint32_t halts_total;
    /* StepStreamFeederStatus_t of the latest feeder error since start; 0 while there was none. */
    uint32_t feeder_status;
    /* StepStreamFeederPhase_t. */
    uint32_t feeder_phase;
    StepGeneratorTaskMemberDiagnostics_t members[STEP_GENERATOR_TASK_STREAM_MEMBERS];
} StepGeneratorTaskStreamDiagnostics_t;

/* Instances with one lane each (X, Y, Z) that play the moves of HOME and CALIBRATE. */
#define STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT 3U

/* Step timing of the driver in nanoseconds; the task turns it into timer ticks of the stream axis. */
typedef struct {
    uint32_t step_pulse_width_ns;
    uint32_t step_low_time_ns;
    /* Deceleration of the braking on a limit switch; 0 leaves the axis without a ramp, so it stops at once. */
    uint32_t limit_deceleration_steps_per_s2;
    /* The queue of plan blocks the task plays; it outlives the task. Without it the task only reports an error. */
    PlanBuffer_t *plan_buffer;
    /* The queues of the one-lane instances, indexed by MotionAxis_t; they outlive the task. Without one the instance of
     * that axis only reports an error. */
    PlanBuffer_t *axis_plan_buffers[STEP_GENERATOR_TASK_AXIS_INSTANCE_COUNT];
} StepGeneratorTaskConfig_t;

/* Turns plan blocks into ring records of the stream channels (X, Y, Z and the syringe), keeps their rings filled and
 * starts and stops the streams.
 * argument is a const StepGeneratorTaskConfig_t * that outlives the task; without it the task only reports an error. */
void StepGeneratorTask(void *argument);
/* One stream: its feeder, channels, halt state and diagnostics. The task serves all instances; the type is opaque. */
typedef struct StepGeneratorInstance StepGeneratorInstance_t;

/* The instance of the printing stream (X, Y, Z and the syringe); the diagnostic frame carries its diagnostics. */
StepGeneratorInstance_t *step_generator_task_get_print_instance(void);
/* The one-lane instance of an axis: it plays the homing and calibration moves of that axis, so the axes of one command
 * run at the same time, each with its own plan queue. NULL for an axis outside MotionAxis_t. */
StepGeneratorInstance_t *step_generator_task_get_axis_instance(MotionAxis_t axis);
/* The queue of plan blocks the instance plays; NULL for a NULL instance and before the task has started. */
PlanBuffer_t *step_generator_instance_get_plan_buffer(const StepGeneratorInstance_t *instance);
/* Called by MotionTask after it pushes a block into PlanBuffer. */
void step_generator_task_notify_block_ready(void);
/* Called from the axis DMA interrupt when a ring half has been played and needs refilling. */
void step_generator_task_notify_ring_half_released(void);
/* What the instance has played and why it stopped: a snapshot that StepGeneratorTask is the only writer of. MotionTask
 * turns its changes into answers to the client. */
typedef struct {
    /* Increases with every STOP and client disconnect (step_generator_instance_halt): a snapshot of an older generation
     * belongs to a stream that was cancelled and its results must not be sent. Wraps. */
    uint32_t generation;
    /* Blocks played completely since the last reset of the instance and MotionPlanBlock_t.order of the last of them.
     * Both wrap; compare differences. */
    uint32_t played_blocks;
    uint32_t played_order;
    /* 1 once the instance is halted, no channel runs any more and the played blocks above are final. The fields below
     * are valid only then. */
    uint8_t stop_resolved;
    /* 1 when a block was cut off, and unplayed_order is the order of the first such block. */
    uint8_t has_unplayed;
    uint32_t unplayed_order;
    /* STEP_GENERATOR_TASK_HALT_CAUSE_* of the stop. */
    uint32_t halt_cause;
    /* MotionAxis_t that the stop is blamed on (the axis whose switch fired first, or the faulted channel), or
     * STEP_GENERATOR_TASK_NO_AXIS. */
    uint32_t halt_axis;
    /* For LIMIT MotionAxisHardwareStopReason_t (START_LIMIT 2, END_LIMIT 3), for GROUP_FAULT StepStreamAxisFault_t, for
     * FEEDER_ERROR StepStreamFeederStatus_t in the low nibble and the StepStreamGeneratorStatus_t that caused it in the
     * high one (shift STEP_GENERATOR_TASK_DETAIL_GENERATOR_SHIFT, 0 when the generator was not the cause); 0 for the
     * others. The whole value goes to a byte of the ERROR frame. */
    uint32_t halt_detail;
} StepGeneratorProgress_t;

/* halt_axis while no axis is to blame. */
#define STEP_GENERATOR_TASK_NO_AXIS 0xFFFFFFFFU
/* Where the generator status sits in the halt detail of a feeder error. */
#define STEP_GENERATOR_TASK_DETAIL_GENERATOR_SHIFT 4U

/* Called from StepGeneratorTask after the snapshot of the printing stream changed; must only signal and return. */
typedef void (*StepGeneratorProgressListener_t)(void);
void step_generator_task_set_progress_listener(StepGeneratorProgressListener_t listener);

/* Copies the snapshot in one piece. A NULL instance or progress does nothing. */
void step_generator_instance_get_progress(const StepGeneratorInstance_t *instance, StepGeneratorProgress_t *progress);
/* Stops the instance's channels and feeding right away; callable from any task. A NULL instance does nothing. */
void step_generator_instance_halt(StepGeneratorInstance_t *instance);
/* STOP: halts every instance. */
void step_generator_task_halt_all(void);
/* 1 from the moment a planned block is queued until its stream has ended (also after a fault, until STOP resets the
 * instance). Homing and calibration use the same timers as the stream, so they must not start while this is set. A
 * NULL instance is not busy. */
uint8_t step_generator_instance_is_busy(const StepGeneratorInstance_t *instance);
/* 1 while the instance does not feed its channels: after STOP, a stop on a limit switch or a fault, until the reset
 * below. A NULL instance reads as not halted. */
uint8_t step_generator_instance_is_halted(const StepGeneratorInstance_t *instance);
/* 1 once the feeder of the instance is built. One that is not ready cannot be reset by STOP and does nothing. A NULL
 * instance is not ready. */
uint8_t step_generator_instance_is_ready(const StepGeneratorInstance_t *instance);
/* How many resets after STOP the instance has finished; the count rises after the reset ended, so a reset that failed
 * has already halted the instance again. A NULL instance reads 0. */
uint32_t step_generator_instance_reset_count(const StepGeneratorInstance_t *instance);
/* The highest STEP rate in Hz that the generator of the instance plays: the timer clock over the longer of the pulse
 * plus the low time and the shortest record. A block with a higher rate would stop the stream with a feeder error. 0
 * for a NULL instance and for one that is not ready (its timing is not known). */
uint32_t step_generator_instance_get_max_step_rate_hz(const StepGeneratorInstance_t *instance);
/* After STOP, once the channels no longer move: drop the started block and the held one of this instance, then go on.
 * Other instances keep streaming. */
void step_generator_instance_resume_after_stop(StepGeneratorInstance_t *instance);
/* The reset of every instance after STOP. */
void step_generator_task_resume_all_after_stop(void);
void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics);
void step_generator_task_get_stream_diagnostics(StepGeneratorTaskStreamDiagnostics_t *diagnostics);

#endif /* FM_V3_STEP_GENERATOR_TASK_H */

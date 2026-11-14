#ifndef FM_V3_STEP_GENERATOR_TASK_H
#define FM_V3_STEP_GENERATOR_TASK_H

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

/* Number of channels that stream together (X, Y, Z); the diagnostic frame carries one entry per channel. */
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
    /* Member (0 = X, 1 = Y, 2 = Z) that reported the latest limit or fault, or STEP_GENERATOR_TASK_NO_MEMBER. */
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

/* Step timing of the driver in nanoseconds; the task turns it into timer ticks of the stream axis. */
typedef struct {
    uint32_t step_pulse_width_ns;
    uint32_t step_low_time_ns;
    /* Deceleration of the braking on a limit switch; 0 leaves the axis without a ramp, so it stops at once. */
    uint32_t limit_deceleration_steps_per_s2;
} StepGeneratorTaskConfig_t;

/* Turns plan blocks into ring records of the X stream axis, keeps its ring filled and starts and stops the stream.
 * argument is a const StepGeneratorTaskConfig_t * that outlives the task; without it the task only reports an error. */
void StepGeneratorTask(void *argument);
/* Called by MotionTask after it pushes a block into PlanBuffer. */
void step_generator_task_notify_block_ready(void);
/* Called from the axis DMA interrupt when a ring half has been played and needs refilling. */
void step_generator_task_notify_ring_half_released(void);
/* STOP: stops the stream axis and feeding right away; callable from any task. */
void step_generator_task_halt(void);
/* 1 from the moment a planned block is queued until its stream has ended (also after a fault, until STOP resets the
 * task). Homing and calibration use the same timer as the stream, so they must not start while this is set. */
uint8_t step_generator_task_is_busy(void);
/* 1 while the task does not feed the axis: after STOP, a stop on a limit switch or a fault, until the reset below. */
uint8_t step_generator_task_is_halted(void);
/* After STOP, once the axis no longer moves: drop the started block and the held one, then go on. */
void step_generator_task_resume_after_stop(void);
void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics);
void step_generator_task_get_stream_diagnostics(StepGeneratorTaskStreamDiagnostics_t *diagnostics);

#endif /* FM_V3_STEP_GENERATOR_TASK_H */

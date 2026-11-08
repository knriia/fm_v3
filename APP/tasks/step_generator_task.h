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

/* Step timing of the driver in nanoseconds; the task turns it into timer ticks of the stream axis. */
typedef struct {
    uint32_t step_pulse_width_ns;
    uint32_t step_low_time_ns;
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
/* After STOP, once the axis no longer moves: drop the started block and the held one, then go on. */
void step_generator_task_resume_after_stop(void);
void step_generator_task_get_diagnostics(StepGeneratorTaskDiagnostics *diagnostics);

#endif /* FM_V3_STEP_GENERATOR_TASK_H */

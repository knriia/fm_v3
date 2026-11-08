#ifndef FM_V3_STEP_STREAM_FEEDER_H
#define FM_V3_STEP_STREAM_FEEDER_H

#include "motion/motion_planner.h"
#include "motion/step_stream_generator.h"

#include <stddef.h>
#include <stdint.h>

/* Fills the ring of one axis from plan blocks. Pure logic: the axis and the plan buffer are reached only through
 * the ports below, so the module does not depend on the BSP. One owner task calls it; it is not reentrant. */

typedef enum {
    STEP_STREAM_FEEDER_AXIS_OK = 0,
    /* acquire_half: both halves are FULL or being played. */
    STEP_STREAM_FEEDER_AXIS_NO_REQUEST,
    STEP_STREAM_FEEDER_AXIS_ERROR
} StepStreamFeederAxisResult_t;

typedef struct {
    /* Resets the ring of an IDLE axis. */
    StepStreamFeederAxisResult_t (*prepare)(void);
    /* The first EMPTY half and the room for HALF_RECORD_COUNT records in it. */
    StepStreamFeederAxisResult_t (*acquire_half)(uint8_t *half, uint16_t **records);
    StepStreamFeederAxisResult_t (*commit_half)(uint8_t half);
    /* Called only while the axis is stopped, before prepare of the next stream. */
    StepStreamFeederAxisResult_t (*set_direction)(uint8_t positive);
    /* Both halves are FULL. */
    StepStreamFeederAxisResult_t (*start)(uint32_t startup_period_ns);
    /* Stops a streaming axis; harmless on an idle one. */
    void (*stop)(void);
    /* Next planned block, or 0 when there is none. */
    uint8_t (*pop_block)(MotionPlanBlock_t *block);
} StepStreamFeederPorts_t;

typedef struct {
    StepStreamGeneratorConfig_t generator;
    uint32_t axis_index;
    uint32_t startup_period_ns;
    /* Length of one filler record in an empty stretch. It bounds how long a new block waits behind an idle half:
     * about idle_record_ticks * records_per_half / timer_clock_hz. */
    uint32_t idle_record_ticks;
    /* Records in one ring half. */
    uint32_t half_record_count;
} StepStreamFeederConfig_t;

typedef enum {
    STEP_STREAM_FEEDER_STATUS_OK = 0,
    STEP_STREAM_FEEDER_STATUS_INVALID_ARGUMENT,
    /* Sticky until reset: the generator rejected a block or returned an error. */
    STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
    /* Sticky until reset: an axis port failed. */
    STEP_STREAM_FEEDER_STATUS_AXIS_ERROR
} StepStreamFeederStatus_t;

typedef enum {
    /* No stream: the axis is stopped. */
    STEP_STREAM_FEEDER_PHASE_IDLE = 0,
    /* Stream prepared, halves are being filled, the axis has not started. */
    STEP_STREAM_FEEDER_PHASE_PRIMING,
    STEP_STREAM_FEEDER_PHASE_STREAMING,
    STEP_STREAM_FEEDER_PHASE_FAULT
} StepStreamFeederPhase_t;

typedef struct {
    uint32_t blocks_started;
    uint32_t blocks_completed;
    uint32_t halves_filled;
    uint32_t streams_started;
    uint32_t streams_stopped;
    uint32_t generator_errors;
    uint32_t axis_errors;
} StepStreamFeederDiagnostics_t;

typedef struct {
    StepStreamFeederConfig_t config;
    StepStreamFeederPorts_t ports;
    StepStreamFeederPhase_t phase;
    /* What a FAULT phase reports until reset. */
    StepStreamFeederStatus_t fault_status;
    StepStreamGenerator_t generator;
    StepStreamGeneratorCarry_t carry;
    MotionPlanBlock_t held_block;
    uint8_t generator_active;
    uint8_t held_valid;
    /* Direction of the running stream; unknown while no block of it moved the axis. */
    uint8_t direction_known;
    uint8_t direction_positive;
    /* A half that carries records of a block, steps or waiting. A half with none is a filler stretch. */
    uint8_t half_has_block[2];
    StepStreamFeederDiagnostics_t diagnostics;
} StepStreamFeeder_t;

/* Converts the axis timing to timer ticks. min_record_ticks keeps one record longer than the DMA burst. */
StepStreamFeederStatus_t step_stream_feeder_make_generator_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    StepStreamGeneratorConfig_t *config
);

/* Whole feeder configuration from nanosecond timing. idle_record_ns is the length of one filler record. */
StepStreamFeederStatus_t step_stream_feeder_make_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    uint32_t idle_record_ns,
    uint32_t startup_period_ns,
    uint32_t half_record_count,
    uint32_t axis_index,
    StepStreamFeederConfig_t *config
);

StepStreamFeederStatus_t step_stream_feeder_init(
    StepStreamFeeder_t *feeder,
    const StepStreamFeederConfig_t *config,
    const StepStreamFeederPorts_t *ports
);

/* Drops the held block, the active block and any fault; the axis must already be stopped. */
void step_stream_feeder_reset(StepStreamFeeder_t *feeder);

/* Fills released halves, starts a primed stream and stops an idle one. A block whose direction differs from the
 * running stream waits until the stream has played an idle stretch and stopped, then opens a new stream. */
StepStreamFeederStatus_t step_stream_feeder_service(StepStreamFeeder_t *feeder);

StepStreamFeederPhase_t step_stream_feeder_get_phase(const StepStreamFeeder_t *feeder);
void step_stream_feeder_get_diagnostics(const StepStreamFeeder_t *feeder, StepStreamFeederDiagnostics_t *diagnostics);

#endif /* FM_V3_STEP_STREAM_FEEDER_H */

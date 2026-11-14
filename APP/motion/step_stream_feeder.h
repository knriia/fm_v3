#ifndef FM_V3_STEP_STREAM_FEEDER_H
#define FM_V3_STEP_STREAM_FEEDER_H

#include "motion/motion_planner.h"
#include "motion/step_stream_generator.h"

#include <stddef.h>
#include <stdint.h>

/* Fills the rings of the axes (lanes) of one stream from plan blocks. Pure logic: the axes and the plan buffer are
 * reached only through the ports below, so the module does not depend on the BSP. One owner task calls it; it is not
 * reentrant.
 *
 * Every lane walks the same sequence of blocks, one generator and one remainder per lane, so the lanes stay in step
 * by construction: all start together and each block takes the same time on every lane, whether the lane moves in it
 * or only waits. The lanes do not go through the blocks at the same pace, though: a half of the ring holds the same
 * number of records on every lane, but a lane with few steps fills it with long waits and so reaches far ahead in
 * time (up to the idle record length times the half size) while a lane with many steps is still in the first block.
 * So each lane keeps its own position in the plan buffer and the buffer keeps a block until every lane has started it.
 * The plan buffer must therefore hold the blocks of the longest half; a lane never pads a half with idle records while
 * a block is queued, because that would shift it against the other lanes.
 *
 * A clear of the plan buffer behind the stream's back (a planner error cancels the queue) is survived by a single lane,
 * which goes on with what is queued afterwards, and is a fault with several lanes: some of them may have started blocks
 * that the others cannot start any more. STOP halts the feeder before it clears, so it is not affected.
 *
 * The same reason closes a stream with several lanes at the end of the plan: the lane that looks first and finds no
 * block fills idle records for the rest of its half, so a block that comes later cannot start on all lanes together.
 * It starts the next stream, after every lane has played an idle stretch. A client has to keep the plan ahead of the
 * playback by the time the longest half spans (see StepStreamGeneratorConfig_t.max_record_ticks) to avoid the stop. A
 * single lane has nothing to get out of step with and takes a late block into the running stream. */

#define STEP_STREAM_FEEDER_MAX_LANES 4U

typedef enum {
    STEP_STREAM_FEEDER_AXIS_OK = 0,
    /* acquire_half: both halves are FULL or being played. */
    STEP_STREAM_FEEDER_AXIS_NO_REQUEST,
    STEP_STREAM_FEEDER_AXIS_ERROR
} StepStreamFeederAxisResult_t;

typedef enum {
    STEP_STREAM_FEEDER_PLAN_OK = 0,
    /* peek_block: no block at that offset. */
    STEP_STREAM_FEEDER_PLAN_NONE,
    /* The plan buffer was cleared since the epoch that the call carried. */
    STEP_STREAM_FEEDER_PLAN_STALE,
    STEP_STREAM_FEEDER_PLAN_ERROR
} StepStreamFeederPlanResult_t;

typedef struct {
    /* Resets the rings of every lane of an IDLE stream. */
    StepStreamFeederAxisResult_t (*prepare)(void);
    /* The first EMPTY half of the lane and the room for HALF_RECORD_COUNT records in it. */
    StepStreamFeederAxisResult_t (*acquire_half)(uint32_t lane, uint8_t *half, uint16_t **records);
    StepStreamFeederAxisResult_t (*commit_half)(uint32_t lane, uint8_t half);
    /* Called only while the stream is stopped, after prepare. */
    StepStreamFeederAxisResult_t (*set_direction)(uint32_t lane, uint8_t positive);
    /* Every lane has both halves FULL. */
    StepStreamFeederAxisResult_t (*start)(uint32_t startup_period_ns);
    /* Stops a running stream; harmless on an idle one. */
    void (*stop)(void);
    /* The three below are the plan buffer: its epoch, the block `offset` places behind the front (without taking it)
     * and the drop of the first `count` blocks. */
    uint32_t (*plan_epoch)(void);
    StepStreamFeederPlanResult_t (*peek_block)(uint32_t epoch, uint32_t offset, MotionPlanBlock_t *block);
    StepStreamFeederPlanResult_t (*discard_blocks)(uint32_t epoch, uint32_t count);
} StepStreamFeederPorts_t;

typedef struct {
    StepStreamGeneratorConfig_t generator;
    uint32_t lane_count;
    /* Planner axis that each lane drives. */
    uint32_t axis_index[STEP_STREAM_FEEDER_MAX_LANES];
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
    /* Sticky until reset: the generator rejected a block or returned an error, or the plan buffer could not be
     * trusted (a block that no lane can drive, a read that failed, a clear in the middle of a stream). */
    STEP_STREAM_FEEDER_STATUS_GENERATOR_ERROR,
    /* Sticky until reset: an axis port failed. */
    STEP_STREAM_FEEDER_STATUS_AXIS_ERROR
} StepStreamFeederStatus_t;

typedef enum {
    /* No stream: the axes are stopped. */
    STEP_STREAM_FEEDER_PHASE_IDLE = 0,
    /* Stream prepared, halves are being filled, the axes have not started. */
    STEP_STREAM_FEEDER_PHASE_PRIMING,
    STEP_STREAM_FEEDER_PHASE_STREAMING,
    STEP_STREAM_FEEDER_PHASE_FAULT
} StepStreamFeederPhase_t;

typedef struct {
    /* Counted on lane 0: every lane goes through the same blocks. */
    uint32_t blocks_started;
    uint32_t blocks_completed;
    uint32_t halves_filled;
    uint32_t streams_started;
    uint32_t streams_stopped;
    uint32_t generator_errors;
    uint32_t axis_errors;
} StepStreamFeederDiagnostics_t;

typedef struct {
    StepStreamGenerator_t generator;
    StepStreamGeneratorCarry_t carry;
    /* Index of the next block of this lane in the sequence of the stream; the block is `next_block - front_block`
     * places behind the front of the plan buffer. */
    uint32_t next_block;
    uint8_t generator_active;
    /* A half that carries records of a block, steps or waiting. A half with none is a filler stretch. */
    uint8_t half_has_block[2];
} StepStreamFeederLane_t;

typedef struct {
    StepStreamFeederConfig_t config;
    StepStreamFeederPorts_t ports;
    StepStreamFeederPhase_t phase;
    /* What a FAULT phase reports until reset. */
    StepStreamFeederStatus_t fault_status;
    StepStreamFeederLane_t lane[STEP_STREAM_FEEDER_MAX_LANES];
    /* Epoch of the plan buffer that the positions of this stream refer to. */
    uint32_t plan_epoch;
    /* Index, in the sequence of the stream, of the block at the front of the plan buffer. */
    uint32_t front_block;
    /* Where the plan ended for the first lane that looked: from this block on the stream takes no more blocks, even
     * ones that arrive later, so that every lane stops at the same block. UINT32_MAX while the stream is open, and
     * always with a single lane. */
    uint32_t end_block;
    /* Per planner axis: the direction of the stream is decided (known) and which it is (positive). DIR changes only
     * while the stream is stopped, so a block that moves an axis against it, or an axis whose direction was not
     * decided, waits for the next stream. */
    uint8_t direction_known_mask;
    uint8_t direction_positive_mask;
    /* Planner axes that a lane drives. */
    uint8_t lane_axes_mask;
    StepStreamFeederDiagnostics_t diagnostics;
} StepStreamFeeder_t;

/* Converts the axis timing to timer ticks. min_record_ticks keeps one record longer than the DMA burst. max_record_ns
 * is the longest record of a wait (0: the format maximum); see StepStreamGeneratorConfig_t. */
StepStreamFeederStatus_t step_stream_feeder_make_generator_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    uint32_t max_record_ns,
    StepStreamGeneratorConfig_t *config
);

/* Whole feeder configuration from nanosecond timing. idle_record_ns is the length of one filler record. The lanes are
 * the planner axes in axis_index, lane_count of them. */
StepStreamFeederStatus_t step_stream_feeder_make_config(
    uint32_t timer_clock_hz,
    uint32_t step_pulse_width_ns,
    uint32_t step_low_time_ns,
    uint32_t min_record_ns,
    uint32_t max_record_ns,
    uint32_t idle_record_ns,
    uint32_t startup_period_ns,
    uint32_t half_record_count,
    const uint32_t *axis_index,
    uint32_t lane_count,
    StepStreamFeederConfig_t *config
);

StepStreamFeederStatus_t step_stream_feeder_init(
    StepStreamFeeder_t *feeder,
    const StepStreamFeederConfig_t *config,
    const StepStreamFeederPorts_t *ports
);

/* Drops the lane positions, the active blocks and any fault; the axes must already be stopped. */
void step_stream_feeder_reset(StepStreamFeeder_t *feeder);

/* Fills released halves, starts a primed stream and stops an idle one. A block whose direction differs from the
 * running stream waits until every lane has played an idle stretch and the stream has stopped, then opens a new one. */
StepStreamFeederStatus_t step_stream_feeder_service(StepStreamFeeder_t *feeder);

StepStreamFeederPhase_t step_stream_feeder_get_phase(const StepStreamFeeder_t *feeder);
void step_stream_feeder_get_diagnostics(const StepStreamFeeder_t *feeder, StepStreamFeederDiagnostics_t *diagnostics);

#endif /* FM_V3_STEP_STREAM_FEEDER_H */

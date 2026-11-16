#include "command/command_queue.h"
#include "command/command_result_queue.h"
#include "motion/axis_bench.h"
#include "motion/axis_calibration_run.h"
#include "motion/calibration_store.h"
#include "motion/home_block.h"
#include "motion/plan_buffer.h"
#include "motion/step_executor.h"
#include "motion_axis_hardware.h"
#include "motion_task.h"
#include "step_generator_task.h"

#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    osStatus_t status;
    CommandDTO_t command;
} TestQueueRead_t;

static uint32_t test_failures;
static jmp_buf test_jump_buffer;
static uint8_t test_jump_active;
static uint8_t test_jump_on_delay;
static uint8_t test_block_when_empty;
/* While set, a scripted axis state does not change the busy flag of the generator instance of that axis: the axis
 * hardware is then active with an idle stream, which only the hardware status of MotionTask can tell. */
static uint8_t test_receive_keeps_generator_idle;
/* While set, the reset of the generator leaves it busy; it becomes idle after the given number of 1 ms waits, and
 * never with 0. */
static uint8_t test_generator_reset_keeps_busy;
static uint32_t test_generator_idle_after_delays;
static uint32_t test_delay_calls;
static uint32_t test_last_delay_ms;
static uint32_t test_receive_calls;
static uint32_t test_last_receive_timeout;
static uint32_t test_receive_script_count;
static uint32_t test_receive_script_position;
static TestQueueRead_t test_receive_script[16];
/* Called with the index of the read that is about to be served, to change the world while MotionTask is blocked. */
static void (*test_receive_hook)(uint32_t read_index);

/* Set while MotionTask runs. Outside of it command_queue_receive serves test_pending_commands instead of the script:
 * that is how motion_task_request_stop (called from another task) drains the queue. */
static uint8_t test_task_running;
#define TEST_PENDING_CAPACITY 8U
static CommandDTO_t test_pending_commands[TEST_PENDING_CAPACITY];
static uint32_t test_pending_count;
static uint32_t test_pending_position;
static uint32_t test_pending_receive_calls;
/* The largest timeout any receive outside of the task asked for. */
static uint32_t test_pending_max_timeout;

/* What MotionTask told the command result queue, in call order. */
#define TEST_RESULT_CAPACITY 64U
typedef struct {
    uint32_t sequence;
    uint8_t response_type;
    uint16_t code;
} TestResult_t;
static TestResult_t test_results[TEST_RESULT_CAPACITY];
static uint32_t test_result_count;
/* discard_all erases the recorded results, like the real queue; the count of calls and of the results it erased are kept. */
static uint32_t test_discard_calls;
static uint32_t test_discarded_results;

/* One faked generator instance: the three one-lane instances of the axes (indexed by MotionAxis_t) and the printing
 * stream behind them. Each has the flags the real task derives from its stream, its own halt and reset counts and, for
 * an axis, its own plan queue. The queue of the printing instance is test_plan_buffer. */
#define TEST_AXIS_QUEUE_CAPACITY 4U
struct StepGeneratorInstance {
    uint8_t busy;
    uint8_t halted;
    uint32_t halts;
    uint32_t resumes;
    /* enqueued_total of the queue at the last block notification; a larger value means a block was queued since. */
    uint32_t enqueued_seen;
    PlanBuffer_t queue;
    MotionPlanBlock_t entries[TEST_AXIS_QUEUE_CAPACITY];
};

#define TEST_PRINT_GENERATOR ((uint32_t)MOTION_AXIS_HARDWARE_COUNT)
#define TEST_GENERATOR_COUNT (TEST_PRINT_GENERATOR + 1U)
static StepGeneratorInstance_t fake_generators[TEST_GENERATOR_COUNT];

#define gen_x fake_generators[MOTION_AXIS_X]
#define gen_y fake_generators[MOTION_AXIS_Y]
#define gen_z fake_generators[MOTION_AXIS_Z]
#define gen_print fake_generators[TEST_PRINT_GENERATOR]

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

static void expect_i64(int64_t actual, int64_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (expected %lld, got %lld)\n", message, (long long)expected, (long long)actual);
    }
}

typedef struct {
    uint8_t apply;
    MotionAxisHardwareStatus_t status;
    MotionAxisHardwareStopReason_t stop_reason;
    uint8_t start_limit_active;
} TestXHardwareState_t;

/* State and call counts of one faked axis. */
typedef struct {
    MotionAxisHardwareStatus_t status;
    MotionAxisHardwareStopReason_t stop_reason;
    uint8_t start_limit_active;
    uint8_t end_limit_active;
    uint32_t init_calls;
    uint32_t init_receive_calls;
    uint32_t stop_requests;
    uint32_t clear_fault_calls;
    uint32_t timing_peak_resets;
    uint32_t ready_after_home_calls;
    /* Moves of the axis, each one plan block built by home_block_make. */
    uint32_t bench_starts;
    int32_t bench_last_signed_steps;
    uint32_t bench_last_target_rate_hz;
    uint32_t release_stop_arms;
    uint32_t release_stop_disarms;
    uint32_t stop_reason_clears;
    /* 1-based index of the move whose block cannot be built. */
    uint32_t failing_start;
    MotionAxisHardwareStartLimitHook_t start_limit_hook;
    int32_t encoder_um;
    /* Hardware state applied just before the matching scripted queue read, i.e. while MotionTask is blocked. */
    TestXHardwareState_t state_script[16];
} FakeAxis_t;

static FakeAxis_t fake_axes[MOTION_AXIS_HARDWARE_COUNT];

/* Most tests drive X; these names keep them short. */
#define fake_x_status fake_axes[MOTION_AXIS_X].status
#define fake_x_stop_reason fake_axes[MOTION_AXIS_X].stop_reason
#define fake_x_start_limit_active fake_axes[MOTION_AXIS_X].start_limit_active
#define fake_x_end_limit_active fake_axes[MOTION_AXIS_X].end_limit_active
#define fake_x_init_calls fake_axes[MOTION_AXIS_X].init_calls
#define fake_x_init_receive_calls fake_axes[MOTION_AXIS_X].init_receive_calls
#define fake_x_stop_requests fake_axes[MOTION_AXIS_X].stop_requests
#define fake_x_clear_fault_calls fake_axes[MOTION_AXIS_X].clear_fault_calls
#define fake_x_ready_after_home_calls fake_axes[MOTION_AXIS_X].ready_after_home_calls
#define fake_x_bench_starts fake_axes[MOTION_AXIS_X].bench_starts
#define fake_x_bench_last_signed_steps fake_axes[MOTION_AXIS_X].bench_last_signed_steps
#define fake_x_stop_reason_clears fake_axes[MOTION_AXIS_X].stop_reason_clears
#define fake_x_release_stop_arms fake_axes[MOTION_AXIS_X].release_stop_arms
#define fake_x_failing_start fake_axes[MOTION_AXIS_X].failing_start
#define fake_x_start_limit_hook fake_axes[MOTION_AXIS_X].start_limit_hook
#define fake_encoder_x_um fake_axes[MOTION_AXIS_X].encoder_um
#define test_x_state_script fake_axes[MOTION_AXIS_X].state_script
#define fake_y fake_axes[MOTION_AXIS_Y]
#define fake_z fake_axes[MOTION_AXIS_Z]

static void script_axis_state(
    MotionAxis_t axis,
    uint32_t index,
    MotionAxisHardwareStatus_t status,
    MotionAxisHardwareStopReason_t stop_reason,
    uint8_t start_limit_active
) {
    fake_axes[axis].state_script[index] = (TestXHardwareState_t){
        .apply = 1U,
        .status = status,
        .stop_reason = stop_reason,
        .start_limit_active = start_limit_active,
    };
}

static void script_x_state(
    uint32_t index,
    MotionAxisHardwareStatus_t status,
    MotionAxisHardwareStopReason_t stop_reason,
    uint8_t start_limit_active
) {
    script_axis_state(MOTION_AXIS_X, index, status, stop_reason, start_limit_active);
}

osStatus_t command_queue_receive(CommandDTO_t *command, uint32_t timeout_ms) {
    if (test_task_running == 0U) {
        ++test_pending_receive_calls;
        if (timeout_ms > test_pending_max_timeout) {
            test_pending_max_timeout = timeout_ms;
        }
        if (test_pending_position < test_pending_count) {
            *command = test_pending_commands[test_pending_position++];
            return osOK;
        }
        return osErrorResource;
    }
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        FakeAxis_t *fake = &fake_axes[axis];
        if (test_receive_calls < 16U && fake->state_script[test_receive_calls].apply != 0U) {
            const TestXHardwareState_t *state = &fake->state_script[test_receive_calls];
            fake->status = state->status;
            fake->stop_reason = state->stop_reason;
            fake->start_limit_active = state->start_limit_active;
            /* A homing stream plays while its axis (X, Y or Z) is active and is over once it is not. */
            if (test_receive_keeps_generator_idle == 0U) {
                fake_generators[axis].busy = (state->status == MOTION_AXIS_HARDWARE_STATUS_STARTING ||
                                              state->status == MOTION_AXIS_HARDWARE_STATUS_WAITING ||
                                              state->status == MOTION_AXIS_HARDWARE_STATUS_RUNNING)
                                                 ? 1U
                                                 : 0U;
            }
        }
    }
    if (test_receive_hook != NULL) {
        test_receive_hook(test_receive_calls);
    }
    ++test_receive_calls;
    test_last_receive_timeout = timeout_ms;
    if (test_receive_script_position < test_receive_script_count) {
        const TestQueueRead_t *read = &test_receive_script[test_receive_script_position++];
        if (read->status == osOK) {
            *command = read->command;
        }
        return read->status;
    }
    if (test_block_when_empty != 0U && test_jump_active != 0U) {
        longjmp(test_jump_buffer, 2);
    }
    return osErrorResource;
}

/* The plan buffer mutex is exercised in its own tests; here it only has to succeed. */
static uint8_t test_plan_buffer_mutex;

/* The queue every test context points to; the file keeps one, like the firmware does for printing. */
static PlanBuffer_t test_plan_buffer;
static MotionPlanBlock_t test_plan_buffer_entries[PLAN_BUFFER_CAPACITY];

static void reset_test_plan_buffer(void) {
    (void)memset(&test_plan_buffer, 0, sizeof(test_plan_buffer));
    (void)memset(test_plan_buffer_entries, 0, sizeof(test_plan_buffer_entries));
}

static uint32_t queued_blocks(PlanBuffer_t *queue) {
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(queue, &diagnostics);
    return diagnostics.queued;
}

static uint32_t enqueued_blocks(PlanBuffer_t *queue) {
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(queue, &diagnostics);
    return diagnostics.enqueued_total;
}

static uint32_t cancelled_blocks(PlanBuffer_t *queue) {
    PlanBufferDiagnostics_t diagnostics = {0};
    (void)plan_buffer_get_diagnostics(queue, &diagnostics);
    return diagnostics.cancelled_blocks;
}

osMutexId_t osMutexNew(const osMutexAttr_t *attr) {
    (void)attr;
    return &test_plan_buffer_mutex;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout) {
    (void)mutex_id;
    (void)timeout;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id) {
    (void)mutex_id;
    return osOK;
}

void Error_Handler(void) {}

static uint32_t test_block_ready_notifications;
static CalibrationStoreStatus_t test_calibration_load_status;
static MachineCalibration_t test_stored_calibration;

CalibrationStoreStatus_t calibration_store_load(MachineCalibration_t *calibration) {
    if (test_calibration_load_status == CALIBRATION_STORE_STATUS_OK) {
        *calibration = test_stored_calibration;
    }
    return test_calibration_load_status;
}

/* The queue an instance plays: its own, except for the printing instance, which plays the queue of the test context. */
static PlanBuffer_t *fake_generator_queue(const StepGeneratorInstance_t *instance) {
    const size_t index = (size_t)(instance - fake_generators);
    return index == TEST_PRINT_GENERATOR ? &test_plan_buffer : &fake_generators[index].queue;
}

/* A block queued for an instance makes it busy until its stream has ended or STOP has reset it. The real task finds
 * the instance by its queue; the fake does the same: it is the one whose queue has taken a block since the last call. */
void step_generator_task_notify_block_ready(void) {
    ++test_block_ready_notifications;
    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        StepGeneratorInstance_t *const generator = &fake_generators[index];
        PlanBufferDiagnostics_t diagnostics = {0};
        (void)plan_buffer_get_diagnostics(fake_generator_queue(generator), &diagnostics);
        if (diagnostics.enqueued_total > generator->enqueued_seen) {
            generator->busy = 1U;
        }
        generator->enqueued_seen = diagnostics.enqueued_total;
    }
}

/* The part of the halt and reset counts of every instance that came from STOP of all instances. */
static uint32_t test_generator_all_halts;
static uint32_t test_generator_all_resumes;
/* After this many 1 ms waits the X axis reports READY; 0 keeps it moving. */
static uint32_t test_axis_settles_after_delays;

StepGeneratorInstance_t *step_generator_task_get_print_instance(void) { return &gen_print; }

StepGeneratorInstance_t *step_generator_task_get_axis_instance(MotionAxis_t axis) {
    return (uint32_t)axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT ? &fake_generators[axis] : NULL;
}

PlanBuffer_t *step_generator_instance_get_plan_buffer(const StepGeneratorInstance_t *instance) {
    return instance == NULL ? NULL : fake_generator_queue(instance);
}

uint8_t step_generator_instance_is_busy(const StepGeneratorInstance_t *instance) { return instance->busy; }

uint8_t step_generator_instance_is_halted(const StepGeneratorInstance_t *instance) { return instance->halted; }

void step_generator_instance_halt(StepGeneratorInstance_t *instance) { ++instance->halts; }

void step_generator_task_halt_all(void) {
    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        ++fake_generators[index].halts;
    }
    ++test_generator_all_halts;
}

/* The reset of the real task leaves the instance idle. */
static void fake_generator_reset(StepGeneratorInstance_t *instance) {
    ++instance->resumes;
    if (test_generator_reset_keeps_busy == 0U) {
        instance->busy = 0U;
    }
    instance->halted = 0U;
}

void step_generator_instance_resume_after_stop(StepGeneratorInstance_t *instance) {
    fake_generator_reset(instance);
}

void step_generator_task_resume_all_after_stop(void) {
    ++test_generator_all_resumes;
    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        fake_generator_reset(&fake_generators[index]);
    }
}

void command_result_queue_post(uint32_t sequence, uint8_t response_type, uint16_t code) {
    if (test_result_count >= TEST_RESULT_CAPACITY) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: more results than the test records\n");
        return;
    }
    test_results[test_result_count].sequence = sequence;
    test_results[test_result_count].response_type = response_type;
    test_results[test_result_count].code = code;
    ++test_result_count;
}

void command_result_queue_discard_all(void) {
    ++test_discard_calls;
    test_discarded_results += test_result_count;
    test_result_count = 0U;
}

uint32_t osDelay(uint32_t milliseconds) {
    ++test_delay_calls;
    test_last_delay_ms = milliseconds;
    if (test_axis_settles_after_delays != 0U && test_delay_calls == test_axis_settles_after_delays) {
        fake_x_status = MOTION_AXIS_HARDWARE_STATUS_READY;
    }
    if (test_generator_idle_after_delays != 0U && test_delay_calls == test_generator_idle_after_delays) {
        gen_x.busy = 0U;
    }
    if (test_jump_active != 0U && test_jump_on_delay != 0U) {
        longjmp(test_jump_buffer, 3);
    }
    return 0U;
}

MotionAxisHardwareStatus_t motion_axis_hardware_init(MotionAxis_t axis, const MotionAxisHardwareConfig_t *config) {
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->init_calls;
    fake->init_receive_calls = test_receive_calls;
    fake->status = config != NULL ? MOTION_AXIS_HARDWARE_STATUS_READY : MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG;
    return fake->status;
}

static uint32_t test_event_source_registrations[MOTION_AXIS_HARDWARE_COUNT];
static void *test_event_source_contexts[MOTION_AXIS_HARDWARE_COUNT];
static uint32_t test_executor_underruns[MOTION_AXIS_HARDWARE_COUNT];

MotionAxisHardwareStatus_t motion_axis_hardware_set_event_source(MotionAxis_t axis, const MotionEventSource_t *source) {
    ++test_event_source_registrations[axis];
    test_event_source_contexts[axis] = source->context;
    return fake_axes[axis].status;
}

MotionEventSource_t step_executor_event_source(StepExecutor_t *executor) {
    return (MotionEventSource_t){.context = executor};
}

/* motion_task keeps one executor per axis in an array; the X source is bound to its first element. */
uint32_t step_executor_get_underrun_count(const StepExecutor_t *executor) {
    const StepExecutor_t *first = test_event_source_contexts[MOTION_AXIS_X];
    assert(first != NULL && executor >= first && executor < first + MOTION_AXIS_HARDWARE_COUNT);
    return test_executor_underruns[executor - first];
}

MotionAxisHardwareStatus_t motion_axis_hardware_get_status(MotionAxis_t axis) { return fake_axes[axis].status; }

MotionAxisHardwareStopReason_t motion_axis_hardware_get_last_stop_reason(MotionAxis_t axis) {
    return fake_axes[axis].stop_reason;
}

void motion_axis_hardware_request_stop(MotionAxis_t axis) { ++fake_axes[axis].stop_requests; }

MotionAxisHardwareStatus_t motion_axis_hardware_clear_fault(MotionAxis_t axis) {
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->clear_fault_calls;
    if (fake->status >= MOTION_AXIS_HARDWARE_STATUS_FAULT_LIMIT &&
        fake->status <= MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING) {
        fake->status = MOTION_AXIS_HARDWARE_STATUS_READY;
    }
    return fake->status;
}

MotionAxisHardwareStatus_t motion_axis_hardware_ready_after_home(MotionAxis_t axis) {
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->ready_after_home_calls;
    if (fake->status == MOTION_AXIS_HARDWARE_STATUS_STOPPED && fake->start_limit_active != 0U) {
        fake->status = MOTION_AXIS_HARDWARE_STATUS_READY;
    }
    return fake->status;
}

uint8_t motion_axis_hardware_start_limit_is_active(MotionAxis_t axis) { return fake_axes[axis].start_limit_active; }

uint8_t motion_axis_hardware_end_limit_is_active(MotionAxis_t axis) { return fake_axes[axis].end_limit_active; }

void motion_axis_hardware_arm_start_limit_release_stop(MotionAxis_t axis) { ++fake_axes[axis].release_stop_arms; }

void motion_axis_hardware_disarm_start_limit_release_stop(MotionAxis_t axis) { ++fake_axes[axis].release_stop_disarms; }

void motion_axis_hardware_arm_start_limit_hook(MotionAxis_t axis, MotionAxisHardwareStartLimitHook_t hook) {
    fake_axes[axis].start_limit_hook = hook;
}

void motion_axis_hardware_reset_timing_peaks(MotionAxis_t axis) { ++fake_axes[axis].timing_peak_resets; }

void encoder_ox_reset_coordinate(void) {}

int32_t encoder_ox_get_coordinate(void) { return fake_axes[MOTION_AXIS_X].encoder_um; }

void encoder_oy_reset_coordinate(void) {}

int32_t encoder_oy_get_coordinate(void) { return fake_axes[MOTION_AXIS_Y].encoder_um; }

void encoder_oz_reset_coordinate(void) {}

int32_t encoder_oz_get_coordinate(void) { return fake_axes[MOTION_AXIS_Z].encoder_um; }

/* The calibration procedure has its own tests; here it only reports scripted outcomes. */
static AxisCalibrationRunResult_t fake_calibration_begin_result;
static AxisCalibrationRunResult_t fake_calibration_poll_result;
static AxisCalibration_t fake_calibration_measured;
static uint32_t fake_calibration_begin_calls;
static uint32_t fake_calibration_poll_calls;
static uint32_t fake_calibration_abort_calls;
static MotionAxis_t fake_calibration_last_axis;

static const AxisCalibrationRunSettings_t *fake_calibration_last_settings;
static const AxisCalibrationRunMover_t *fake_calibration_last_mover;

/* Only its address matters here: every axis moves through the planned stream now, so MotionTask never hands this mover
 * to the procedure. The jog mover is covered through the axis_calibration_run tests. */
const AxisCalibrationRunMover_t axis_calibration_run_jog_mover = {0};

AxisCalibrationRunResult_t axis_calibration_run_begin(
    AxisCalibrationRun_t *run,
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *config,
    const AxisCalibrationRunSettings_t *settings,
    const AxisCalibrationRunMover_t *mover
) {
    (void)config;
    fake_calibration_last_settings = settings;
    fake_calibration_last_mover = mover;
    run->mover = mover;
    ++fake_calibration_begin_calls;
    fake_calibration_last_axis = axis;
    run->axis = axis;
    run->phase = fake_calibration_begin_result == AXIS_CALIBRATION_RUN_RESULT_RUNNING
                     ? AXIS_CALIBRATION_RUN_PHASE_SEEK_END
                     : AXIS_CALIBRATION_RUN_PHASE_IDLE;
    return fake_calibration_begin_result;
}

AxisCalibrationRunResult_t axis_calibration_run_poll(
    AxisCalibrationRun_t *run,
    const MotionAxisHardwareConfig_t *config,
    AxisCalibration_t *result
) {
    (void)config;
    ++fake_calibration_poll_calls;
    if (fake_calibration_poll_result != AXIS_CALIBRATION_RUN_RESULT_RUNNING) {
        run->phase = AXIS_CALIBRATION_RUN_PHASE_IDLE;
    }
    if (fake_calibration_poll_result == AXIS_CALIBRATION_RUN_RESULT_OK) {
        *result = fake_calibration_measured;
    }
    return fake_calibration_poll_result;
}

void axis_calibration_run_abort(AxisCalibrationRun_t *run) {
    ++fake_calibration_abort_calls;
    if (run->mover != NULL && run->mover->abort != NULL) {
        run->mover->abort(run->axis);
    }
    run->phase = AXIS_CALIBRATION_RUN_PHASE_IDLE;
}

uint8_t axis_calibration_run_is_active(const AxisCalibrationRun_t *run) {
    return run->phase != AXIS_CALIBRATION_RUN_PHASE_IDLE ? 1U : 0U;
}

/* All three axes move through the planned stream, so the ramped driver start is never used; a call is a failure.
 * Counted over the whole run and checked at its end. */
static uint32_t test_axis_bench_calls;

AxisBenchStatus_t axis_bench_start_profile(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t initial_step_rate_hz,
    uint32_t target_step_rate_hz,
    uint32_t acceleration_steps_per_s2,
    MotionAxisHardwareStatus_t *hardware_status
) {
    (void)axis;
    (void)hardware_config;
    (void)direction;
    (void)steps;
    (void)initial_step_rate_hz;
    (void)target_step_rate_hz;
    (void)acceleration_steps_per_s2;
    ++test_axis_bench_calls;
    *hardware_status = MOTION_AXIS_HARDWARE_STATUS_UNINITIALIZED;
    return AXIS_BENCH_STATUS_HARDWARE_ERROR;
}

void motion_axis_hardware_clear_stop_reason(MotionAxis_t axis) {
    ++fake_axes[axis].stop_reason_clears;
    fake_axes[axis].stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
}

/* X, Y and Z home through the stream, so a homing move of them shows up here. All axes are configured with positive
 * motion toward the end switch. A move counts as one start; the failing one builds no block. */
uint8_t home_block_make(
    MotionAxis_t axis,
    const MotionAxisHardwareConfig_t *hardware_config,
    AxisBenchDirection_t direction,
    uint32_t steps,
    uint32_t step_rate_hz,
    MotionPlanBlock_t *block
) {
    (void)hardware_config;
    FakeAxis_t *fake = &fake_axes[axis];
    ++fake->bench_starts;
    fake->bench_last_target_rate_hz = step_rate_hz;
    fake->bench_last_signed_steps = direction == AXIS_BENCH_TOWARD_END ? (int32_t)steps : -(int32_t)steps;
    if (fake->bench_starts == fake->failing_start) {
        return 0U;
    }
    *block = (MotionPlanBlock_t){.master_step_count = steps, .moving_axes_mask = (uint8_t)(1U << (uint32_t)axis)};
    return 1U;
}

static void reset_receive_script(void) {
    (void)memset(fake_axes, 0, sizeof(fake_axes));
    (void)memset(fake_generators, 0, sizeof(fake_generators));
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        (void)plan_buffer_init(
            &fake_generators[axis].queue,
            fake_generators[axis].entries,
            TEST_AXIS_QUEUE_CAPACITY
        );
    }
    test_receive_hook = NULL;
    test_receive_keeps_generator_idle = 0U;
    test_generator_reset_keeps_busy = 0U;
    test_generator_idle_after_delays = 0U;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        fake_axes[axis].init_receive_calls = UINT32_MAX;
    }
    test_jump_active = 0U;
    test_jump_on_delay = 0U;
    test_block_when_empty = 0U;
    test_delay_calls = 0U;
    test_last_delay_ms = 0U;
    test_receive_calls = 0U;
    test_last_receive_timeout = 0U;
    test_receive_script_count = 0U;
    test_receive_script_position = 0U;
    (void)memset(test_receive_script, 0, sizeof(test_receive_script));

    const MotionTaskDiagnostics diagnostics = {0};
    motion_task_test_seed_diagnostics(NULL);
    motion_task_test_seed_diagnostics(&diagnostics);
    motion_task_test_reset_planner();
    reset_test_plan_buffer();
    (void)plan_buffer_init(&test_plan_buffer, test_plan_buffer_entries, PLAN_BUFFER_CAPACITY);
    test_block_ready_notifications = 0U;
    test_generator_all_halts = 0U;
    test_generator_all_resumes = 0U;
    test_task_running = 0U;
    (void)memset(test_pending_commands, 0, sizeof(test_pending_commands));
    test_pending_count = 0U;
    test_pending_position = 0U;
    test_pending_receive_calls = 0U;
    test_pending_max_timeout = 0U;
    (void)memset(test_results, 0, sizeof(test_results));
    test_result_count = 0U;
    test_discard_calls = 0U;
    test_discarded_results = 0U;
    test_axis_settles_after_delays = 0U;
    test_calibration_load_status = CALIBRATION_STORE_STATUS_EMPTY;
    test_stored_calibration = (MachineCalibration_t){0};
    fake_calibration_begin_result = AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_RUNNING;
    fake_calibration_measured = (AxisCalibration_t){0};
    fake_calibration_begin_calls = 0U;
    fake_calibration_poll_calls = 0U;
    fake_calibration_abort_calls = 0U;
    fake_calibration_last_axis = MOTION_AXIS_X;
    fake_calibration_last_mover = NULL;
}

static void script_read(uint32_t index, osStatus_t status, const CommandDTO_t *command) {
    test_receive_script[index].status = status;
    if (command != NULL) {
        test_receive_script[index].command = *command;
    }
    if (test_receive_script_count <= index) {
        test_receive_script_count = index + 1U;
    }
}

static MotionTaskContext_t valid_motion_context(void) {
    MotionTaskContext_t context = {
        .machine_state =
            {
                .position_um = {0, 0, 0},
                .homed_axes_mask = MOTION_PLANNER_ALL_AXES_MASK,
            },
        .planner_config =
            {
                .max_step_event_rate_hz = 50000U,
            },
        .plan_buffer = &test_plan_buffer,
    };
    for (uint32_t axis = 0U; axis < MOTION_PLANNER_AXIS_COUNT; ++axis) {
        context.planner_config.axis[axis] = (MotionPlannerAxisConfig_t){
            .min_position_um = -100000,
            .max_position_um = 100000,
            .steps_per_um_numerator = 80U,
            .steps_per_um_denominator = 1000U,
            .max_speed_um_per_s = 100000U,
        };
    }
    return context;
}

static int run_motion_task_until_scripted_stop(MotionTaskContext_t *context) {
    test_jump_active = 1U;
    test_task_running = 1U;
    const int jumped = setjmp(test_jump_buffer);
    if (jumped == 0) {
        MotionTask(context);
    }
    test_task_running = 0U;
    test_jump_active = 0U;
    return jumped;
}

static void test_successful_receive_marks_dequeue(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 101U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1234,
                .y = 0,
                .z = 0,
                .speed = 12000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    expect_u32((uint32_t)run_motion_task_until_scripted_stop(&context), 3U, "queue error enters bounded retry path");
    expect_u32(test_last_receive_timeout, osWaitForever, "consumer waits forever for a queue item");
    expect_u32(test_delay_calls, 1U, "queue error applies one retry delay");
    expect_u32(test_last_delay_ms, MOTION_TASK_ERROR_RETRY_DELAY_MS, "retry delay is bounded");

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(NULL);
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, 1U, "successfully received command is counted as dequeued");
    expect_u32(diagnostics.queue_receive_errors, 1U, "queue receive error is counted");
    expect_u32(diagnostics.last_sequence, 101U, "last dequeued sequence is recorded");
    expect_u32(diagnostics.last_command_code, COMMAND_CODE_MOTION_OPERATION, "last dequeued command code is recorded");
    expect_u32(diagnostics.planner_calls, 1U, "planner call is counted");
    expect_u32(diagnostics.planner_blocks_created, 1U, "successful planner result is counted as a block");
    expect_u32(diagnostics.planner_no_motion, 0U, "moving command is not counted as no-motion");
    expect_u32(diagnostics.planner_errors, 0U, "successful planning is not counted as an error");
    expect_u32(diagnostics.last_planner_status, MOTION_PLANNER_STATUS_OK, "last planner status is recorded");
    expect_u32(diagnostics.last_planner_sequence, 101U, "last planner sequence is recorded");

    const MotionPlannerResult_t planner_result = motion_task_test_get_last_planner_result();
    expect_u32(motion_task_test_get_planner_call_count(), 1U, "XYZ command invokes MotionPlanner once");
    expect_u32(planner_result.status, MOTION_PLANNER_STATUS_OK, "valid dequeued XYZ command produces a plan");
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_OK,
        "last planner status is retained for callers"
    );
    expect_u32(planner_result.block.sequence, 101U, "planner receives the command sequence");
    expect_i64(planner_result.block.step_delta[0], 99, "planner receives the X target");
    expect_i64(planner_result.block.step_delta[1], 0, "Y stays in place");
    expect_i64(planner_result.block.step_delta[2], 0, "Z stays in place");
    expect_u32(planner_result.block.feedrate_um_per_s, 12000U, "planner receives the XYZ feedrate");
    expect_u32(test_result_count, 1U, "the accepted move is answered once");
    expect_u32(test_results[0].sequence, 101U, "the answer carries the sequence of the move");
    expect_u32(test_results[0].response_type, COMMAND_MESSAGE_TYPE_DONE, "an accepted move is answered with DONE");
    expect_u32(test_results[0].code, COMMAND_DONE_ACCEPTED, "an accepted move is answered with ACCEPTED");

    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    expect_u32(
        plan_buffer_get_diagnostics(&test_plan_buffer, &plan_buffer_diagnostics),
        PLAN_BUFFER_STATUS_OK,
        "plan buffer diagnostics are available"
    );
    expect_u32(plan_buffer_diagnostics.queued, 1U, "successful plan is queued");
    expect_u32(plan_buffer_diagnostics.enqueued_total, 1U, "successful plan enqueue is counted");
    expect_u32(test_block_ready_notifications, 1U, "step generator is woken for the queued plan");
    expect_u32((uint32_t)context.machine_state.position_um[0], 1234U, "planned X becomes the next start");
    expect_u32((uint32_t)context.machine_state.position_um[1], 0U, "planned Y is unchanged");
    expect_u32((uint32_t)context.machine_state.position_um[2], 0U, "planned Z is unchanged");
    MotionPlanBlock_t buffered_block = {0};
    expect_u32(
        plan_buffer_try_pop(&test_plan_buffer, &buffered_block),
        PLAN_BUFFER_STATUS_OK,
        "plan is available to the next pipeline block"
    );
    expect_u32(buffered_block.sequence, 101U, "plan buffer preserves source command sequence");
    expect_i64(buffered_block.step_delta[0], 99, "plan buffer preserves planner X output");
    expect_i64(buffered_block.step_delta[1], 0, "plan buffer preserves planner Y output");
    expect_i64(buffered_block.step_delta[2], 0, "plan buffer preserves planner Z output");
}

static void test_fifo_commands_are_dequeued_once(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 90U};
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &pending_plan), PLAN_BUFFER_STATUS_OK, "pending plan exists before STOP");
    const CommandDTO_t commands[] = {
        {.sequence = 102U, .code = COMMAND_CODE_HOME, .parameters.home = {.axes = 0x08U}},
        {.sequence = 103U, .code = COMMAND_CODE_STOP},
        {
            .sequence = 104U,
            .code = COMMAND_CODE_SET_TEMPERATURE,
            .parameters.set_temperature = {.heater_id = 2U, .temperature = 215U},
        },
        {.sequence = 105U, .code = COMMAND_CODE_SET_OUTPUT, .parameters.set_output = {.output_id = 4U, .state = 1U}},
        {.sequence = 106U, .code = COMMAND_CODE_CHANGE_TOOL, .parameters.change_tool = {.tool_id = 7U}},
    };
    for (uint32_t index = 0U; index < sizeof(commands) / sizeof(commands[0]); ++index) {
        script_read(index, osOK, &commands[index]);
    }
    test_block_when_empty = 1U;

    MotionTaskContext_t context = {.plan_buffer = &test_plan_buffer};
    expect_u32(
        (uint32_t)run_motion_task_until_scripted_stop(&context),
        2U,
        "consumer blocks again after draining scripted commands"
    );
    expect_u32(test_receive_calls, 6U, "consumer performs one receive per command then waits again");
    expect_u32(test_receive_script_position, 5U, "each scripted queue item is removed exactly once");
    expect_u32(test_last_receive_timeout, osWaitForever, "each consumer read blocks indefinitely");
    /* The refusal of HOME (102) is erased by STOP (103); the commands after STOP are answered as usual. */
    expect_u32(test_discard_calls, 1U, "STOP discards the earlier results once");
    expect_u32(test_discarded_results, 1U, "only the answer of HOME was waiting");
    expect_u32(test_result_count, 4U, "STOP and the three commands after it are answered");
    const uint8_t expected_types[4] = {
        COMMAND_MESSAGE_TYPE_DONE,
        COMMAND_MESSAGE_TYPE_ERROR,
        COMMAND_MESSAGE_TYPE_ERROR,
        COMMAND_MESSAGE_TYPE_ERROR,
    };
    const uint16_t expected_codes[4] = {COMMAND_DONE_ACCEPTED, 0x0124U, 0x0124U, 0x0124U};
    for (uint32_t index = 0U; index < 4U; ++index) {
        expect_u32(test_results[index].sequence, 103U + index, "results follow the command order");
        expect_u32(test_results[index].response_type, expected_types[index], "response type of the command");
        expect_u32(test_results[index].code, expected_codes[index], "result code of the command");
    }

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, 5U, "all valid queue items are counted as dequeued");
    expect_u32(diagnostics.queue_receive_errors, 0U, "blocking on an empty queue is not a receive error");
    expect_u32(diagnostics.last_sequence, 106U, "last dequeued sequence is retained");
    expect_u32(diagnostics.last_command_code, COMMAND_CODE_CHANGE_TOOL, "last dequeued command code is retained");
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "non-XYZ commands do not invoke MotionPlanner");
    expect_u32(diagnostics.planner_calls, 0U, "non-XYZ commands do not increment planner calls");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 0U, "STOP clears pending plans");
    expect_u32(plan_buffer_diagnostics.cancelled_blocks, 1U, "STOP counts cancelled plans");
}

static void test_receive_error_does_not_count_as_dequeue(void) {
    reset_receive_script();
    MotionTaskContext_t context = {.plan_buffer = &test_plan_buffer};
    script_read(0U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    expect_u32((uint32_t)run_motion_task_until_scripted_stop(&context), 3U, "receive error retries after delay");
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, 0U, "failed receive does not count as a dequeued command");
    expect_u32(diagnostics.queue_receive_errors, 1U, "failed receive increments error count");
    expect_u32(diagnostics.last_sequence, 0U, "failed receive does not change last sequence");
}

static void test_diagnostic_counters_saturate(void) {
    reset_receive_script();
    const MotionTaskDiagnostics saturated = {
        .commands_dequeued = UINT32_MAX,
        .queue_receive_errors = UINT32_MAX,
        .planner_calls = UINT32_MAX,
        .planner_blocks_created = UINT32_MAX,
        .planner_no_motion = UINT32_MAX,
        .planner_errors = UINT32_MAX,
    };
    motion_task_test_seed_diagnostics(&saturated);

    MotionTaskContext_t valid_context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 107U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1000,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&valid_context);

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.commands_dequeued, UINT32_MAX, "dequeued counter saturates");
    expect_u32(diagnostics.queue_receive_errors, UINT32_MAX, "queue receive error counter saturates");
    expect_u32(diagnostics.planner_calls, UINT32_MAX, "planner call counter saturates");
    expect_u32(diagnostics.planner_blocks_created, UINT32_MAX, "planner block counter saturates");
    expect_u32(diagnostics.planner_no_motion, UINT32_MAX, "planner no-motion counter remains saturated");
    expect_u32(diagnostics.planner_errors, UINT32_MAX, "planner error counter remains saturated");
}

static void test_xyz_command_reports_unconfigured_planner_inputs(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 91U};
    expect_u32(plan_buffer_try_push(&test_plan_buffer, &pending_plan), PLAN_BUFFER_STATUS_OK, "pending plan exists before planner error");
    MotionTaskContext_t context = {.plan_buffer = &test_plan_buffer};
    const CommandDTO_t command = {
        .sequence = 108U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1000,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(motion_task_test_get_planner_call_count(), 1U, "XYZ command reaches MotionPlanner with runtime context");
    expect_u32(
        motion_task_test_get_last_planner_result().status,
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "zero-initialized runtime configuration rejects planning safely"
    );
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "task exposes the runtime planner rejection status"
    );
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_calls, 1U, "unconfigured XYZ command increments planner calls");
    expect_u32(diagnostics.planner_blocks_created, 0U, "unconfigured XYZ command creates no block");
    expect_u32(diagnostics.planner_errors, 1U, "unconfigured XYZ command increments planner errors");
    expect_u32(
        diagnostics.last_planner_status,
        MOTION_PLANNER_STATUS_INVALID_CONFIGURATION,
        "invalid configuration status is recorded"
    );
    expect_u32(diagnostics.last_planner_sequence, 108U, "failed planner sequence is recorded");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 0U, "planner error clears pending plans");
    expect_u32(plan_buffer_diagnostics.cancelled_blocks, 1U, "planner error counts cancelled plans");
}

static void test_xyz_command_without_position_change_counts_no_motion(void) {
    reset_receive_script();
    const MotionPlanBlock_t pending_plan = {.sequence = 92U};
    expect_u32(
        plan_buffer_try_push(&test_plan_buffer, &pending_plan),
        PLAN_BUFFER_STATUS_OK,
        "pending plan exists before no-motion command"
    );
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 111U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 0,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_calls, 1U, "stationary XYZ command invokes planner");
    expect_u32(diagnostics.planner_blocks_created, 0U, "stationary XYZ command creates no block");
    expect_u32(diagnostics.planner_no_motion, 1U, "stationary XYZ command increments no-motion count");
    expect_u32(diagnostics.planner_errors, 0U, "no movement is not counted as a planner error");
    expect_u32(diagnostics.last_planner_status, MOTION_PLANNER_STATUS_NO_MOVEMENT, "no-movement status is recorded");
    expect_u32(diagnostics.last_planner_sequence, 111U, "no-movement sequence is recorded");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, 1U, "no-motion result preserves earlier plans");
    expect_u32(plan_buffer_diagnostics.enqueued_total, 1U, "no block is queued for a move to the current point");
    expect_u32(test_block_ready_notifications, 0U, "a move to the current point does not wake the step generator");
    expect_u32(test_result_count, 1U, "a move to the current point is answered once");
    expect_u32(test_results[0].sequence, 111U, "the answer carries the sequence of the move");
    expect_u32(test_results[0].response_type, COMMAND_MESSAGE_TYPE_DONE, "a move to the current point is answered with DONE");
    expect_u32(test_results[0].code, COMMAND_DONE_ALREADY_AT_TARGET, "the DONE says the axes are already there");
}

static void test_relative_move_without_increment_is_already_at_target(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[0] = 500;
    context.machine_state.position_um[1] = -700;
    context.machine_state.position_um[2] = 300;
    const CommandDTO_t command = {
        .sequence = 113U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE | COMMAND_MOTION_OPERATION_FLAG_RELATIVE,
                .x = 0,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_no_motion, 1U, "a zero relative move is counted as no-motion");
    expect_u32(diagnostics.planner_errors, 0U, "a zero relative move is not a planner error");
    expect_u32(enqueued_blocks(&test_plan_buffer), 0U, "a zero relative move queues no block");
    expect_u32(test_block_ready_notifications, 0U, "a zero relative move does not wake the step generator");
    expect_i64(context.machine_state.position_um[0], 500, "X is unchanged");
    expect_i64(context.machine_state.position_um[1], -700, "Y is unchanged");
    expect_i64(context.machine_state.position_um[2], 300, "Z is unchanged");
    expect_u32(test_result_count, 1U, "a zero relative move is answered once");
    expect_u32(test_results[0].sequence, 113U, "the answer carries the sequence of the move");
    expect_u32(test_results[0].response_type, COMMAND_MESSAGE_TYPE_DONE, "a zero relative move is answered with DONE");
    expect_u32(test_results[0].code, COMMAND_DONE_ALREADY_AT_TARGET, "a zero relative move is already at the target");
}

static void test_full_plan_buffer_rejects_new_block_without_losing_existing_plans(void) {
    reset_receive_script();
    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = {.sequence = sequence};
        expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "plan buffer fills before integration check");
    }

    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 112U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1234,
                .y = 0,
                .z = 0,
                .speed = 12000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    MotionTaskDiagnostics motion_diagnostics = {0};
    motion_task_get_diagnostics(&motion_diagnostics);
    expect_u32(motion_diagnostics.planner_blocks_created, 1U, "planner output remains distinct from buffer acceptance");
    PlanBufferDiagnostics_t plan_buffer_diagnostics = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan_buffer_diagnostics);
    expect_u32(plan_buffer_diagnostics.queued, PLAN_BUFFER_CAPACITY, "full buffer retains existing plans");
    expect_u32(
        plan_buffer_diagnostics.enqueued_total,
        PLAN_BUFFER_CAPACITY,
        "rejected plan is not counted as enqueued"
    );
    expect_u32(plan_buffer_diagnostics.full_rejections, 1U, "full plan buffer rejection is visible");
    expect_u32(test_block_ready_notifications, 0U, "rejected plan does not wake the step generator");

    MotionPlanBlock_t first_block = {0};
    expect_u32(
        plan_buffer_try_pop(&test_plan_buffer, &first_block),
        PLAN_BUFFER_STATUS_OK,
        "existing plans remain consumable after full rejection"
    );
    expect_u32(first_block.sequence, 1U, "full rejection does not replace earlier plans");
}

static void test_xyz_command_with_null_context_is_not_configured(void) {
    reset_receive_script();
    const CommandDTO_t command = {
        .sequence = 109U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = 1000,
                .y = 0,
                .z = 0,
                .speed = 1000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(NULL);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "MotionPlanner is not called without a task context");
    expect_u32(test_result_count, 1U, "an XYZ move without a task context is answered once");
    expect_u32(test_results[0].sequence, 109U, "the answer carries the sequence of the move");
    expect_u32(test_results[0].response_type, COMMAND_MESSAGE_TYPE_ERROR, "a move without a task context is refused");
    expect_u32(test_results[0].code, 0x0126U, "an XYZ move without a task context is not configured");
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_calls, 0U, "missing motion context is not counted as a planner call");
    expect_u32(diagnostics.planner_errors, 0U, "missing motion context is not a planner error");
    expect_u32(diagnostics.last_planner_sequence, 0U, "the planner never saw the move");
}

static void test_motion_operation_without_xyz_flag_does_not_call_planner(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t command = {
        .sequence = 110U,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = 0x04U,
                .spindle_pwm = 5000U,
            },
    };
    script_read(0U, osOK, &command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "non-XYZ operation is not sent to MotionPlanner");
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "planner status is unavailable before the first planner call"
    );
}

static void test_x_hardware_is_initialized_before_first_command(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_read(0U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_init_calls, 1U, "X hardware is initialized once at task start");
    expect_u32(fake_x_init_receive_calls, 0U, "X hardware is initialized before the first queue read");
    expect_u32(fake_x_bench_starts, 0U, "initialization does not start motion");
}

static void test_stream_event_source_is_registered_for_every_axis(void) {
    memset(test_event_source_registrations, 0, sizeof(test_event_source_registrations));
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_read(0U, osErrorResource, NULL);
    test_jump_on_delay = 1U;

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(test_event_source_registrations[MOTION_AXIS_X], 1U, "X gets an event source");
    expect_u32(test_event_source_registrations[MOTION_AXIS_Y], 1U, "Y gets an event source");
    expect_u32(test_event_source_registrations[MOTION_AXIS_Z], 1U, "Z gets an event source");
    expect_u32(test_event_source_contexts[MOTION_AXIS_X] != NULL, 1U, "the source is bound to an executor instance");
    expect_u32(
        test_event_source_contexts[MOTION_AXIS_Z] != test_event_source_contexts[MOTION_AXIS_Y] &&
            test_event_source_contexts[MOTION_AXIS_Z] != test_event_source_contexts[MOTION_AXIS_X],
        1U,
        "Z has an executor instance of its own"
    );

    test_executor_underruns[MOTION_AXIS_X] = 3U;
    test_executor_underruns[MOTION_AXIS_Y] = 4U;
    test_executor_underruns[MOTION_AXIS_Z] = 20U;
    expect_u32(motion_task_get_stream_underrun_count(), 27U, "underruns are summed over axes");
}

static void run_single_stop(MotionTaskContext_t *context, MotionAxisHardwareStatus_t status_after_init) {
    const CommandDTO_t stop = {
        .sequence = 120U,
        .code = COMMAND_CODE_STOP,
    };
    script_read(0U, osOK, &stop);
    test_block_when_empty = 1U;
    /* MotionTask initializes the hardware itself, so the state is injected on the first queue read. */
    script_x_state(0U, status_after_init, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    (void)run_motion_task_until_scripted_stop(context);
}

static const CommandDTO_t test_home_x_command = {
    .sequence = 130U,
    .code = COMMAND_CODE_HOME,
    .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK,
};

static const CommandDTO_t test_home_y_command = {
    .sequence = 150U,
    .code = COMMAND_CODE_HOME,
    .parameters.home.axes = MOTION_PLANNER_AXIS_Y_MASK,
};

static const CommandDTO_t test_home_z_command = {
    .sequence = 158U,
    .code = COMMAND_CODE_HOME,
    .parameters.home.axes = MOTION_PLANNER_AXIS_Z_MASK,
};

/* Read 0 delivers HOME; every later read is a 1 ms poll timeout preceded by the scripted hardware state. */
static void script_home_polls(uint32_t poll_count) {
    script_read(0U, osOK, &test_home_x_command);
    for (uint32_t index = 1U; index <= poll_count; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
}

static void test_home_fast_seek_moves_toward_start_and_backs_off_on_pf10(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(1U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "HOME starts fast seek and then backoff");
    expect_u32(fake_x_release_stop_arms, 1U, "backoff arms the PF10 release stop");
    expect_i64(fake_x_bench_last_signed_steps, 1000000, "backoff moves away from PF10 (positive with this config)");
    expect_u32(fake_x_stop_requests, 0U, "a PF10 stop is not treated as an abort");
    expect_u32(test_last_receive_timeout, 1U, "active homing polls the queue every millisecond");
}

static void test_home_fast_seek_aborts_on_end_limit(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    script_home_polls(1U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "PF9 stop does not start a backoff");
    expect_i64(fake_x_bench_last_signed_steps, -1000000, "fast seek moves toward PF10 (negative with this config)");
    expect_u32(fake_x_stop_requests, 1U, "PF9 stop aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "aborted homing stops polling");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "aborted homing leaves X unhomed"
    );
}

/* The three hardware states of a successful homing, applied on reads first..first+2:
 * fast seek stopped by PF10, backoff stopped by the PF10 release, approach stopped by PF10. */
static void script_successful_home_states(uint32_t first) {
    script_x_state(first, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_x_state(
        first + 1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
        0U
    );
    script_x_state(first + 2U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
}

/* Drives HOME through the fast seek and the backoff, then applies the given approach result. */
static void run_home_to_approach_result(
    MotionTaskContext_t *context,
    MotionAxisHardwareStopReason_t approach_reason,
    uint8_t start_limit_active
) {
    reset_receive_script();
    *context = valid_motion_context();
    context->machine_state.position_um[0] = 12345;
    script_successful_home_states(1U);
    script_x_state(3U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, approach_reason, start_limit_active);
    script_home_polls(3U);
    (void)run_motion_task_until_scripted_stop(context);
}

static const CommandDTO_t test_calibrate_x_command = {
    .sequence = 140U,
    .code = COMMAND_CODE_CALIBRATE,
    .parameters.calibrate.axes = MOTION_PLANNER_AXIS_X_MASK,
};

/* CALIBRATE, homing on reads 1..3, the measurement finishes on read 4, then `polls` more reads. */
static void run_calibration_with_final_home_polls(MotionTaskContext_t *context, uint32_t polls) {
    script_successful_home_states(1U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 4U + polls; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_calibration_runs_after_homing_and_applies_result(void) {
    MotionTaskContext_t context;
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 10U, .distance_um = 12U, .travel_um = 301000U};
    fake_encoder_x_um = -300000;
    context = valid_motion_context();
    context.machine_state.position_um[0] = 777;
    script_successful_home_states(5U);
    run_calibration_with_final_home_polls(&context, 3U);

    expect_u32(fake_calibration_begin_calls, 1U, "calibration starts once homing has finished");
    expect_u32(fake_calibration_poll_calls, 1U, "calibration is polled while it runs");
    expect_u32(fake_x_bench_starts, 6U, "calibration homes X before and after the measurement");
    expect_i64(fake_x_bench_last_signed_steps, -1000000, "the last move is the approach of the final homing");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "X is marked calibrated");
    expect_u32(context.calibration.axis[0].distance_um, 12U, "measured distance is kept in RAM");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 10U, "planner uses the measured steps");
    expect_u32(context.planner_config.axis[0].steps_per_um_denominator, 12U, "planner uses the measured distance");
    expect_u32(context.planner_config.axis[0].max_speed_um_per_s, 60000U, "speed limit follows from the step rate");
    expect_i64(context.machine_state.position_um[0], 0, "calibration ends with X at home");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished calibration stops polling");

    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, NULL);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_OK, "diagnostics report the calibration result");
    expect_u32(axis.calibrated, 1U, "diagnostics report X as calibrated");
    expect_u32(axis.calibration_steps, 10U, "diagnostics report the measured steps");
    expect_u32(axis.calibration_distance_um, 12U, "diagnostics report the measured distance");
    expect_u32(axis.travel_um, 301000U, "diagnostics report the travel");
    expect_u32((uint32_t)axis.max_position_um, 301000U, "diagnostics report the usable travel");
    expect_u32(axis.max_speed_um_per_s, 60000U, "diagnostics report the speed limit");
    expect_u32(axis.homed, 1U, "diagnostics report X as homed");
    expect_u32((uint32_t)axis.planned_position_um, 0U, "diagnostics report the planned position");
    expect_u32((uint32_t)axis.scale_position_um, (uint32_t)-300000, "diagnostics read the scale live");

    /* While the final homing runs the calibration is still busy and other X commands wait. */
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 10U, .distance_um = 12U, .travel_um = 301000U};
    context = valid_motion_context();
    script_x_state(5U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_calibration_with_final_home_polls(&context, 1U);
    expect_u32(test_last_receive_timeout, 1U, "calibration keeps polling during the final homing");
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(
        axis.calibration_result,
        AXIS_CALIBRATION_RUN_RESULT_RUNNING,
        "calibration is not done before X is home"
    );

    /* The final homing fails: the measurement is kept, X is left unhomed. */
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 10U, .distance_um = 12U, .travel_um = 301000U};
    context = valid_motion_context();
    script_x_state(5U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    run_calibration_with_final_home_polls(&context, 1U);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED, "failed final homing is reported");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "measurement survives it");
    expect_u32(axis.homed, 0U, "X is unhomed after the failed final homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed final homing ends the calibration");

    /* STOP during the final homing aborts the calibration as well. */
    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    context = valid_motion_context();
    script_successful_home_states(1U);
    script_x_state(5U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 4U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    const CommandDTO_t stop = {.sequence = 143U, .code = COMMAND_CODE_STOP};
    script_read(5U, osOK, &stop);
    /* Delay 1 is the settle pause of the first homing; the axis is at rest on the STOP wait that follows. */
    test_axis_settles_after_delays = 2U;
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_ABORTED, "STOP aborts the final homing");
    expect_u32(axis.homed, 0U, "aborted final homing leaves X unhomed");
}

static const CommandDTO_t test_calibrate_y_command = {
    .sequence = 141U,
    .code = COMMAND_CODE_CALIBRATE,
    .parameters.calibrate.axes = MOTION_PLANNER_AXIS_Y_MASK,
};

static const CommandDTO_t test_calibrate_z_command = {
    .sequence = 144U,
    .code = COMMAND_CODE_CALIBRATE,
    .parameters.calibrate.axes = MOTION_PLANNER_AXIS_Z_MASK,
};

/* CALIBRATE of `command`'s axis, homing on reads 1..3; the procedure starts on read 3. A final read 4 follows when
 * `final_command` is given or `final_status` is not osOK. */
static void run_calibration_until_start(
    MotionTaskContext_t *context,
    MotionAxis_t axis,
    const CommandDTO_t *command,
    osStatus_t final_status,
    const CommandDTO_t *final_command
) {
    reset_receive_script();
    *context = valid_motion_context();
    script_axis_state(axis, 1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_axis_state(
        axis,
        2U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
        0U
    );
    script_axis_state(axis, 3U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_read(0U, osOK, command);
    for (uint32_t index = 1U; index <= 3U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    if (final_command != NULL || final_status != osOK) {
        script_read(4U, final_status, final_command);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_calibration_moves_play_through_the_stream(void) {
    MotionTaskContext_t context;
    const MotionAxisHardwareConfig_t hardware = {0};
    const AxisBenchProfile_t profile = {1000U, 2000U, 2000U};

    run_calibration_until_start(&context, MOTION_AXIS_Z, &test_calibrate_z_command, osOK, NULL);
    expect_u32(fake_calibration_begin_calls, 1U, "Z calibration starts after its homing");
    const AxisCalibrationRunMover_t *z_mover = fake_calibration_last_mover;
    expect_u32(z_mover != NULL && z_mover != &axis_calibration_run_jog_mover, 1U, "Z moves by the planned stream");
    expect_u32(fake_calibration_last_axis, MOTION_AXIS_Z, "the procedure runs on Z");

    run_calibration_until_start(&context, MOTION_AXIS_Y, &test_calibrate_y_command, osOK, NULL);
    expect_u32(fake_calibration_begin_calls, 1U, "Y calibration starts after its homing");
    const AxisCalibrationRunMover_t *y_mover = fake_calibration_last_mover;
    expect_u32(y_mover != NULL && y_mover != &axis_calibration_run_jog_mover, 1U, "Y moves by the planned stream");
    expect_u32(z_mover == y_mover, 1U, "Z and Y share the mover of the planned stream");

    run_calibration_until_start(&context, MOTION_AXIS_X, &test_calibrate_x_command, osOK, NULL);
    const AxisCalibrationRunMover_t *mover = fake_calibration_last_mover;
    expect_u32(mover != NULL && mover != &axis_calibration_run_jog_mover, 1U, "X moves by the planned stream");
    expect_u32(mover == y_mover, 1U, "X and Y share the mover of the planned stream");
    if (mover == NULL || mover == &axis_calibration_run_jog_mover) {
        return;
    }

    /* The move of Y is its own block: the length and the rate belong to the axis that is asked to move. */
    fake_y.failing_start = 0U;
    expect_u32(mover->start(MOTION_AXIS_Y, &hardware, AXIS_BENCH_TOWARD_END, 777U, &profile), 1U, "Y move starts");
    expect_i64(fake_y.bench_last_signed_steps, 777, "the move of Y has the requested length");
    expect_u32(fake_y.bench_last_target_rate_hz, 2000U, "the move of Y runs at the target rate");
    /* The block goes to the queue of the generator instance of Y and to no other queue. */
    expect_u32(queued_blocks(&gen_y.queue), 1U, "the block of Y is in the queue of the Y instance");
    expect_u32(queued_blocks(&gen_x.queue) + queued_blocks(&gen_z.queue), 0U, "X and Z queues get no block of Y");
    expect_u32(queued_blocks(&test_plan_buffer), 0U, "the printing queue gets no block of Y");
    MotionPlanBlock_t queued_block = {0};
    expect_u32(plan_buffer_try_pop(&gen_y.queue, &queued_block), PLAN_BUFFER_STATUS_OK, "the Y block can be taken");
    expect_u32(queued_block.master_step_count, 777U, "the queued block of Y has the requested length");
    expect_u32(queued_block.moving_axes_mask, MOTION_PLANNER_AXIS_Y_MASK, "the queued block moves Y only");
    expect_u32(gen_y.busy, 1U, "the Y instance is busy with its block");
    expect_u32(gen_x.busy + gen_z.busy + gen_print.busy, 0U, "no other instance is busy with the block of Y");

    /* The same for Z: its own block, its own length and its own rate. */
    const uint32_t x_starts_before_z = fake_x_bench_starts;
    expect_u32(mover->start(MOTION_AXIS_Z, &hardware, AXIS_BENCH_TOWARD_START, 555U, &profile), 1U, "Z move starts");
    expect_i64(fake_z.bench_last_signed_steps, -555, "the move of Z has the requested length and direction");
    expect_u32(fake_z.bench_last_target_rate_hz, 2000U, "the move of Z runs at the target rate");
    expect_u32(fake_x_bench_starts, x_starts_before_z, "the move of Z does not move X");
    expect_u32(queued_blocks(&gen_z.queue), 1U, "the block of Z is in the queue of the Z instance");
    expect_u32(queued_blocks(&gen_x.queue), 0U, "the X queue gets no block of Z");
    expect_u32(gen_z.busy, 1U, "the Z instance is busy with its block");
    expect_u32(gen_x.busy, 0U, "the X instance is not busy with the block of Z");
    expect_u32(plan_buffer_try_pop(&gen_z.queue, &queued_block), PLAN_BUFFER_STATUS_OK, "the Z block can be taken");
    expect_u32(queued_block.master_step_count, 555U, "the queued block of Z has the requested length");
    expect_u32(queued_block.moving_axes_mask, MOTION_PLANNER_AXIS_Z_MASK, "the queued block moves Z only");

    fake_z.failing_start = fake_z.bench_starts + 1U;
    expect_u32(
        mover->start(MOTION_AXIS_Z, &hardware, AXIS_BENCH_TOWARD_END, 100U, &profile),
        0U,
        "no block of Z, no move"
    );
    fake_z.failing_start = 0U;

    /* A block that cannot be built does not start. */
    fake_x_failing_start = fake_x_bench_starts + 1U;
    expect_u32(
        mover->start(MOTION_AXIS_X, &hardware, AXIS_BENCH_TOWARD_END, 12345U, &profile),
        0U,
        "no block, no move"
    );
    fake_x_failing_start = 0U;

    const uint32_t notifications = test_block_ready_notifications;
    expect_u32(mover->start(MOTION_AXIS_X, &hardware, AXIS_BENCH_TOWARD_END, 12345U, &profile), 1U, "move starts");
    expect_i64(fake_x_bench_last_signed_steps, 12345, "the move has the requested length");
    expect_u32(fake_axes[MOTION_AXIS_X].bench_last_target_rate_hz, 2000U, "the move runs at the target rate");
    expect_u32(test_block_ready_notifications, notifications + 1U, "the generator is told about the block");
    expect_u32(queued_blocks(&gen_x.queue), 1U, "the block of X is in the queue of the X instance");
    expect_u32(plan_buffer_try_pop(&gen_x.queue, &queued_block), PLAN_BUFFER_STATUS_OK, "the X block can be taken");
    expect_u32(queued_block.master_step_count, 12345U, "the queued block of X has the requested length");
    expect_u32(queued_block.moving_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "the queued block moves X only");

    gen_x.busy = 1U;
    gen_x.halted = 0U;
    fake_x_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    expect_u32(mover->poll(MOTION_AXIS_X, 12345U), AXIS_CALIBRATION_RUN_MOVE_RUNNING, "playing block runs");

    fake_x_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    const uint32_t resumes_before_switch_stop = gen_x.resumes;
    expect_u32(mover->poll(MOTION_AXIS_X, 12345U), AXIS_CALIBRATION_RUN_MOVE_END_LIMIT, "end switch ends the move");
    expect_u32(gen_x.resumes, resumes_before_switch_stop + 1U, "a stop on a switch flushes the X instance");
    expect_u32(gen_y.resumes + gen_z.resumes + gen_print.resumes, 0U, "a stop of X flushes no other instance");

    gen_x.busy = 0U;
    fake_x_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_NONE;
    expect_u32(mover->poll(MOTION_AXIS_X, 12345U), AXIS_CALIBRATION_RUN_MOVE_COMPLETED, "played block completes");

    gen_x.halted = 1U;
    expect_u32(mover->poll(MOTION_AXIS_X, 12345U), AXIS_CALIBRATION_RUN_MOVE_FAILED, "halted generator fails");
    expect_u32(gen_x.halted, 0U, "a failed move leaves the generator reset");

    fake_x_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT;
    expect_u32(mover->poll(MOTION_AXIS_X, 12345U), AXIS_CALIBRATION_RUN_MOVE_FAILED, "start switch fails the move");

    fake_x_stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT;
    const uint32_t halts = gen_x.halts;
    const uint32_t resumes = gen_x.resumes;
    const uint32_t clears = fake_x_stop_reason_clears;
    mover->abort(MOTION_AXIS_X);
    expect_u32(gen_x.halts, halts + 1U, "abort stops the stream");
    expect_u32(gen_x.resumes, resumes, "abort leaves the reset of the generator to STOP");
    expect_u32(fake_x_stop_reason_clears, clears + 1U, "abort clears the stop reason once");
    expect_u32(fake_x_stop_reason, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, "abort does not leave the stop reason behind");
    expect_u32(gen_y.halts + gen_z.halts + gen_print.halts, 0U, "abort of X halts no other instance");
}

/* STOP and a queue failure while a calibration move runs: every axis plays through the stream, so each is flushed and
 * reset the same way. */
static void check_calibration_abort_flush(MotionAxis_t axis, const CommandDTO_t *calibrate) {
    MotionTaskContext_t context;
    const CommandDTO_t stop = {.sequence = 143U, .code = COMMAND_CODE_STOP};
    StepGeneratorInstance_t *const own = &fake_generators[axis];
    const uint32_t other_axis_count = (uint32_t)MOTION_AXIS_HARDWARE_COUNT - 1U;
    uint32_t other_halts = 0U;
    uint32_t other_resumes = 0U;

    /* The homing flushes its instance on every switch stop; that is the baseline of the runs below. */
    run_calibration_until_start(&context, axis, calibrate, osErrorTimeout, NULL);
    const uint32_t base_halts = own->halts;
    const uint32_t base_resumes = own->resumes;
    expect_u32(base_halts, 3U, "the homing of the calibration halts its instance on each of its three stops");

    run_calibration_until_start(&context, axis, calibrate, osOK, &stop);
    expect_u32(fake_calibration_abort_calls, 1U, "STOP aborts the calibration");
    expect_u32(own->halts, base_halts + 1U, "STOP halts the stream of the calibration move");
    expect_u32(own->resumes, base_resumes + 1U, "STOP resets the instance once, when the axes are at rest");
    for (uint32_t index = 0U; index < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++index) {
        if (index != (uint32_t)axis) {
            other_halts += fake_generators[index].halts;
            other_resumes += fake_generators[index].resumes;
        }
    }
    expect_u32(other_halts, 0U, "STOP of a calibration halts the instances of no other axis");
    /* STOP resets every instance once. */
    expect_u32(other_resumes, other_axis_count, "STOP resets each of the other axis instances once");
    expect_u32(gen_print.resumes, 1U, "STOP resets the printing instance once");

    run_calibration_until_start(&context, axis, calibrate, osErrorParameter, NULL);
    expect_u32(fake_calibration_abort_calls, 1U, "a queue failure aborts the calibration");
    expect_u32(own->halts, base_halts + 2U, "a queue failure halts the stream and flushes it");
    expect_u32(own->resumes, base_resumes + 1U, "a queue failure resets the instance, no STOP follows");
    other_halts = 0U;
    other_resumes = 0U;
    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        if (index != (uint32_t)axis) {
            other_halts += fake_generators[index].halts;
            other_resumes += fake_generators[index].resumes;
        }
    }
    expect_u32(other_halts, 0U, "a queue failure halts no other instance");
    expect_u32(other_resumes, 0U, "a queue failure resets no other instance");
}

/* Every axis plays through the instance of its own: it is flushed and reset the same way, the others are left alone. */
static void test_calibration_abort_flushes_the_stream(void) {
    check_calibration_abort_flush(MOTION_AXIS_X, &test_calibrate_x_command);
    check_calibration_abort_flush(MOTION_AXIS_Y, &test_calibrate_y_command);
    check_calibration_abort_flush(MOTION_AXIS_Z, &test_calibrate_z_command);
}

static void test_calibration_failures(void) {
    MotionTaskContext_t context;

    reset_receive_script();
    context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osErrorTimeout, NULL);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_calibration_begin_calls, 0U, "calibration does not start when homing fails");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED, "failed homing is reported");

    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    expect_u32(fake_calibration_begin_calls, 0U, "HOME alone does not calibrate");

    reset_receive_script();
    fake_calibration_begin_result = AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED;
    context = valid_motion_context();
    script_successful_home_states(1U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 3U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_MOVE_FAILED, "calibration that cannot start fails");
    expect_u32(context.calibration.calibrated_axes_mask, 0U, "failed calibration marks nothing");

    reset_receive_script();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE;
    context = valid_motion_context();
    script_successful_home_states(1U);
    script_read(0U, osOK, &test_calibrate_x_command);
    for (uint32_t index = 1U; index <= 4U; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NO_DISTANCE, "measurement failure is reported");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "failed calibration keeps the limits");
}

/* Both switches active at once can only be a wiring or sensor fault, so nothing may move. */
static void test_both_limits_active_refuse_homing_and_calibration(void) {
    MotionTaskContext_t context;
    MotionAxisDiagnostics axis = {0};

    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    fake_x_end_limit_active = 1U;
    script_read(0U, osOK, &test_calibrate_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_LIMITS_CONFLICT, "CALIBRATE reports the conflict");
    expect_u32(fake_x_bench_starts, 0U, "CALIBRATE with both switches active does not move");
    expect_u32(test_last_receive_timeout, osWaitForever, "refused calibration does not poll");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    fake_x_end_limit_active = 1U;
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "HOME with both switches active does not move");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "refused HOME leaves X unhomed"
    );

    /* Starting on PF10 alone is a normal start: the fast seek is blocked at once and the backoff follows. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "HOME on PF10 alone starts the seek");
}

static void test_calibration_is_cancelled_and_guarded(void) {
    MotionTaskContext_t context;
    const CommandDTO_t stop = {.sequence = 141U, .code = COMMAND_CODE_STOP};

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osOK, &test_calibrate_x_command);
    script_read(2U, osOK, &test_home_x_command);
    script_read(3U, osOK, &stop);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "CALIBRATE and HOME are ignored while calibration homes");
    expect_u32(fake_calibration_abort_calls, 1U, "STOP aborts the calibration");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_ABORTED, "aborted calibration is reported");

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osErrorParameter, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_calibration_abort_calls, 1U, "queue failure aborts the calibration");

    reset_receive_script();
    context = valid_motion_context();
    const CommandDTO_t calibrate_unknown = {
        .sequence = 142U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_ALL_AXES_MASK | 0x08U,
    };
    script_read(0U, osOK, &calibrate_unknown);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "a mask with an unknown axis is refused as a whole");
}

static void test_home_completes_only_on_pf10_during_approach(void) {
    MotionTaskContext_t context;
    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    expect_u32(fake_x_bench_starts, 3U, "fast seek, backoff and approach are started");
    expect_i64(fake_x_bench_last_signed_steps, -1000000, "the approach heads back toward PF10");
    expect_u32(test_delay_calls, 1U, "the approach waits once after the backoff");
    expect_u32(test_last_delay_ms, 10U, "the belt gets 10 ms to come to rest");
    expect_u32(
        fake_x_start_limit_hook == encoder_ox_reset_coordinate ? 1U : 0U,
        1U,
        "the approach lets the PF10 interrupt zero the scale"
    );
    expect_u32(fake_x_stop_requests, 0U, "a successful homing requests no stop");
    expect_u32(fake_x_ready_after_home_calls, 1U, "successful home readies the X hardware");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "successful homing marks X homed");
    expect_i64(context.machine_state.position_um[0], 0, "successful homing zeroes X");

    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 1U);
    expect_u32(fake_x_stop_requests, 1U, "approach ended by PF9 aborts homing");
    expect_u32(fake_x_start_limit_hook == NULL ? 1U : 0U, 1U, "aborted homing disarms the scale reset");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "approach ended by PF9 leaves X unhomed"
    );
    expect_i64(context.machine_state.position_um[0], 12345, "aborted homing keeps the old X position");
}

static void test_home_backoff_aborts_when_stopped_by_end_limit(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_x_state(2U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT, 0U);
    script_home_polls(2U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "backoff stopped by PF9 does not start the approach");
    expect_u32(fake_x_stop_requests, 1U, "backoff stopped by PF9 aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "aborted homing stops polling");
}

static void test_home_backoff_continues_while_pf10_flickers(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    /* The release interrupt fired, but PF10 is active again once the axis has stopped. */
    script_x_state(2U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE, 1U);
    script_home_polls(2U);

    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 3U, "a flickering PF10 restarts the backoff");
    expect_u32(fake_x_release_stop_arms, 2U, "the restarted backoff arms the release stop again");
    expect_i64(fake_x_bench_last_signed_steps, 1000000, "the restarted backoff keeps moving away from PF10");
    expect_u32(test_delay_calls, 0U, "no approach is started while PF10 is active");
    expect_u32(test_last_receive_timeout, 1U, "homing stays active");
}

static void test_stop_clears_runtime_fault_and_x_homing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);

    expect_u32(fake_x_clear_fault_calls, 1U, "STOP acknowledges an X runtime fault");
    expect_u32(fake_x_status, MOTION_AXIS_HARDWARE_STATUS_READY, "cleared fault returns X hardware to READY");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "X position is no longer trusted after a fault"
    );
}

static void test_stop_request_and_pipeline_reset(void) {
    reset_receive_script();
    motion_task_request_stop();
    expect_u32(test_generator_all_halts, 1U, "STOP halts every generator instance, not only one");
    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        expect_u32(fake_generators[index].halts, 1U, "STOP halts each instance once at once");
    }
    expect_u32(fake_x_stop_requests, 1U, "STOP stops the pulses at once");
    expect_u32(test_pending_receive_calls, 1U, "STOP reads the command queue until it is empty");
    expect_u32(test_result_count, 0U, "STOP with an empty command queue answers nothing");

    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[0] = 5000;
    fake_encoder_x_um = -1234;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_u32(test_generator_all_resumes, 1U, "STOP resets every generator instance, not only one");
    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        expect_u32(fake_generators[index].resumes, 1U, "STOP restarts each instance once the axes are at rest");
    }
    expect_i64(context.machine_state.position_um[0], 1234, "after STOP X continues from the scale reading");

    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    context.machine_state.position_um[0] = 5000;
    fake_encoder_x_um = 777;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_i64(context.machine_state.position_um[0], 5000, "unhomed X keeps its position after STOP");

    reset_receive_script();
    context = valid_motion_context();
    fake_encoder_x_um = 42;
    test_axis_settles_after_delays = 3U;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_RUNNING);
    expect_u32(test_delay_calls, 3U, "STOP waits for the axis to come to rest");
    expect_u32(test_generator_all_resumes, 1U, "generators restart after the axis settled");
    expect_i64(context.machine_state.position_um[0], 42, "positive scale reading is used as is");

    reset_receive_script();
    context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_RUNNING);
    expect_u32(test_delay_calls, MOTION_TASK_STOP_SETTLE_TIMEOUT_MS, "STOP gives up after the settle timeout");
    expect_u32(test_generator_all_resumes, 0U, "generators stay halted while the axis still moves");
}

static void test_stop_without_fault_keeps_homing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);

    expect_u32(fake_x_stop_requests, 1U, "STOP requests an X hardware stop");
    expect_u32(fake_x_clear_fault_calls, 0U, "STOP without a fault does not clear anything");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "homing survives a plain STOP");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(fake_axes[axis].timing_peak_resets, 1U, "STOP starts a new step-timing measurement on every axis");
    }

    reset_receive_script();
    context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_STOPPED);
    expect_u32(fake_x_clear_fault_calls, 0U, "an axis stopped by a switch is not a fault");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "homing survives a switch stop");
}

typedef enum { TEST_HOME_PHASE_FAST_SEEK = 0, TEST_HOME_PHASE_BACKOFF, TEST_HOME_PHASE_APPROACH } TestHomePhase_t;

typedef struct {
    const char *name;
    TestHomePhase_t phase;
    TestXHardwareState_t state;
    uint8_t expect_active;
    uint32_t expect_starts;
} TestHomePhaseCase_t;

/* Scripts the hardware states that walk HOME into `phase` (poll N moves it into phase N). */
static void script_home_prefix(TestHomePhase_t phase) {
    if (phase >= TEST_HOME_PHASE_BACKOFF) {
        script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    }
    if (phase >= TEST_HOME_PHASE_APPROACH) {
        script_x_state(
            2U,
            MOTION_AXIS_HARDWARE_STATUS_STOPPED,
            MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
            0U
        );
    }
}

#define TEST_X_STATE(status_, reason_, limit_)                                                                         \
    {.apply = 1U,                                                                                                      \
        .status = MOTION_AXIS_HARDWARE_STATUS_##status_,                                                               \
        .stop_reason = MOTION_AXIS_HARDWARE_STOP_REASON_##reason_,                                                     \
        .start_limit_active = (limit_)}

static const TestHomePhaseCase_t test_home_phase_cases[] = {
    {"fast seek fault aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(FAULT_TIMING, NONE, 0U), 0U, 1U},
    {"fast seek completing without PF10 aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(COMPLETE, NONE, 0U), 0U, 1U},
    {"fast seek with idle hardware aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(READY, NONE, 0U), 0U, 1U},
    {"fast seek stopped on request aborts", TEST_HOME_PHASE_FAST_SEEK, TEST_X_STATE(STOPPED, REQUEST, 0U), 0U, 1U},
    {"fast seek keeps waiting while STEP is pending",
        TEST_HOME_PHASE_FAST_SEEK,
        TEST_X_STATE(WAITING, NONE, 0U),
        1U,
        1U},

    {"backoff fault aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(FAULT_TIMING, NONE, 1U), 0U, 2U},
    {"backoff completing without the PF10 release aborts",
        TEST_HOME_PHASE_BACKOFF,
        TEST_X_STATE(COMPLETE, NONE, 1U),
        0U,
        2U},
    {"backoff with idle hardware aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(READY, NONE, 0U), 0U, 2U},
    {"backoff stopped on request aborts", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(STOPPED, REQUEST, 0U), 0U, 2U},
    {"backoff keeps running while PF10 is active", TEST_HOME_PHASE_BACKOFF, TEST_X_STATE(RUNNING, NONE, 1U), 1U, 2U},
    {"backoff stopped by the PF10 release starts the approach",
        TEST_HOME_PHASE_BACKOFF,
        TEST_X_STATE(STOPPED, START_LIMIT_RELEASE, 0U),
        1U,
        3U},

    {"approach PF10 stop with released switch aborts",
        TEST_HOME_PHASE_APPROACH,
        TEST_X_STATE(STOPPED, START_LIMIT, 0U),
        0U,
        3U},
    {"approach keeps running", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(RUNNING, NONE, 0U), 1U, 3U},
    {"approach fault aborts", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(FAULT_TIMING, NONE, 0U), 0U, 3U},
    {"approach completing without PF10 aborts", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(COMPLETE, NONE, 0U), 0U, 3U},
    {"approach with idle hardware aborts", TEST_HOME_PHASE_APPROACH, TEST_X_STATE(READY, NONE, 0U), 0U, 3U},
};

static void test_home_phase_outcomes(void) {
    for (size_t index = 0U; index < sizeof(test_home_phase_cases) / sizeof(test_home_phase_cases[0]); ++index) {
        const TestHomePhaseCase_t *test_case = &test_home_phase_cases[index];
        reset_receive_script();
        MotionTaskContext_t context = valid_motion_context();
        const uint32_t final_poll = (uint32_t)test_case->phase + 1U;
        script_home_prefix(test_case->phase);
        test_x_state_script[final_poll] = test_case->state;
        script_home_polls(final_poll);

        (void)run_motion_task_until_scripted_stop(&context);
        expect_u32(fake_x_bench_starts, test_case->expect_starts, test_case->name);
        expect_u32(test_last_receive_timeout, test_case->expect_active != 0U ? 1U : osWaitForever, test_case->name);
        expect_u32(
            context.machine_state.homed_axes_mask,
            MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
            "unfinished homing leaves X unhomed"
        );
    }
}

static void test_home_start_failures(void) {
    MotionTaskContext_t context;

    /* A seek that starts on a pressed PF10 is stopped by the stream like any other seek; that is the first
     * scenario of test_home_fast_seek_moves_toward_start_and_backs_off_on_pf10. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    script_home_polls(0U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_stop_requests, 1U, "failed seek start aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed seek start ends homing");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 2U;
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_STOPPED, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(2U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 2U, "backoff start is attempted");
    expect_u32(fake_x_stop_requests, 1U, "a backoff that cannot start aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "failed backoff start ends homing");

    /* The queue of the X instance refuses the block (here: it is not initialized): homing aborts. */
    reset_receive_script();
    (void)memset(&gen_x.queue, 0, sizeof(gen_x.queue));
    context = valid_motion_context();
    script_home_polls(0U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_stop_requests, 1U, "a refused block aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "a refused block ends homing");
}

/* The homing of X plays through the step stream; these are the cases that only the stream has. */
static void test_x_homing_through_the_stream(void) {
    MotionTaskContext_t context;

    /* A complete homing clears the old stop reason before each move and frees the generator after each stop. */
    run_home_to_approach_result(&context, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    expect_u32(
        fake_axes[MOTION_AXIS_X].stop_reason_clears,
        4U,
        "each homing move starts with no stop reason and the finished homing leaves none behind"
    );
    expect_u32(fake_axes[MOTION_AXIS_X].release_stop_disarms, 1U, "the release stop is disarmed before the approach");
    expect_u32(gen_x.halts, 3U, "each stop of the homing stream halts the generator of X");
    expect_u32(gen_x.resumes, 3U, "each stop of the homing stream resets the generator of X");
    expect_u32(test_generator_all_halts, 0U, "a homing stop halts the instance of the axis only");
    expect_u32(test_generator_all_resumes, 0U, "a homing stop resets the instance of the axis only");
    expect_u32(
        gen_y.halts + gen_z.halts + gen_print.halts + gen_y.resumes + gen_z.resumes + gen_print.resumes,
        0U,
        "the homing of X halts and resets no other instance"
    );
    expect_u32(enqueued_blocks(&gen_x.queue), 3U, "the three homing moves of X go into the queue of the X instance");
    expect_u32(
        enqueued_blocks(&gen_y.queue) + enqueued_blocks(&gen_z.queue) + enqueued_blocks(&test_plan_buffer),
        0U,
        "no other queue gets a block of the homing of X"
    );
    expect_u32(queued_blocks(&gen_x.queue), 0U, "each stop of the homing flushes the queue of X");
    expect_u32(test_block_ready_notifications, 3U, "each homing move is one block for the generator");
    expect_u32(fake_axes[MOTION_AXIS_X].bench_last_target_rate_hz, 100U, "the approach plays at the approach rate");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "X is homed");

    /* The generator resets only after a few waits: homing goes on with the backoff. */
    reset_receive_script();
    context = valid_motion_context();
    test_generator_reset_keeps_busy = 1U;
    test_generator_idle_after_delays = 3U;
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(1U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(test_delay_calls, 3U, "the stop waits for the generator reset");
    expect_u32(fake_x_bench_starts, 2U, "a reset that takes a few waits does not stop the homing");
    expect_u32(test_last_receive_timeout, 1U, "homing goes on after the wait");

    /* The generator never resets: the homing is aborted. */
    reset_receive_script();
    context = valid_motion_context();
    test_generator_reset_keeps_busy = 1U;
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT, 1U);
    script_home_polls(1U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(test_delay_calls, 2U * MOTION_TASK_STOP_SETTLE_TIMEOUT_MS, "the wait for the reset is bounded");
    expect_u32(fake_x_bench_starts, 1U, "no backoff starts without a reset generator");
    expect_u32(test_last_receive_timeout, osWaitForever, "a generator that does not reset ends the homing");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_X_MASK,
        "X stays unhomed"
    );

    /* The generator halts itself without a switch (a fault): the homing is aborted and the generator is reset. */
    reset_receive_script();
    context = valid_motion_context();
    gen_x.halted = 1U;
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_home_polls(1U);
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "a fault of the generator does not start a backoff");
    expect_u32(gen_x.resumes, 1U, "an aborted homing resets the generator");
    expect_u32(gen_x.halted, 0U, "the reset leaves the generator running");
    expect_u32(test_last_receive_timeout, osWaitForever, "a fault of the generator ends the homing");
}

static const CommandDTO_t test_relative_move_command = {
    .sequence = 140U,
    .code = COMMAND_CODE_MOTION_OPERATION,
    .parameters.motion_operation =
        {
            .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE | COMMAND_MOTION_OPERATION_FLAG_RELATIVE,
            .x = 2000,
            .speed = 1000U,
        },
};

static const CommandDTO_t test_xyz_move_command = {
    .sequence = 141U,
    .code = COMMAND_CODE_MOTION_OPERATION,
    .parameters.motion_operation =
        {
            .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
            .x = 1000,
            .speed = 1000U,
        },
};

static const CommandDTO_t test_stop_command = {
    .sequence = 142U,
    .code = COMMAND_CODE_STOP,
};

/* HOME, then `command` arrives while the fast seek is running. */
static void run_command_during_home(MotionTaskContext_t *context, const CommandDTO_t *command) {
    reset_receive_script();
    *context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, command);
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_commands_during_home(void) {
    MotionTaskContext_t context;

    run_command_during_home(&context, &test_home_x_command);
    expect_u32(fake_x_bench_starts, 1U, "repeated HOME does not restart an active homing");
    expect_u32(test_last_receive_timeout, 1U, "repeated HOME keeps homing active");

    run_command_during_home(&context, &test_relative_move_command);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "a relative move is ignored while homing");
    expect_u32(test_last_receive_timeout, 1U, "a relative move does not interrupt homing");

    run_command_during_home(&context, &test_xyz_move_command);
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "XYZ move is not planned while homing");

    run_command_during_home(&context, &test_stop_command);
    expect_u32(fake_x_stop_requests, 1U, "STOP aborts active homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "STOP ends homing");
    expect_u32(test_generator_all_resumes, 0U, "STOP leaves the generators halted until every axis is at rest");
    expect_u32(gen_x.resumes, 0U, "STOP does not reset the X instance while X is not at rest");
    expect_u32(fake_x_stop_reason_clears, 2U, "STOP clears the stop reason once more after the one of the move start");

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osErrorResource, NULL);
    test_jump_on_delay = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_stop_requests, 1U, "queue failure aborts active homing");
}

static void test_home_is_refused_when_it_cannot_start(void) {
    MotionTaskContext_t context;

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_x_state(0U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "HOME is ignored while the axis is moving");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "ignored HOME keeps X homed");

    reset_receive_script();
    script_read(0U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(NULL);
    expect_u32(fake_x_bench_starts, 0U, "HOME without a task context is ignored");
}

static void test_home_and_calibrate_wait_for_the_planned_stream(void) {
    MotionTaskContext_t context;
    const CommandDTO_t home_all = {
        .sequence = 170U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_ALL_AXES_MASK,
    };
    const CommandDTO_t calibrate_all = {
        .sequence = 171U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_ALL_AXES_MASK,
    };
    const CommandDTO_t home_xy = {
        .sequence = 172U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
    };
    const CommandDTO_t calibrate_xz = {
        .sequence = 173U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
    };

    /* The printing stream holds the timers of all three axes: every HOME and CALIBRATE is refused, whatever it names. */
    const CommandDTO_t *const refused_commands[] = {
        &test_home_x_command,
        &test_home_y_command,
        &test_home_z_command,
        &test_calibrate_x_command,
        &test_calibrate_y_command,
        &test_calibrate_z_command,
        &home_all,
        &calibrate_all,
    };
    for (uint32_t index = 0U; index < sizeof(refused_commands) / sizeof(refused_commands[0]); ++index) {
        reset_receive_script();
        context = valid_motion_context();
        gen_print.busy = 1U;
        script_read(0U, osOK, refused_commands[index]);
        test_block_when_empty = 1U;
        (void)run_motion_task_until_scripted_stop(&context);
        expect_u32(
            fake_x_bench_starts + fake_y.bench_starts + fake_z.bench_starts,
            0U,
            "no axis starts while the printing stream is busy"
        );
        expect_u32(fake_calibration_begin_calls, 0U, "no calibration starts while the printing stream is busy");
        expect_u32(test_block_ready_notifications, 0U, "a refused command queues no block");
        expect_u32(
            enqueued_blocks(&gen_x.queue) + enqueued_blocks(&gen_y.queue) + enqueued_blocks(&gen_z.queue),
            0U,
            "a refused command leaves the axis queues empty"
        );
        expect_u32(
            context.machine_state.homed_axes_mask,
            MOTION_PLANNER_ALL_AXES_MASK,
            "a refused command leaves the axes homed"
        );
        expect_u32(test_last_receive_timeout, osWaitForever, "a refused command leaves nothing to poll");
        expect_u32(gen_print.busy, 1U, "the printing stream is left running");
    }

    /* Only the X instance is busy: X is refused, also as part of a command, and Y and Z start. */
    reset_receive_script();
    context = valid_motion_context();
    gen_x.busy = 1U;
    script_read(0U, osOK, &home_xy);
    script_read(1U, osOK, &calibrate_xz);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts + fake_y.bench_starts + fake_z.bench_starts, 0U, "a command naming busy X is refused");
    expect_u32(fake_calibration_begin_calls, 0U, "CALIBRATE naming busy X starts no calibration");
    expect_u32(test_block_ready_notifications, 0U, "the refused commands queue no block");

    reset_receive_script();
    context = valid_motion_context();
    gen_x.busy = 1U;
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &test_calibrate_x_command);
    script_read(2U, osOK, &test_home_y_command);
    script_read(3U, osOK, &test_calibrate_z_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 0U, "HOME and CALIBRATE of X are refused while the X instance is busy");
    expect_u32(fake_y.bench_starts, 1U, "HOME of Y starts although the X instance is busy");
    expect_u32(fake_z.bench_starts, 1U, "CALIBRATE of Z starts although the X instance is busy");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Z, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_RUNNING, "the calibration of Z runs");
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NONE, "the refused calibration of X never began");
    expect_u32(enqueued_blocks(&gen_x.queue), 0U, "nothing is queued for the busy X instance");
    expect_u32(enqueued_blocks(&gen_y.queue), 1U, "the seek of Y is in the queue of the Y instance");
    expect_u32(enqueued_blocks(&gen_z.queue), 1U, "the seek of Z is in the queue of the Z instance");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_AXIS_X_MASK,
        "the axes that started are unhomed, the refused one keeps its homing"
    );
}

static void test_step_timing_comes_from_the_axis_config(void) {
    uint32_t pulse_ns = 1U;
    uint32_t low_ns = 2U;
    motion_task_get_step_timing(MOTION_AXIS_X, &pulse_ns, &low_ns);
    expect_u32(pulse_ns, 5000U, "X STEP pulse width");
    expect_u32(low_ns, MOTION_AXIS_HARDWARE_DM556_MIN_STEP_LOW_NS, "X STEP low time");

    pulse_ns = 1U;
    low_ns = 2U;
    motion_task_get_step_timing(MOTION_AXIS_HARDWARE_COUNT, &pulse_ns, &low_ns);
    motion_task_get_step_timing(MOTION_AXIS_X, NULL, &low_ns);
    motion_task_get_step_timing(MOTION_AXIS_X, &pulse_ns, NULL);
    expect_u32(pulse_ns, 1U, "unknown axis leaves the pulse width");
    expect_u32(low_ns, 2U, "unknown axis leaves the low time");
}

static void test_limit_deceleration_comes_from_the_axis_config(void) {
    expect_u32(motion_task_get_limit_deceleration(MOTION_AXIS_X), 600000U, "X limit deceleration");
    expect_u32(motion_task_get_limit_deceleration(MOTION_AXIS_HARDWARE_COUNT), 0U, "unknown axis has none");
}

static void run_relative_move(MotionTaskContext_t *context, int32_t delta_x_um) {
    CommandDTO_t move = test_relative_move_command;
    move.parameters.motion_operation.x = delta_x_um;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_relative_move_is_planned_from_the_current_position(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[0] = 1000;
    context.machine_state.position_um[1] = 500;
    run_relative_move(&context, 2000);
    MotionPlannerResult_t result = motion_task_test_get_last_planner_result();
    expect_u32(result.status, MOTION_PLANNER_STATUS_OK, "a relative move is planned like any other move");
    expect_u32(result.block.sequence, 140U, "a relative move keeps its sequence");
    expect_i64(result.block.step_delta[0], 160, "a relative move moves X by its increment");
    expect_i64(result.block.step_delta[1], 0, "a zero increment leaves Y in place");
    expect_u32(result.block.feedrate_um_per_s, 1000U, "a relative move uses its speed");
    expect_i64(context.machine_state.position_um[0], 3000, "the next move starts at the relative target");
    expect_i64(context.machine_state.position_um[1], 500, "Y position is unchanged");
    expect_u32(test_block_ready_notifications, 1U, "step generator is woken for the relative move");
    expect_u32(fake_x_bench_starts, 0U, "a relative move does not drive the axis directly");

    reset_receive_script();
    context = valid_motion_context();
    run_relative_move(&context, -200000);
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_POSITION_OUT_OF_RANGE,
        "a relative move beyond the travel is rejected"
    );
    expect_i64(context.machine_state.position_um[0], 0, "a rejected relative move keeps the position");
    expect_u32(test_block_ready_notifications, 0U, "a rejected relative move moves nothing");

    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    run_relative_move(&context, 2000);
    expect_u32(motion_task_get_last_planner_status(), MOTION_PLANNER_STATUS_NOT_HOMED, "a relative move needs a homed X");

    /* Targets beyond int32 are clamped instead of wrapping around. */
    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.position_um[0] = INT32_MAX;
    run_relative_move(&context, 1);
    expect_i64(motion_task_test_get_last_planner_result().block.step_delta[0], 0, "positive overflow is clamped");
    expect_i64(context.machine_state.position_um[0], INT32_MAX, "a clamped move leaves X at the int32 limit");

    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.position_um[0] = INT32_MIN;
    run_relative_move(&context, -1);
    expect_i64(context.machine_state.position_um[0], INT32_MIN, "negative overflow is clamped");

    reset_receive_script();
    run_relative_move(NULL, 2000);
    expect_u32(
        motion_task_get_last_planner_status(),
        MOTION_PLANNER_STATUS_INVALID_ARGUMENT,
        "a relative move without a task context is rejected by the planner"
    );
}

static void test_relative_move_applies_increments_to_every_axis(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[0] = 1000;
    context.machine_state.position_um[1] = 2000;
    context.machine_state.position_um[2] = 300;
    CommandDTO_t move = test_relative_move_command;
    move.parameters.motion_operation.x = -500;
    move.parameters.motion_operation.y = 1000;
    move.parameters.motion_operation.z = -300;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    const MotionPlannerResult_t result = motion_task_test_get_last_planner_result();
    expect_u32(result.status, MOTION_PLANNER_STATUS_OK, "a relative XYZ move is planned");
    /* 80 steps per 1000 um on every axis. */
    expect_i64(result.block.step_delta[0], -40, "X moves back by its increment");
    expect_i64(result.block.step_delta[1], 80, "Y moves forward by its increment");
    expect_i64(result.block.step_delta[2], -24, "Z moves back by its increment");
    expect_i64(context.machine_state.position_um[0], 500, "X ends at position plus increment");
    expect_i64(context.machine_state.position_um[1], 3000, "Y ends at position plus increment");
    expect_i64(context.machine_state.position_um[2], 0, "Z ends at position plus increment");
}

static void test_stop_fault_acknowledgement_edges(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_FAULT_CONFIG);
    expect_u32(fake_x_clear_fault_calls, 1U, "STOP tries to clear a configuration fault");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK,
        "uncleared configuration fault leaves homing untouched"
    );

    reset_receive_script();
    run_single_stop(NULL, MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING);
    expect_u32(fake_x_clear_fault_calls, 1U, "STOP clears a runtime fault without a task context");
    expect_u32(fake_x_status, MOTION_AXIS_HARDWARE_STATUS_READY, "fault is cleared without a task context");
}

static void test_stored_calibration_sets_planner_limits(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const MotionPlannerAxisConfig_t default_y = context.planner_config.axis[1];
    test_calibration_load_status = CALIBRATION_STORE_STATUS_OK;
    test_stored_calibration.calibrated_axes_mask = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK;
    test_stored_calibration.axis[0] =
        (AxisCalibration_t){.steps = 100000U, .distance_um = 50000U, .travel_um = 301000U};
    /* Z is marked but its travel does not fit the planner, so it keeps the default limits. */
    test_stored_calibration.axis[2] =
        (AxisCalibration_t){.steps = 1U, .distance_um = 1U, .travel_um = (uint32_t)INT32_MAX + 1U};
    const MotionPlannerAxisConfig_t default_z = context.planner_config.axis[2];
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 100000U, "X steps come from the calibration");
    expect_u32(
        context.planner_config.axis[0].steps_per_um_denominator,
        50000U,
        "X micrometres come from the calibration"
    );
    expect_u32(
        (uint32_t)context.planner_config.axis[0].max_position_um,
        301000U,
        "X travel comes from the calibration"
    );
    expect_u32(context.planner_config.axis[0].max_speed_um_per_s, 25000U, "X speed follows from the step-rate limit");
    expect_u32(
        context.planner_config.axis[1].steps_per_um_numerator,
        default_y.steps_per_um_numerator,
        "uncalibrated Y keeps its defaults"
    );
    expect_u32(
        context.planner_config.axis[2].steps_per_um_numerator,
        default_z.steps_per_um_numerator,
        "unusable Z calibration is ignored"
    );

    reset_receive_script();
    context = valid_motion_context();
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "missing calibration keeps defaults");
}

/* The three hardware states of a successful homing of `axis`, applied on reads first..first+2. */
static void script_axis_home_success(MotionAxis_t axis, uint32_t first) {
    script_axis_state(
        axis,
        first,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
        1U
    );
    script_axis_state(
        axis,
        first + 1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT_RELEASE,
        0U
    );
    script_axis_state(
        axis,
        first + 2U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_START_LIMIT,
        1U
    );
}

/* `command` on read 0, then `polls` poll timeouts. */
static void run_command_with_polls(MotionTaskContext_t *context, const CommandDTO_t *command, uint32_t polls) {
    script_read(0U, osOK, command);
    for (uint32_t index = 1U; index <= polls; ++index) {
        script_read(index, osErrorTimeout, NULL);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

/* Z homes through the step stream like X and Y, with the screw rates of its own, its own switches and its own scale. */
static void test_z_homing_through_the_stream(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[2] = 4321;
    context.machine_state.position_um[0] = 777;
    script_axis_home_success(MOTION_AXIS_Z, 1U);
    run_command_with_polls(&context, &test_home_z_command, 3U);

    expect_u32(fake_z.init_calls, 1U, "Z hardware is initialized at task start");
    expect_u32(fake_z.bench_starts, 3U, "Z plays fast seek, backoff and approach as three blocks");
    expect_u32(fake_x_bench_starts + fake_y.bench_starts, 0U, "HOME Z does not move X or Y");
    expect_u32(fake_z.release_stop_arms, 1U, "the Z backoff arms the Z release stop");
    expect_u32(fake_z.release_stop_disarms, 1U, "the Z release stop is disarmed before the approach");
    expect_i64(fake_z.bench_last_signed_steps, -1000000, "the Z approach heads toward the Z start switch");
    expect_u32(fake_z.bench_last_target_rate_hz, 4800U, "the Z approach plays at the screw approach rate");
    expect_u32(
        fake_z.start_limit_hook == encoder_oz_reset_coordinate ? 1U : 0U,
        1U,
        "the Z approach zeroes the Z scale"
    );
    expect_u32(fake_z.ready_after_home_calls, 1U, "successful homing readies the Z hardware");
    expect_u32(fake_z.stop_reason_clears, 4U, "each Z move starts with no stop reason and none is left behind");
    expect_u32(test_block_ready_notifications, 3U, "each homing move of Z is one block for the generator");
    expect_u32(gen_z.halts, 3U, "each stop of the Z stream halts the generator of Z");
    expect_u32(gen_z.resumes, 3U, "each stop of the Z stream resets the generator of Z");
    expect_u32(enqueued_blocks(&gen_z.queue), 3U, "the three homing moves of Z go into the queue of the Z instance");
    expect_u32(
        gen_x.halts + gen_y.halts + gen_print.halts + enqueued_blocks(&gen_x.queue) + enqueued_blocks(&gen_y.queue),
        0U,
        "the homing of Z touches no other instance or queue"
    );
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "Z is homed");
    expect_i64(context.machine_state.position_um[2], 0, "Z is at zero");
    expect_i64(context.machine_state.position_um[0], 777, "X keeps its position");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished homing stops polling");

    /* The fast seek runs at the Z seek rate, not at the belt rate. */
    reset_receive_script();
    context = valid_motion_context();
    run_command_with_polls(&context, &test_home_z_command, 0U);
    expect_u32(fake_z.bench_last_target_rate_hz, 80000U, "the Z fast seek plays at 80 kHz");
    expect_i64(fake_z.bench_last_signed_steps, -1000000, "the Z fast seek heads toward the Z start switch");
    expect_u32(test_last_receive_timeout, 1U, "homing stays active while the seek plays");

    /* A failed Z homing leaves only Z unhomed. */
    reset_receive_script();
    context = valid_motion_context();
    script_axis_state(
        MOTION_AXIS_Z,
        1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
        0U
    );
    run_command_with_polls(&context, &test_home_z_command, 1U);
    expect_u32(fake_z.stop_requests, 1U, "the end switch aborts the Z homing");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_Z_MASK,
        "aborted Z homing leaves only Z unhomed"
    );

    /* A block that cannot be built aborts the Z homing. */
    reset_receive_script();
    context = valid_motion_context();
    fake_z.failing_start = 1U;
    run_command_with_polls(&context, &test_home_z_command, 0U);
    expect_u32(fake_z.stop_requests, 1U, "a Z seek that cannot be built aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "a Z seek that cannot be built ends homing");

    /* A generator that halts itself without a switch aborts the Z homing and is reset. */
    reset_receive_script();
    context = valid_motion_context();
    gen_z.halted = 1U;
    script_axis_state(MOTION_AXIS_Z, 1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_command_with_polls(&context, &test_home_z_command, 1U);
    expect_u32(fake_z.bench_starts, 1U, "a fault of the generator does not start a Z backoff");
    expect_u32(gen_z.resumes, 1U, "an aborted Z homing resets the generator");
    expect_u32(gen_x.resumes + gen_y.resumes, 0U, "an aborted Z homing resets no other instance");
    expect_u32(test_last_receive_timeout, osWaitForever, "a fault of the generator ends the Z homing");

    /* A Z that moves is not homed again, the same as X. */
    reset_receive_script();
    context = valid_motion_context();
    script_axis_state(MOTION_AXIS_Z, 0U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_command_with_polls(&context, &test_home_z_command, 0U);
    expect_u32(fake_z.bench_starts, 0U, "HOME is ignored while Z is moving");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "ignored HOME keeps Z homed");
}

/* Y homes through the step stream like X, with its own block, its own switches and its own scale. */
static void test_y_homing_through_the_stream(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.position_um[1] = 4321;
    script_axis_home_success(MOTION_AXIS_Y, 1U);
    run_command_with_polls(&context, &test_home_y_command, 3U);

    expect_u32(fake_axes[MOTION_AXIS_Y].init_calls, 1U, "Y hardware is initialized at task start");
    expect_u32(fake_y.bench_starts, 3U, "Y plays fast seek, backoff and approach as three blocks");
    expect_u32(fake_x_bench_starts, 0U, "HOME Y does not move X");
    expect_u32(fake_y.release_stop_arms, 1U, "the Y backoff arms the Y release stop");
    expect_u32(fake_y.release_stop_disarms, 1U, "the Y release stop is disarmed before the approach");
    expect_i64(fake_y.bench_last_signed_steps, -1000000, "the Y approach heads toward the Y start switch");
    expect_u32(fake_y.bench_last_target_rate_hz, 100U, "the Y approach plays at the belt approach rate");
    expect_u32(
        fake_y.start_limit_hook == encoder_oy_reset_coordinate ? 1U : 0U,
        1U,
        "the Y approach zeroes the Y scale"
    );
    expect_u32(fake_y.ready_after_home_calls, 1U, "successful homing readies the Y hardware");
    expect_u32(fake_y.stop_reason_clears, 4U, "each Y move starts with no stop reason and none is left behind");
    expect_u32(test_block_ready_notifications, 3U, "each homing move of Y is one block for the generator");
    expect_u32(gen_y.halts, 3U, "each stop of the Y stream halts the generator of Y");
    expect_u32(gen_y.resumes, 3U, "each stop of the Y stream resets the generator of Y");
    expect_u32(enqueued_blocks(&gen_y.queue), 3U, "the three homing moves of Y go into the queue of the Y instance");
    expect_u32(
        gen_x.halts + gen_z.halts + gen_print.halts + enqueued_blocks(&gen_x.queue) + enqueued_blocks(&gen_z.queue),
        0U,
        "the homing of Y touches no other instance or queue"
    );
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "Y is homed");
    expect_i64(context.machine_state.position_um[1], 0, "Y is at zero");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished homing stops polling");

    /* A failed Y homing leaves only Y unhomed. */
    reset_receive_script();
    context = valid_motion_context();
    script_axis_state(
        MOTION_AXIS_Y,
        1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
        0U
    );
    run_command_with_polls(&context, &test_home_y_command, 1U);
    expect_u32(fake_y.stop_requests, 1U, "the end switch aborts the Y homing");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_Y_MASK,
        "aborted Y homing leaves only Y unhomed"
    );

    /* A block that cannot be built aborts the Y homing. */
    reset_receive_script();
    context = valid_motion_context();
    fake_y.failing_start = 1U;
    run_command_with_polls(&context, &test_home_y_command, 0U);
    expect_u32(fake_y.stop_requests, 1U, "a Y seek that cannot be built aborts homing");
    expect_u32(test_last_receive_timeout, osWaitForever, "a Y seek that cannot be built ends homing");

    /* A generator that halts itself without a switch aborts the Y homing and is reset. */
    reset_receive_script();
    context = valid_motion_context();
    gen_y.halted = 1U;
    script_axis_state(MOTION_AXIS_Y, 1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_command_with_polls(&context, &test_home_y_command, 1U);
    expect_u32(fake_y.bench_starts, 1U, "a fault of the generator does not start a Y backoff");
    expect_u32(gen_y.resumes, 1U, "an aborted Y homing resets the generator");
    expect_u32(gen_x.resumes + gen_z.resumes, 0U, "an aborted Y homing resets no other instance");
    expect_u32(test_last_receive_timeout, osWaitForever, "a fault of the generator ends the Y homing");
}

static CommandDTO_t home_command_for(uint8_t axes) {
    CommandDTO_t command = {.sequence = 180U, .code = COMMAND_CODE_HOME};
    command.parameters.home.axes = axes;
    return command;
}

static CommandDTO_t calibrate_command_for(uint8_t axes) {
    CommandDTO_t command = {.sequence = 181U, .code = COMMAND_CODE_CALIBRATE};
    command.parameters.calibrate.axes = axes;
    return command;
}

static uint8_t test_axis_bit(uint32_t axis) { return (uint8_t)(1U << axis); }

/* Target rates of the first homing move (the fast seek) and of the last one (the approach), from the axis settings:
 * belts X and Y 50 kHz and 100 Hz, the screw Z 80 kHz and 4.8 kHz. */
static const uint32_t test_seek_rate_hz[MOTION_AXIS_HARDWARE_COUNT] = {50000U, 50000U, 80000U};
static const uint32_t test_approach_rate_hz[MOTION_AXIS_HARDWARE_COUNT] = {100U, 100U, 4800U};

static MotionAxisHardwareStartLimitHook_t test_scale_zero_hook(uint32_t axis) {
    return axis == (uint32_t)MOTION_AXIS_X   ? encoder_ox_reset_coordinate
           : axis == (uint32_t)MOTION_AXIS_Y ? encoder_oy_reset_coordinate
                                             : encoder_oz_reset_coordinate;
}

/* One command may name any set of the axes; each of them starts at once on its own instance, the others stay put. */
static void test_axes_of_one_command_start_together(void) {
    const uint8_t masks[] = {
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
        MOTION_PLANNER_AXIS_Y_MASK | MOTION_PLANNER_AXIS_Z_MASK,
        MOTION_PLANNER_ALL_AXES_MASK,
    };
    for (uint32_t index = 0U; index < sizeof(masks) / sizeof(masks[0]); ++index) {
        for (uint32_t calibrate = 0U; calibrate < 2U; ++calibrate) {
            reset_receive_script();
            MotionTaskContext_t context = valid_motion_context();
            const CommandDTO_t command =
                calibrate != 0U ? calibrate_command_for(masks[index]) : home_command_for(masks[index]);
            run_command_with_polls(&context, &command, 0U);

            uint32_t started = 0U;
            for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
                const uint32_t named = (masks[index] & test_axis_bit(axis)) != 0U ? 1U : 0U;
                started += named;
                expect_u32(fake_axes[axis].bench_starts, named, "an axis named by the command starts its seek");
                expect_u32(
                    enqueued_blocks(&fake_generators[axis].queue),
                    named,
                    "the seek goes into the queue of the instance of its own axis"
                );
                expect_u32(fake_generators[axis].busy, named, "the instance of a named axis is busy with its seek");
                expect_i64(
                    fake_axes[axis].bench_last_signed_steps,
                    named != 0U ? -1000000 : 0,
                    "the seek heads toward the start switch of its axis"
                );
                expect_u32(
                    fake_axes[axis].bench_last_target_rate_hz,
                    named != 0U ? test_seek_rate_hz[axis] : 0U,
                    "the seek runs at the seek rate of its axis"
                );
                expect_u32(
                    context.machine_state.homed_axes_mask & test_axis_bit(axis),
                    named != 0U ? 0U : test_axis_bit(axis),
                    "a named axis is unhomed, an axis that is not named keeps its homing"
                );
                if (named != 0U) {
                    MotionPlanBlock_t block = {0};
                    expect_u32(
                        plan_buffer_try_pop(&fake_generators[axis].queue, &block),
                        PLAN_BUFFER_STATUS_OK,
                        "the seek block can be taken from its queue"
                    );
                    expect_u32(block.moving_axes_mask, test_axis_bit(axis), "the seek block moves its axis only");
                    expect_u32(block.master_step_count, 1000000U, "the seek is not ended by a step count");
                }
            }
            expect_u32(test_block_ready_notifications, started, "the generator is woken once for each seek");
            expect_u32(enqueued_blocks(&test_plan_buffer), 0U, "nothing goes into the printing queue");
            expect_u32(gen_print.busy, 0U, "the printing instance is not busy with a homing");
            expect_u32(test_last_receive_timeout, 1U, "the started homings are polled every millisecond");
            expect_u32(fake_calibration_begin_calls, 0U, "a calibration does not begin before its homing ends");
        }
    }
}

/* HOME of X, Y and Z in one command: every axis goes through its three moves on its own instance and its own queue. */
static void test_home_of_three_axes_runs_in_parallel(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    context.machine_state.position_um[0] = 111;
    context.machine_state.position_um[1] = 222;
    context.machine_state.position_um[2] = 333;
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        script_axis_home_success((MotionAxis_t)axis, 1U);
    }
    const CommandDTO_t home_all = home_command_for(MOTION_PLANNER_ALL_AXES_MASK);
    run_command_with_polls(&context, &home_all, 3U);

    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        const FakeAxis_t *fake = &fake_axes[axis];
        const StepGeneratorInstance_t *generator = &fake_generators[axis];
        expect_u32(fake->bench_starts, 3U, "each axis plays fast seek, backoff and approach");
        expect_u32(fake->release_stop_arms, 1U, "the backoff of each axis arms its release stop");
        expect_u32(fake->release_stop_disarms, 1U, "the release stop of each axis is disarmed before its approach");
        expect_u32(fake->ready_after_home_calls, 1U, "each axis is readied after its own homing");
        expect_u32(fake->stop_requests, 0U, "a successful homing requests no stop");
        expect_u32(fake->stop_reason_clears, 4U, "each move of each axis starts with no stop reason");
        expect_i64(fake->bench_last_signed_steps, -1000000, "the approach of each axis heads toward its start switch");
        expect_u32(fake->bench_last_target_rate_hz, test_approach_rate_hz[axis], "each axis approaches at its own rate");
        expect_u32(
            fake->start_limit_hook == test_scale_zero_hook(axis) ? 1U : 0U,
            1U,
            "the approach of each axis zeroes the scale of that axis"
        );
        expect_u32(generator->halts, 3U, "each stop of an axis halts its own instance");
        expect_u32(generator->resumes, 3U, "each stop of an axis resets its own instance");
        expect_u32(enqueued_blocks(&fake_generators[axis].queue), 3U, "three moves went into the queue of each axis");
        expect_u32(queued_blocks(&fake_generators[axis].queue), 0U, "each stop flushed the queue of its axis");
        expect_i64(context.machine_state.position_um[axis], 0, "each axis is at zero");
    }
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "all three axes are homed");
    expect_u32(test_block_ready_notifications, 9U, "the generator is woken for each of the nine homing moves");
    expect_u32(test_generator_all_halts + test_generator_all_resumes, 0U, "a homing stop never halts every instance");
    expect_u32(gen_print.halts + gen_print.resumes, 0U, "the printing instance is not touched by the homing");
    expect_u32(enqueued_blocks(&test_plan_buffer), 0U, "the printing queue gets no block");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished homings stop polling");
}

/* A switch or a failure of one axis ends only its own homing: the other axes go on and each flushes only its instance. */
static void test_failure_of_one_axis_does_not_end_the_homing_of_the_others(void) {
    const CommandDTO_t home_all = home_command_for(MOTION_PLANNER_ALL_AXES_MASK);

    /* Right after the failure: Y is over, X and Z are in their backoff. */
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    script_axis_home_success(MOTION_AXIS_X, 1U);
    script_axis_home_success(MOTION_AXIS_Z, 1U);
    script_axis_state(
        MOTION_AXIS_Y,
        1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
        0U
    );
    run_command_with_polls(&context, &home_all, 1U);
    expect_u32(fake_y.stop_requests, 1U, "the end switch aborts the homing of Y");
    expect_u32(fake_y.bench_starts, 1U, "Y does not start a backoff");
    expect_u32(gen_y.halts, 2U, "Y flushes its own instance on the switch stop and again on the abort");
    expect_u32(gen_y.resumes, 2U, "Y resets its own instance twice");
    expect_u32(queued_blocks(&gen_y.queue), 0U, "the queue of Y is empty");
    expect_u32(fake_x_bench_starts, 2U, "X goes on to its backoff");
    expect_u32(fake_z.bench_starts, 2U, "Z goes on to its backoff");
    expect_u32(fake_x_stop_requests + fake_z.stop_requests, 0U, "X and Z are not stopped by the failure of Y");
    expect_u32(gen_x.halts, 1U, "X flushes only its own instance for its own stop");
    expect_u32(gen_z.halts, 1U, "Z flushes only its own instance for its own stop");
    expect_u32(queued_blocks(&gen_x.queue), 1U, "the backoff of X is queued");
    expect_u32(queued_blocks(&gen_z.queue), 1U, "the backoff of Z is queued");
    expect_u32(fake_x_release_stop_arms, 1U, "the backoff of X arms the release stop of X");
    expect_u32(fake_z.release_stop_arms, 1U, "the backoff of Z arms the release stop of Z");
    expect_u32(fake_y.release_stop_arms, 0U, "Y has no backoff, so no release stop");
    expect_u32(test_generator_all_halts, 0U, "no instance is halted on behalf of the others");
    expect_u32(test_last_receive_timeout, 1U, "X and Z keep being polled");

    /* The rest of the run: X and Z finish their homing, Y stays unhomed. */
    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    script_axis_home_success(MOTION_AXIS_X, 1U);
    script_axis_home_success(MOTION_AXIS_Z, 1U);
    script_axis_state(
        MOTION_AXIS_Y,
        1U,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STOP_REASON_END_LIMIT,
        0U
    );
    run_command_with_polls(&context, &home_all, 3U);
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
        "X and Z are homed, Y is not"
    );
    expect_u32(fake_x_bench_starts, 3U, "X plays all three moves");
    expect_u32(fake_z.bench_starts, 3U, "Z plays all three moves");
    expect_u32(fake_y.bench_starts, 1U, "Y played only its seek");
    expect_u32(fake_x_stop_requests + fake_z.stop_requests, 0U, "X and Z are never stopped");
    expect_u32(fake_y.stop_requests, 1U, "Y was stopped once");
    expect_u32(fake_y.release_stop_disarms, 1U, "the cancel of the Y homing disarms its release stop");
    expect_u32(
        fake_x_start_limit_hook == encoder_ox_reset_coordinate ? 1U : 0U,
        1U,
        "X zeroes its own scale at its approach"
    );
    expect_u32(fake_z.start_limit_hook == encoder_oz_reset_coordinate ? 1U : 0U, 1U, "Z zeroes its own scale");
    expect_u32(gen_x.halts, 3U, "X flushed its own instance at each of its three stops");
    expect_u32(gen_z.halts, 3U, "Z flushed its own instance at each of its three stops");
    expect_u32(gen_y.halts, 2U, "Y flushed its own instance only for its own failure");
    expect_u32(test_last_receive_timeout, osWaitForever, "nothing is left to poll once the others have finished");
}

/* CALIBRATE of two axes in one command: both home, measure and home again at the same time, each on its own lane. */
static void test_calibrate_of_two_axes_runs_in_parallel(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 50000U, .distance_um = 25000U, .travel_um = 50300U};
    const MotionAxis_t axes[] = {MOTION_AXIS_X, MOTION_AXIS_Z};
    for (uint32_t index = 0U; index < sizeof(axes) / sizeof(axes[0]); ++index) {
        script_axis_home_success(axes[index], 1U);
        script_axis_home_success(axes[index], 5U);
    }
    const CommandDTO_t calibrate = calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK);
    run_command_with_polls(&context, &calibrate, 7U);

    expect_u32(fake_calibration_begin_calls, 2U, "the procedure begins once for each axis");
    expect_u32(fake_calibration_poll_calls, 2U, "the procedure is polled once for each axis");
    expect_u32(fake_x_bench_starts, 6U, "X is homed before and after its measurement");
    expect_u32(fake_z.bench_starts, 6U, "Z is homed before and after its measurement");
    expect_u32(fake_y.bench_starts, 0U, "Y is not named, so it does not move");
    expect_u32(gen_x.halts, 6U, "X flushed its own instance at each of its six stops");
    expect_u32(gen_z.halts, 6U, "Z flushed its own instance at each of its six stops");
    expect_u32(gen_y.halts + gen_y.resumes + enqueued_blocks(&gen_y.queue), 0U, "the instance of Y is not touched");
    expect_u32(
        context.calibration.calibrated_axes_mask,
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
        "X and Z are marked calibrated"
    );
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
        "X and Z are homed again at the end"
    );
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_OK, "X reports its calibration as done");
    motion_task_get_axis_diagnostics(MOTION_AXIS_Z, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_OK, "Z reports its calibration as done");
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NONE, "Y has no calibration result");
    expect_u32(test_last_receive_timeout, osWaitForever, "finished calibrations stop polling");
}

static void push_pending_print_block_at_read_2(uint32_t read_index) {
    if (read_index == 2U) {
        const MotionPlanBlock_t pending = {.sequence = 77U};
        (void)plan_buffer_try_push(&test_plan_buffer, &pending);
    }
}

/* STOP while the three axes home together: every axis is stopped and unhomed, the printing queue and the queue of each
 * axis are emptied and every instance is reset once. */
static void test_stop_during_the_homing_of_three_axes(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t home_all = home_command_for(MOTION_PLANNER_ALL_AXES_MASK);
    const CommandDTO_t stop = {.sequence = 182U, .code = COMMAND_CODE_STOP};
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        script_axis_state(
            (MotionAxis_t)axis,
            1U,
            MOTION_AXIS_HARDWARE_STATUS_RUNNING,
            MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
            0U
        );
        script_axis_state(
            (MotionAxis_t)axis,
            2U,
            MOTION_AXIS_HARDWARE_STATUS_STOPPED,
            MOTION_AXIS_HARDWARE_STOP_REASON_REQUEST,
            0U
        );
    }
    script_read(0U, osOK, &home_all);
    script_read(1U, osErrorTimeout, NULL);
    script_read(2U, osOK, &stop);
    test_block_when_empty = 1U;
    /* A block that reached the printing queue just before STOP. */
    test_receive_hook = push_pending_print_block_at_read_2;

    /* A command that was accepted before STOP and still waits in the queue. */
    test_pending_commands[0] = (CommandDTO_t){.sequence = 190U, .code = COMMAND_CODE_SET_OUTPUT};
    test_pending_count = 1U;

    /* The first half of STOP, which runs in the task that received it. */
    motion_task_request_stop();
    expect_u32(test_generator_all_halts, 1U, "the immediate STOP halts every instance");
    expect_u32(test_result_count, 0U, "the immediate STOP answers nothing");
    expect_u32(test_pending_position, 1U, "the immediate STOP took the command behind it out of the queue");
    (void)run_motion_task_until_scripted_stop(&context);

    for (uint32_t index = 0U; index < TEST_GENERATOR_COUNT; ++index) {
        expect_u32(fake_generators[index].halts, 1U, "each instance was halted once, by the immediate STOP only");
        expect_u32(fake_generators[index].resumes, 1U, "each instance is reset once, when the axes are at rest");
    }
    expect_u32(test_generator_all_resumes, 1U, "the reset is the reset of every instance");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        expect_u32(fake_axes[axis].bench_starts, 1U, "no axis started anything after its seek");
        expect_u32(fake_axes[axis].stop_requests, 2U, "each axis is stopped by the immediate STOP and by its homing cancel");
        expect_u32(queued_blocks(&fake_generators[axis].queue), 0U, "STOP emptied the queue of each axis");
        expect_u32(cancelled_blocks(&fake_generators[axis].queue), 1U, "STOP cancelled the seek queued for each axis");
        expect_u32(fake_axes[axis].timing_peak_resets, 1U, "STOP starts a new timing measurement on each axis");
    }
    expect_u32(queued_blocks(&test_plan_buffer), 0U, "STOP emptied the printing queue");
    expect_u32(cancelled_blocks(&test_plan_buffer), 1U, "STOP cancelled the block of the printing queue");
    expect_u32(context.machine_state.homed_axes_mask, 0U, "no axis is homed after the cancelled homing");
    expect_u32(test_discarded_results, 1U, "STOP erased the answer of HOME, which the client had not read");
    expect_u32(test_result_count, 1U, "only the answer of STOP is left");
    expect_u32(test_results[0].sequence, 182U, "STOP is answered by its sequence");
    expect_u32(test_results[0].response_type, COMMAND_MESSAGE_TYPE_DONE, "STOP settled with the axes at rest");
    expect_u32(test_results[0].code, COMMAND_DONE_ACCEPTED, "STOP is accepted");
    expect_u32(test_last_receive_timeout, osWaitForever, "no homing is left to poll");
}

/* HOME of Y is accepted while HOME of X runs: it does not depend on the instance of X. */
static void test_home_of_y_is_accepted_while_x_homes(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &test_home_y_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    expect_u32(fake_x_bench_starts, 1U, "X is not restarted by the command for Y");
    expect_u32(fake_y.bench_starts, 1U, "Y starts its homing while X homes");
    expect_u32(fake_z.bench_starts, 0U, "Z is not named, so it does not move");
    expect_u32(queued_blocks(&gen_x.queue), 1U, "the seek of X is still queued for X");
    expect_u32(queued_blocks(&gen_y.queue), 1U, "the seek of Y is queued for Y");
    MotionPlanBlock_t block = {0};
    expect_u32(plan_buffer_try_pop(&gen_y.queue, &block), PLAN_BUFFER_STATUS_OK, "the Y seek can be taken");
    expect_u32(block.moving_axes_mask, MOTION_PLANNER_AXIS_Y_MASK, "the block of Y moves Y");
    expect_u32(plan_buffer_try_pop(&gen_x.queue, &block), PLAN_BUFFER_STATUS_OK, "the X seek can be taken");
    expect_u32(block.moving_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "the block of X moves X");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_AXIS_Z_MASK,
        "X and Y are unhomed, Z keeps its homing"
    );
    expect_u32(test_last_receive_timeout, 1U, "both homings are polled");
}

/* An XYZ move is not planned while an axis homes or calibrates, whichever axis it is. */
static void test_xyz_move_is_refused_while_any_axis_homes(void) {
    const CommandDTO_t *const homes[MOTION_AXIS_HARDWARE_COUNT] = {
        &test_home_x_command,
        &test_home_y_command,
        &test_home_z_command,
    };
    const CommandDTO_t *const calibrations[MOTION_AXIS_HARDWARE_COUNT] = {
        &test_calibrate_x_command,
        &test_calibrate_y_command,
        &test_calibrate_z_command,
    };
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        for (uint32_t calibrate = 0U; calibrate < 2U; ++calibrate) {
            reset_receive_script();
            MotionTaskContext_t context = valid_motion_context();
            script_read(0U, osOK, calibrate != 0U ? calibrations[axis] : homes[axis]);
            script_read(1U, osOK, &test_xyz_move_command);
            test_block_when_empty = 1U;
            (void)run_motion_task_until_scripted_stop(&context);
            expect_u32(fake_axes[axis].bench_starts, 1U, "the homing of the axis is running");
            expect_u32(motion_task_test_get_planner_call_count(), 0U, "the XYZ move is not planned while an axis homes");
            expect_u32(enqueued_blocks(&test_plan_buffer), 0U, "the XYZ move is not queued while an axis homes");
            expect_u32(test_block_ready_notifications, 1U, "only the seek of the homing woke the generator");
            expect_i64(context.machine_state.position_um[0], 0, "the refused move leaves the planned position");
        }
    }
}

static void release_print_stream_at_read_1(uint32_t read_index) {
    if (read_index == 1U) {
        gen_print.busy = 0U;
    }
}

/* A printing stream that has blocks to play holds all three axes: HOME is refused while it is busy, and the same HOME
 * starts once the stream has ended. */
static void test_home_starts_once_the_printing_stream_is_idle(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    gen_print.busy = 1U;
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    test_receive_hook = release_print_stream_at_read_1;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "HOME of X is refused while printing and starts once the stream has ended");
    expect_u32(enqueued_blocks(&gen_x.queue), 1U, "the seek of X is queued once");
    expect_u32(test_block_ready_notifications, 1U, "the refused HOME woke nothing");
}

static void test_home_and_calibrate_masks(void) {
    MotionTaskContext_t context;
    const CommandDTO_t home_none = {.sequence = 152U, .code = COMMAND_CODE_HOME, .parameters.home.axes = 0U};
    const CommandDTO_t calibrate_none = {
        .sequence = 153U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = 0U,
    };
    const CommandDTO_t home_xy = {
        .sequence = 154U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
    };

    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &home_none);
    script_read(1U, osOK, &calibrate_none);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts + fake_y.bench_starts, 0U, "HOME and CALIBRATE without axes move nothing");

    /* Every axis has its instance: while X homes, Y starts, also in a moment when the instance of X is idle, e.g.
     * between two homing moves. The state of X at read 1 makes the instance of X idle. */
    reset_receive_script();
    context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_READY, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &test_home_y_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "X starts homing");
    expect_u32(fake_y.bench_starts, 1U, "Y starts homing while X homes");
    expect_u32(enqueued_blocks(&gen_y.queue), 1U, "the seek of Y is queued for Y");

    /* The same holds for a calibration: Y starts while X calibrates. */
    reset_receive_script();
    context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_READY, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, &test_calibrate_x_command);
    script_read(1U, osOK, &test_calibrate_y_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "X starts its calibration homing");
    expect_u32(fake_y.bench_starts, 1U, "Y starts calibrating while X calibrates");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_RUNNING, "the calibration of Y runs");

    /* Z starts while X homes... */
    reset_receive_script();
    context = valid_motion_context();
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_READY, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &test_home_z_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "X starts homing");
    expect_u32(fake_z.bench_starts, 1U, "Z starts homing while X homes");

    /* ...and the other way round: X starts while Z calibrates. */
    reset_receive_script();
    context = valid_motion_context();
    script_axis_state(MOTION_AXIS_Z, 1U, MOTION_AXIS_HARDWARE_STATUS_READY, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, &test_calibrate_z_command);
    script_read(1U, osOK, &test_home_x_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_z.bench_starts, 1U, "Z starts its calibration homing");
    expect_u32(fake_x_bench_starts, 1U, "X starts homing while Z calibrates");

    /* A command naming a busy axis is refused as a whole. */
    reset_receive_script();
    context = valid_motion_context();
    script_read(0U, osOK, &test_home_x_command);
    script_read(1U, osOK, &home_xy);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_x_bench_starts, 1U, "X is not restarted");
    expect_u32(fake_y.bench_starts, 0U, "Y does not start with a refused command");
    expect_u32(enqueued_blocks(&gen_y.queue), 0U, "nothing is queued for Y by a refused command");
    expect_u32(enqueued_blocks(&gen_x.queue), 1U, "X queues no second seek");

    /* The same for a calibration that names the axis that is calibrating, and for a busy axis in the middle of the
     * mask: Z calibrates, so HOME of X and Z is refused whole and X does not start either. */
    reset_receive_script();
    context = valid_motion_context();
    const CommandDTO_t home_xz = {
        .sequence = 155U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK,
    };
    const CommandDTO_t calibrate_xy = {
        .sequence = 156U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK,
    };
    script_read(0U, osOK, &test_calibrate_z_command);
    script_read(1U, osOK, &home_xz);
    script_read(2U, osOK, &test_calibrate_x_command);
    script_read(3U, osOK, &calibrate_xy);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(fake_z.bench_starts, 1U, "Z starts its calibration homing");
    expect_u32(fake_x_bench_starts, 1U, "X starts only with the command that does not name Z");
    expect_u32(fake_y.bench_starts, 0U, "CALIBRATE of X and Y is refused as a whole while X calibrates");
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NONE, "the refused calibration of Y never began");
}

static void test_y_calibration_reports_on_y(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 50000U, .distance_um = 23300U, .travel_um = 401000U};
    fake_y.encoder_um = 1234;
    script_axis_home_success(MOTION_AXIS_Y, 1U);
    script_axis_home_success(MOTION_AXIS_Y, 5U);
    const CommandDTO_t calibrate_y = {
        .sequence = 155U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_AXIS_Y_MASK,
    };
    run_command_with_polls(&context, &calibrate_y, 7U);

    expect_u32(fake_calibration_last_axis, MOTION_AXIS_Y, "the procedure runs on Y");
    expect_u32(
        fake_calibration_last_mover != NULL && fake_calibration_last_mover != &axis_calibration_run_jog_mover,
        1U,
        "the procedure moves Y through the planned stream"
    );
    expect_u32(fake_x_bench_starts, 0U, "CALIBRATE Y does not move X");
    expect_u32(fake_y.bench_starts, 6U, "Y is homed before and after the measurement");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_Y_MASK, "Y is marked calibrated");
    expect_u32(context.calibration.axis[1].distance_um, 23300U, "the Y measurement is kept for Y");
    expect_u32(context.planner_config.axis[1].steps_per_um_numerator, 50000U, "Y limits follow the measurement");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "X limits are untouched");

    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, &axis);
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_OK, "Y diagnostics report the result");
    expect_u32(axis.calibrated, 1U, "Y diagnostics report Y as calibrated");
    expect_u32(axis.travel_um, 401000U, "Y diagnostics report the Y travel");
    expect_u32(axis.homed, 1U, "Y diagnostics report Y as homed");
    expect_u32((uint32_t)axis.scale_position_um, 1234U, "Y diagnostics read the Y scale");
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(axis.calibrated, 0U, "X stays uncalibrated");
    expect_u32(axis.calibration_result, AXIS_CALIBRATION_RUN_RESULT_NONE, "X has no calibration result");
}

static void test_z_home_and_calibration(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    const CommandDTO_t home_all = {
        .sequence = 156U,
        .code = COMMAND_CODE_HOME,
        .parameters.home.axes = MOTION_PLANNER_ALL_AXES_MASK,
    };
    /* Every axis has its own instance, so a command for all of them starts all of them. */
    run_command_with_polls(&context, &home_all, 0U);
    expect_u32(fake_x_bench_starts, 1U, "HOME of all axes starts the seek of X");
    expect_u32(fake_y.bench_starts, 1U, "HOME of all axes starts the seek of Y");
    expect_u32(fake_z.bench_starts, 1U, "HOME of all axes starts the seek of Z");
    expect_u32(fake_z.bench_last_target_rate_hz, 80000U, "the Z seek of the combined command runs at the Z seek rate");
    expect_u32(test_last_receive_timeout, 1U, "the three homings are polled");
    expect_u32(context.machine_state.homed_axes_mask, 0U, "no axis is homed before its homing has ended");

    reset_receive_script();
    context = valid_motion_context();
    context.planner_config.max_step_event_rate_hz = 150000U;
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 50000U, .distance_um = 25000U, .travel_um = 50300U};
    fake_z.encoder_um = -42;
    script_axis_home_success(MOTION_AXIS_Z, 1U);
    script_axis_home_success(MOTION_AXIS_Z, 5U);
    const CommandDTO_t calibrate_z = {
        .sequence = 157U,
        .code = COMMAND_CODE_CALIBRATE,
        .parameters.calibrate.axes = MOTION_PLANNER_AXIS_Z_MASK,
    };
    run_command_with_polls(&context, &calibrate_z, 7U);
    expect_u32(fake_calibration_last_axis, MOTION_AXIS_Z, "the procedure runs on Z");
    expect_u32(fake_calibration_last_settings->measure_steps, 233600U, "Z measures 12 mm of its screw");
    expect_u32(fake_calibration_last_settings->seek.target_step_rate_hz, 80000U, "Z seeks its end at 80 kHz");
    expect_u32(fake_calibration_last_settings->seek.acceleration_steps_per_s2, 320000U, "Z ramps up its seek");
    expect_u32(
        fake_calibration_last_mover != NULL && fake_calibration_last_mover != &axis_calibration_run_jog_mover,
        1U,
        "the procedure moves Z through the planned stream"
    );
    expect_u32(fake_z.bench_last_target_rate_hz, 4800U, "Z approaches its home switch slowly");
    expect_u32(fake_z.bench_starts, 6U, "Z is homed before and after the measurement");
    expect_u32(fake_x_bench_starts + fake_y.bench_starts, 0U, "CALIBRATE Z does not move X or Y");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_Z_MASK, "Z is marked calibrated");
    expect_u32(context.calibration.axis[2].distance_um, 25000U, "the Z measurement is kept for Z");
    expect_u32(context.planner_config.axis[0].steps_per_um_numerator, 80U, "X limits are untouched");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Z, &axis);
    expect_u32(axis.travel_um, 50300U, "Z diagnostics report the Z travel");
    expect_u32(axis.max_speed_um_per_s, 40000U, "Z speed follows from its own 80 kHz motor limit");
    expect_u32((uint32_t)axis.scale_position_um, (uint32_t)-42, "Z diagnostics read the Z scale");
}

static void test_axis_diagnostics_edges(void) {
    MotionAxisDiagnostics axis = {.travel_um = 77U};
    motion_task_get_axis_diagnostics(MOTION_AXIS_Y, NULL);
    motion_task_get_axis_diagnostics(MOTION_AXIS_HARDWARE_COUNT, &axis);
    expect_u32(axis.travel_um, 77U, "an unknown axis leaves the result untouched");
}

static void test_planned_move_of_z_is_queued(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    CommandDTO_t move = test_xyz_move_command;
    move.parameters.motion_operation.x = 0;
    move.parameters.motion_operation.z = 1000;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    const MotionPlannerResult_t result = motion_task_test_get_last_planner_result();
    expect_u32(result.status, MOTION_PLANNER_STATUS_OK, "a homed Z is planned like X and Y");
    /* 1000 um at 80 steps per 1000 um. */
    expect_i64(result.block.step_delta[2], 80, "the move of Z has the steps of Z");
    expect_i64(result.block.step_delta[0], 0, "the move of Z leaves X in place");
    expect_i64(result.block.step_delta[1], 0, "the move of Z leaves Y in place");
    expect_u32(result.block.moving_axes_mask, MOTION_PLANNER_AXIS_Z_MASK, "only Z moves");
    expect_u32(test_block_ready_notifications, 1U, "the generator is told about the Z block");
    expect_i64(context.machine_state.position_um[2], 1000, "the next move starts at the Z target");
    expect_i64(context.machine_state.position_um[0], 0, "X keeps its position");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "planning unhomes nothing");
    PlanBufferDiagnostics_t plan = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan);
    expect_u32(plan.queued, 1U, "the Z block is in the plan buffer");

    /* An XYZ move that includes Z is one block too. */
    reset_receive_script();
    context = valid_motion_context();
    move.parameters.motion_operation.x = 1000;
    move.parameters.motion_operation.y = 500;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    const MotionPlannerResult_t xyz = motion_task_test_get_last_planner_result();
    expect_u32(xyz.status, MOTION_PLANNER_STATUS_OK, "a move of all three axes is planned");
    expect_u32(xyz.block.moving_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "X, Y and Z all move");
    expect_i64(xyz.block.step_delta[2], 80, "the Z steps are in the XYZ block");
    expect_i64(context.machine_state.position_um[2], 1000, "the next move starts at the Z target of the XYZ block");
    expect_u32(test_block_ready_notifications, 1U, "an XYZ block is one wakeup of the generator");

    /* An unhomed Z is refused and nothing is queued. */
    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK;
    move.parameters.motion_operation.x = 0;
    move.parameters.motion_operation.y = 0;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(motion_task_get_last_planner_status(), MOTION_PLANNER_STATUS_NOT_HOMED, "an unhomed Z is refused");
    expect_u32(test_block_ready_notifications, 0U, "nothing is queued for an unhomed Z");
    expect_i64(context.machine_state.position_um[2], 0, "a refused move keeps the Z position");
}

static void test_planned_move_of_y_is_queued(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    CommandDTO_t move = test_xyz_move_command;
    move.parameters.motion_operation.x = 0;
    move.parameters.motion_operation.y = 1000;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    const MotionPlannerResult_t result = motion_task_test_get_last_planner_result();
    expect_u32(result.status, MOTION_PLANNER_STATUS_OK, "a homed Y is planned like X");
    /* 1000 um at 80 steps per 1000 um. */
    expect_i64(result.block.step_delta[1], 80, "the move of Y has the steps of Y");
    expect_i64(result.block.step_delta[0], 0, "the move of Y leaves X in place");
    expect_u32(result.block.moving_axes_mask, MOTION_PLANNER_AXIS_Y_MASK, "only Y moves");
    expect_u32(test_block_ready_notifications, 1U, "the generator is told about the Y block");
    expect_i64(context.machine_state.position_um[1], 1000, "the next move starts at the Y target");
    expect_i64(context.machine_state.position_um[0], 0, "X keeps its position");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "planning unhomes nothing");

    /* An unhomed Y is refused. */
    reset_receive_script();
    context = valid_motion_context();
    context.machine_state.homed_axes_mask = MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Z_MASK;
    script_read(0U, osOK, &move);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);
    expect_u32(motion_task_get_last_planner_status(), MOTION_PLANNER_STATUS_NOT_HOMED, "an unhomed Y is refused");
    expect_u32(test_block_ready_notifications, 0U, "nothing is queued for an unhomed Y");
}

static void test_stop_reaches_every_axis(void) {
    reset_receive_script();
    motion_task_request_stop();
    expect_u32(fake_y.stop_requests, 1U, "STOP stops the Y pulses at once");
    expect_u32(fake_z.stop_requests, 1U, "STOP stops the Z pulses at once");

    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    script_axis_state(
        MOTION_AXIS_Y,
        0U,
        MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING,
        MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
        0U
    );
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_u32(fake_y.clear_fault_calls, 1U, "STOP acknowledges a Y fault");
    expect_u32(fake_x_clear_fault_calls, 0U, "X without a fault is left alone");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_Y_MASK,
        "only Y loses its homing"
    );

    reset_receive_script();
    context = valid_motion_context();
    script_axis_state(
        MOTION_AXIS_Z,
        0U,
        MOTION_AXIS_HARDWARE_STATUS_FAULT_TIMING,
        MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
        0U
    );
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_u32(fake_z.clear_fault_calls, 1U, "STOP acknowledges a Z fault");
    expect_u32(fake_y.clear_fault_calls, 0U, "Y without a fault is left alone");
    expect_u32(
        context.machine_state.homed_axes_mask,
        MOTION_PLANNER_ALL_AXES_MASK & (uint8_t)~MOTION_PLANNER_AXIS_Z_MASK,
        "only Z loses its homing"
    );

    reset_receive_script();
    context = valid_motion_context();
    fake_y.encoder_um = -2500;
    fake_z.encoder_um = -4000;
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_i64(context.machine_state.position_um[1], 2500, "after STOP Y continues from its scale reading");
    expect_i64(context.machine_state.position_um[2], 4000, "after STOP Z continues from its scale reading");
}

static const MotionAxisHardwareStatus_t test_active_statuses[] = {
    MOTION_AXIS_HARDWARE_STATUS_STARTING,
    MOTION_AXIS_HARDWARE_STATUS_WAITING,
    MOTION_AXIS_HARDWARE_STATUS_RUNNING,
};
#define TEST_ACTIVE_STATUS_COUNT (sizeof(test_active_statuses) / sizeof(test_active_statuses[0]))

/* HOME and CALIBRATE of an axis whose hardware is STARTING, WAITING or RUNNING do not start, whichever of the three
 * statuses it is and whichever axis it is. The stream is idle, so only the hardware status refuses. */
static void test_home_and_calibrate_are_refused_while_the_hardware_is_active(void) {
    const CommandDTO_t *const home_commands[MOTION_AXIS_HARDWARE_COUNT] = {
        &test_home_x_command,
        &test_home_y_command,
        &test_home_z_command,
    };
    const CommandDTO_t *const calibrate_commands[MOTION_AXIS_HARDWARE_COUNT] = {
        &test_calibrate_x_command,
        &test_calibrate_y_command,
        &test_calibrate_z_command,
    };
    const uint8_t axis_masks[MOTION_AXIS_HARDWARE_COUNT] = {
        MOTION_PLANNER_AXIS_X_MASK,
        MOTION_PLANNER_AXIS_Y_MASK,
        MOTION_PLANNER_AXIS_Z_MASK,
    };

    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        for (uint32_t index = 0U; index < TEST_ACTIVE_STATUS_COUNT; ++index) {
            /* HOME: the homed state, the queued block and the idle polling are untouched. */
            reset_receive_script();
            test_receive_keeps_generator_idle = 1U;
            MotionTaskContext_t context = valid_motion_context();
            const MotionPlanBlock_t pending = {.sequence = 90U};
            expect_u32(plan_buffer_try_push(&test_plan_buffer, &pending), PLAN_BUFFER_STATUS_OK, "a block is queued before HOME");
            script_axis_state(
                (MotionAxis_t)axis,
                0U,
                test_active_statuses[index],
                MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
                0U
            );
            script_read(0U, osOK, home_commands[axis]);
            test_block_when_empty = 1U;
            (void)run_motion_task_until_scripted_stop(&context);
            expect_u32(
                fake_axes[axis].bench_starts,
                0U,
                "HOME does not start while the hardware is STARTING, WAITING or RUNNING"
            );
            expect_u32(
                context.machine_state.homed_axes_mask,
                MOTION_PLANNER_ALL_AXES_MASK,
                "a refused HOME leaves the axis homed"
            );
            PlanBufferDiagnostics_t plan = {0};
            (void)plan_buffer_get_diagnostics(&test_plan_buffer, &plan);
            expect_u32(plan.queued, 1U, "a refused HOME does not clear the plan buffer");
            expect_u32(test_block_ready_notifications, 0U, "a refused HOME queues no block");
            expect_u32(test_last_receive_timeout, osWaitForever, "a refused HOME leaves no homing to poll");
            expect_u32(fake_axes[axis].stop_requests, 0U, "a refused HOME is not an abort");
            expect_u32(test_axis_bench_calls, 0U, "a refused HOME does not use the ramped driver start");

            /* CALIBRATE: the axis is active, so the command is refused as a whole and the calibration never begins. */
            reset_receive_script();
            test_receive_keeps_generator_idle = 1U;
            context = valid_motion_context();
            script_axis_state(
                (MotionAxis_t)axis,
                0U,
                test_active_statuses[index],
                MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
                0U
            );
            script_read(0U, osOK, calibrate_commands[axis]);
            test_block_when_empty = 1U;
            (void)run_motion_task_until_scripted_stop(&context);
            MotionAxisDiagnostics diagnostics = {0};
            motion_task_get_axis_diagnostics((MotionAxis_t)axis, &diagnostics);
            expect_u32(
                diagnostics.calibration_result,
                AXIS_CALIBRATION_RUN_RESULT_NONE,
                "a refused CALIBRATE of an active axis starts no calibration"
            );
            expect_u32(diagnostics.homed, 1U, "the axis keeps its homing");
            expect_u32(fake_calibration_begin_calls, 0U, "the calibration does not begin");
            expect_u32(fake_axes[axis].bench_starts, 0U, "CALIBRATE does not start a move on an active axis");
            expect_u32(test_block_ready_notifications, 0U, "a refused CALIBRATE queues no block");
            expect_u32(test_last_receive_timeout, osWaitForever, "a refused calibration leaves nothing to poll");
            expect_u32(
                context.machine_state.homed_axes_mask & axis_masks[axis],
                axis_masks[axis],
                "the homed state of the axis is kept"
            );
        }
    }
}

/* STOP waits while any axis is STARTING, WAITING or RUNNING and gives up after the settle timeout; the generator is
 * then not reset. Every status on every axis is checked on its own. */
static void test_stop_waits_for_active_hardware_of_any_axis(void) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        for (uint32_t index = 0U; index < TEST_ACTIVE_STATUS_COUNT; ++index) {
            reset_receive_script();
            MotionTaskContext_t context = valid_motion_context();
            context.machine_state.position_um[0] = 5000;
            fake_encoder_x_um = -1234;
            if (axis == (uint32_t)MOTION_AXIS_X) {
                run_single_stop(&context, test_active_statuses[index]);
            } else {
                script_axis_state(
                    (MotionAxis_t)axis,
                    0U,
                    test_active_statuses[index],
                    MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
                    0U
                );
                run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
            }
            expect_u32(
                test_delay_calls,
                MOTION_TASK_STOP_SETTLE_TIMEOUT_MS,
                "STOP waits the whole settle time for an active axis"
            );
            expect_u32(test_generator_all_resumes, 0U, "the generators stay halted while an axis is active");
            expect_i64(context.machine_state.position_um[0], 5000, "positions are not taken from the scale meanwhile");
        }
    }

    /* Statuses that are not active do not hold STOP back. */
    const MotionAxisHardwareStatus_t at_rest[] = {
        MOTION_AXIS_HARDWARE_STATUS_READY,
        MOTION_AXIS_HARDWARE_STATUS_STOPPED,
        MOTION_AXIS_HARDWARE_STATUS_COMPLETE,
    };
    for (uint32_t index = 0U; index < sizeof(at_rest) / sizeof(at_rest[0]); ++index) {
        reset_receive_script();
        MotionTaskContext_t context = valid_motion_context();
        script_axis_state(MOTION_AXIS_Z, 0U, at_rest[index], MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
        run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
        expect_u32(test_delay_calls, 0U, "STOP does not wait for an axis at rest");
        expect_u32(test_generator_all_resumes, 1U, "the generators are reset once every axis is at rest");
    }
}

static void test_move_goes_to_the_queue_of_the_context(void) {
    static PlanBuffer_t own_buffer;
    static MotionPlanBlock_t own_buffer_entries[4];
    reset_receive_script();
    (void)memset(&own_buffer, 0, sizeof(own_buffer));
    expect_u32(plan_buffer_init(&own_buffer, own_buffer_entries, 4U), PLAN_BUFFER_STATUS_OK, "the queue of the context initializes");

    MotionTaskContext_t context = valid_motion_context();
    context.plan_buffer = &own_buffer;
    script_read(0U, osOK, &test_xyz_move_command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    PlanBufferDiagnostics_t own = {0};
    PlanBufferDiagnostics_t shared = {0};
    (void)plan_buffer_get_diagnostics(&own_buffer, &own);
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &shared);
    expect_u32(own.queued, 1U, "the move is queued in the queue of the context");
    expect_u32(own.enqueued_total, 1U, "the queue of the context counts the push");
    expect_u32(shared.queued, 0U, "the other queue gets no block");
    expect_u32(shared.enqueued_total, 0U, "the other queue counts no push");
    expect_u32(test_block_ready_notifications, 1U, "the generator is woken for the block");
    expect_i64(context.machine_state.position_um[0], 1000, "the next move starts at the target");

    MotionPlanBlock_t block = {0};
    expect_u32(plan_buffer_try_pop(&own_buffer, &block), PLAN_BUFFER_STATUS_OK, "the block is in the queue of the context");
    expect_u32(block.sequence, 141U, "the block carries the sequence of the command");
    expect_i64(block.step_delta[0], 80, "the block has the steps of 1000 um at 80 per 1000 um");
}

static void test_move_without_a_queue_is_dropped(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.plan_buffer = NULL;
    script_read(0U, osOK, &test_xyz_move_command);
    test_block_when_empty = 1U;
    expect_u32((uint32_t)run_motion_task_until_scripted_stop(&context), 2U, "the task survives a context without a queue");

    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_blocks_created, 1U, "the planner still builds the block");
    expect_u32(motion_task_get_last_planner_status(), MOTION_PLANNER_STATUS_OK, "the planner accepts the move");
    expect_u32(test_block_ready_notifications, 0U, "no block is queued, so the generator is not woken");
    expect_i64(context.machine_state.position_um[0], 0, "a dropped block does not move the planned position");
    PlanBufferDiagnostics_t shared = {0};
    (void)plan_buffer_get_diagnostics(&test_plan_buffer, &shared);
    expect_u32(shared.queued, 0U, "the other queue gets no block");
    expect_u32(shared.enqueued_total, 0U, "the other queue counts no push");
}

/* The hardware of an axis is active while its stream is idle (a move the stream does not know about): the command that
 * names the axis is refused whole, so no other axis of it starts either. */
static void check_command_refused_for_active_axis(const CommandDTO_t *command, MotionAxis_t active_axis) {
    reset_receive_script();
    test_receive_keeps_generator_idle = 1U;
    MotionTaskContext_t context = valid_motion_context();
    script_axis_state(active_axis, 0U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    script_read(0U, osOK, command);
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(&context);

    expect_u32(
        fake_x_bench_starts + fake_y.bench_starts + fake_z.bench_starts,
        0U,
        "no axis starts when the command names an axis whose hardware is active"
    );
    expect_u32(test_block_ready_notifications, 0U, "a refused command queues no block");
    expect_u32(
        enqueued_blocks(&gen_x.queue) + enqueued_blocks(&gen_y.queue) + enqueued_blocks(&gen_z.queue),
        0U,
        "a refused command leaves the axis queues empty"
    );
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "a refused command unhomes nothing");
    expect_u32(fake_calibration_begin_calls, 0U, "a refused command begins no calibration");
    expect_u32(test_last_receive_timeout, osWaitForever, "a refused command leaves nothing to poll");
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        MotionAxisDiagnostics diagnostics = {0};
        motion_task_get_axis_diagnostics((MotionAxis_t)axis, &diagnostics);
        expect_u32(
            diagnostics.calibration_result,
            AXIS_CALIBRATION_RUN_RESULT_NONE,
            "a refused command leaves every calibration result untouched"
        );
        expect_u32(fake_axes[axis].stop_requests, 0U, "a refused command stops nothing");
    }
}

static void test_command_naming_an_active_axis_is_refused_whole(void) {
    const CommandDTO_t home_xy = home_command_for(MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK);
    const CommandDTO_t calibrate_xy = calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK);
    check_command_refused_for_active_axis(&home_xy, MOTION_AXIS_Y);
    check_command_refused_for_active_axis(&calibrate_xy, MOTION_AXIS_Y);

    const CommandDTO_t home_all = home_command_for(MOTION_PLANNER_ALL_AXES_MASK);
    const CommandDTO_t calibrate_all = calibrate_command_for(MOTION_PLANNER_ALL_AXES_MASK);
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        check_command_refused_for_active_axis(&home_all, (MotionAxis_t)axis);
        check_command_refused_for_active_axis(&calibrate_all, (MotionAxis_t)axis);
    }
}

static MotionAxis_t test_released_axis;
static uint32_t test_planner_calls_at_release;

static void release_axis_instance_at_read_1(uint32_t read_index) {
    if (read_index == 1U) {
        test_planner_calls_at_release = motion_task_test_get_planner_call_count();
        fake_generators[test_released_axis].busy = 0U;
    }
}

/* The MotionTask states of the axes are free here; only the instance of an axis still plays its block. */
static void test_xyz_move_waits_for_the_instance_of_an_axis(void) {
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        reset_receive_script();
        MotionTaskContext_t context = valid_motion_context();
        fake_generators[axis].busy = 1U;
        test_released_axis = (MotionAxis_t)axis;
        test_planner_calls_at_release = UINT32_MAX;
        script_read(0U, osOK, &test_xyz_move_command);
        script_read(1U, osOK, &test_xyz_move_command);
        test_block_when_empty = 1U;
        test_receive_hook = release_axis_instance_at_read_1;
        (void)run_motion_task_until_scripted_stop(&context);

        expect_u32(test_planner_calls_at_release, 0U, "the move is not planned while an axis instance is busy");
        expect_u32(motion_task_test_get_planner_call_count(), 1U, "the move is planned once the instance is idle");
        expect_u32(test_block_ready_notifications, 1U, "only the accepted move wakes the generator");
        expect_u32(queued_blocks(&test_plan_buffer), 1U, "only the accepted move is queued for printing");
        expect_i64(context.machine_state.position_um[0], 1000, "the planned position follows the accepted move");
    }
}

/* A calibration that cannot start its final homing because the hardware of the axis is active must not report OK. */
static void test_calibration_whose_final_homing_cannot_start_fails(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    context.machine_state.homed_axes_mask = 0U;
    fake_calibration_poll_result = AXIS_CALIBRATION_RUN_RESULT_OK;
    fake_calibration_measured = (AxisCalibration_t){.steps = 50000U, .distance_um = 25000U, .travel_um = 50300U};
    script_axis_home_success(MOTION_AXIS_X, 1U);
    /* The procedure ends on read 4 while the X hardware is active again. */
    script_axis_state(MOTION_AXIS_X, 4U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_command_with_polls(&context, &test_calibrate_x_command, 5U);

    expect_u32(fake_x_bench_starts, 3U, "the final homing does not start while the hardware is active");
    expect_u32(context.calibration.calibrated_axes_mask, MOTION_PLANNER_AXIS_X_MASK, "the measurement is kept");
    MotionAxisDiagnostics axis = {0};
    motion_task_get_axis_diagnostics(MOTION_AXIS_X, &axis);
    expect_u32(
        axis.calibration_result,
        AXIS_CALIBRATION_RUN_RESULT_HOME_FAILED,
        "a calibration without its final homing is reported as failed"
    );
    expect_u32(axis.homed, 0U, "X is not homed after the calibration moved it away");
}

/* Wire values of the command results, written out here and not taken from the macros so that a changed value is
 * noticed: the GUI depends on them. The planner refusals are 0x0100 plus the MotionPlannerStatus_t. */
#define TEST_ERROR_INVALID_PARAMETER 0x0005U
#define TEST_ERROR_PLANNER_NOT_HOMED 0x0104U
#define TEST_ERROR_PLANNER_OUT_OF_RANGE 0x0105U
#define TEST_ERROR_PLANNER_INVALID_FEEDRATE 0x0106U
#define TEST_ERROR_PLANNER_AXIS_SPEED_LIMIT 0x0107U
#define TEST_ERROR_AXIS_BUSY 0x0120U
#define TEST_ERROR_LIMITS_CONFLICT 0x0121U
#define TEST_ERROR_HOME_NOT_STARTED 0x0122U
#define TEST_ERROR_PLAN_QUEUE_FULL 0x0123U
#define TEST_ERROR_NOT_SUPPORTED 0x0124U
#define TEST_ERROR_STOP_NOT_SETTLED 0x0125U
#define TEST_ERROR_NOT_CONFIGURED 0x0126U
#define TEST_DONE_ALREADY_AT_TARGET 0x0001U

/* Checks one recorded result: with a code of 0 a DONE frame with the result code ACCEPTED, otherwise an ERROR frame
 * with that code. */
static void expect_result_at(uint32_t index, uint32_t sequence, uint16_t error_code, const char *message) {
    if (index >= test_result_count) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (no result at index %lu)\n", message, (unsigned long)index);
        return;
    }
    expect_u32(test_results[index].sequence, sequence, message);
    expect_u32(test_results[index].response_type, error_code == 0U ? 0x03U : 0xFFU, message);
    expect_u32(test_results[index].code, error_code, message);
}

/* Checks one recorded DONE result with the given result code. */
static void expect_done_at(uint32_t index, uint32_t sequence, uint16_t result_code, const char *message) {
    if (index >= test_result_count) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s (no result at index %lu)\n", message, (unsigned long)index);
        return;
    }
    expect_u32(test_results[index].sequence, sequence, message);
    expect_u32(test_results[index].response_type, 0x03U, message);
    expect_u32(test_results[index].code, result_code, message);
}

static CommandDTO_t with_sequence(CommandDTO_t command, uint32_t sequence) {
    command.sequence = sequence;
    return command;
}

static CommandDTO_t xyz_move_to(uint32_t sequence, int32_t x_um, uint32_t speed_um_per_s) {
    return (CommandDTO_t){
        .sequence = sequence,
        .code = COMMAND_CODE_MOTION_OPERATION,
        .parameters.motion_operation =
            {
                .operation_flags = COMMAND_MOTION_OPERATION_FLAG_XYZ_MOVE,
                .x = x_um,
                .speed = speed_um_per_s,
            },
    };
}

/* Delivers the commands one per read, then ends the run when the queue is empty. */
static void run_commands(MotionTaskContext_t *context, const CommandDTO_t *commands, uint32_t count) {
    for (uint32_t index = 0U; index < count; ++index) {
        script_read(index, osOK, &commands[index]);
    }
    test_block_when_empty = 1U;
    (void)run_motion_task_until_scripted_stop(context);
}

static void test_result_of_an_accepted_move_is_zero(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t move = xyz_move_to(141U, 1000, 1000U);
    run_commands(&context, &move, 1U);

    expect_u32(test_result_count, 1U, "an accepted move is answered once");
    expect_done_at(0U, 141U, COMMAND_DONE_ACCEPTED, "an accepted move is answered with DONE ACCEPTED");
    expect_u32(queued_blocks(&test_plan_buffer), 1U, "the accepted move is queued");
}

typedef struct {
    const char *name;
    uint8_t homed_axes_mask;
    int32_t x_um;
    uint32_t speed_um_per_s;
    uint16_t expected_code;
} TestPlannerRefusal_t;

/* The context allows X from -100000 to 100000 um at up to 100000 um/s. */
static const TestPlannerRefusal_t test_planner_refusals[] = {
    {"a move of an unhomed axis", 0U, 1000, 1000U, TEST_ERROR_PLANNER_NOT_HOMED},
    {"a target beyond the travel", MOTION_PLANNER_ALL_AXES_MASK, 100001, 1000U, TEST_ERROR_PLANNER_OUT_OF_RANGE},
    {"a move without a feedrate", MOTION_PLANNER_ALL_AXES_MASK, 1000, 0U, TEST_ERROR_PLANNER_INVALID_FEEDRATE},
    {"a move faster than the axis", MOTION_PLANNER_ALL_AXES_MASK, 1000, 200000U, TEST_ERROR_PLANNER_AXIS_SPEED_LIMIT},
};

static void test_result_of_a_planner_refusal_is_the_planner_code(void) {
    for (uint32_t index = 0U; index < sizeof(test_planner_refusals) / sizeof(test_planner_refusals[0]); ++index) {
        const TestPlannerRefusal_t *refusal = &test_planner_refusals[index];
        reset_receive_script();
        MotionTaskContext_t context = valid_motion_context();
        context.machine_state.homed_axes_mask = refusal->homed_axes_mask;
        const CommandDTO_t move = xyz_move_to(300U + index, refusal->x_um, refusal->speed_um_per_s);
        run_commands(&context, &move, 1U);

        expect_u32(test_result_count, 1U, refusal->name);
        expect_result_at(0U, 300U + index, refusal->expected_code, refusal->name);
        expect_u32(enqueued_blocks(&test_plan_buffer), 0U, "a refused move queues nothing");
        expect_u32(test_block_ready_notifications, 0U, "a refused move wakes no generator");
    }

    reset_receive_script();
    const CommandDTO_t move = xyz_move_to(310U, 1000, 1000U);
    run_commands(NULL, &move, 1U);
    expect_u32(test_result_count, 1U, "a move without a context is answered once");
    expect_result_at(0U, 310U, TEST_ERROR_NOT_CONFIGURED, "a move without a context is not configured");
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "a move without a context does not reach the planner");
    MotionTaskDiagnostics diagnostics = {0};
    motion_task_get_diagnostics(&diagnostics);
    expect_u32(diagnostics.planner_calls, 0U, "a move without a context is not counted as a planner call");
    expect_u32(test_block_ready_notifications, 0U, "a move without a context wakes no generator");
}

static void test_result_of_a_move_to_the_current_position_is_done_at_target(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t move = xyz_move_to(142U, 0, 1000U);
    run_commands(&context, &move, 1U);

    expect_u32(test_result_count, 1U, "a move to the current position is answered once");
    expect_done_at(0U, 142U, TEST_DONE_ALREADY_AT_TARGET, "a move to the current position is DONE, already at target");
    expect_u32(enqueued_blocks(&test_plan_buffer), 0U, "a move to the current position queues nothing");
    expect_u32(test_block_ready_notifications, 0U, "a move to the current position wakes no generator");
}

static void test_result_of_a_move_into_a_full_plan_buffer(void) {
    reset_receive_script();
    for (uint32_t sequence = 1U; sequence <= PLAN_BUFFER_CAPACITY; ++sequence) {
        const MotionPlanBlock_t block = {.sequence = sequence};
        expect_u32(plan_buffer_try_push(&test_plan_buffer, &block), PLAN_BUFFER_STATUS_OK, "the plan buffer is filled");
    }
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t move = xyz_move_to(320U, 1234, 12000U);
    run_commands(&context, &move, 1U);

    expect_u32(test_result_count, 1U, "a move that does not fit is answered once");
    expect_result_at(0U, 320U, TEST_ERROR_PLAN_QUEUE_FULL, "a full plan buffer refuses the move");
    expect_u32(test_block_ready_notifications, 0U, "the refused block wakes no generator");
    expect_i64(context.machine_state.position_um[0], 0, "the planned position stays where it was");
}

static void test_result_of_a_move_while_an_axis_is_busy(void) {
    /* An axis homes. */
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t homing_then_move[] = {with_sequence(test_home_x_command, 330U), xyz_move_to(331U, 1000, 1000U)};
    run_commands(&context, homing_then_move, 2U);
    expect_u32(test_result_count, 2U, "both commands are answered");
    expect_result_at(0U, 330U, 0U, "HOME of an idle axis is accepted");
    expect_result_at(1U, 331U, TEST_ERROR_AXIS_BUSY, "a move is refused while an axis homes");
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "the refused move is not planned");

    /* The stream of one axis still plays a block, whichever axis it is. */
    for (uint32_t axis = 0U; axis < (uint32_t)MOTION_AXIS_HARDWARE_COUNT; ++axis) {
        reset_receive_script();
        context = valid_motion_context();
        fake_generators[axis].busy = 1U;
        const CommandDTO_t move = xyz_move_to(340U + (uint32_t)axis, 1000, 1000U);
        run_commands(&context, &move, 1U);
        expect_u32(test_result_count, 1U, "a move is answered once while a stream is busy");
        expect_result_at(0U, 340U + axis, TEST_ERROR_AXIS_BUSY, "a move is refused while an axis stream plays");
        expect_u32(motion_task_test_get_planner_call_count(), 0U, "the refused move is not planned");
    }
}

static void test_result_of_home_and_calibrate_that_start(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t commands[] = {
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 350U),
        with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_Y_MASK), 351U),
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_Z_MASK), 352U),
    };
    run_commands(&context, commands, 3U);
    expect_u32(test_result_count, 3U, "every command is answered once");
    expect_result_at(0U, 350U, 0U, "HOME of X is accepted");
    expect_result_at(1U, 351U, 0U, "CALIBRATE of Y is accepted while X homes");
    expect_result_at(2U, 352U, 0U, "HOME of Z is accepted while X homes and Y calibrates");
    expect_u32(fake_x_bench_starts, 1U, "X started its seek");
    expect_u32(fake_y.bench_starts, 1U, "Y started its seek");
    expect_u32(fake_z.bench_starts, 1U, "Z started its seek");
}

static void test_result_of_home_and_calibrate_for_busy_axes(void) {
    /* The axis already homes or calibrates. */
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t repeated[] = {
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 360U),
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 361U),
        with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK), 362U),
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK | MOTION_PLANNER_AXIS_Y_MASK), 363U),
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_Y_MASK), 364U),
    };
    run_commands(&context, repeated, 5U);
    expect_u32(test_result_count, 5U, "every command is answered once");
    expect_result_at(0U, 360U, 0U, "the first HOME is accepted");
    expect_result_at(1U, 361U, TEST_ERROR_AXIS_BUSY, "HOME of a homing axis is refused");
    expect_result_at(2U, 362U, TEST_ERROR_AXIS_BUSY, "CALIBRATE of a homing axis is refused");
    expect_result_at(3U, 363U, TEST_ERROR_AXIS_BUSY, "a command that names a busy axis is refused as a whole");
    expect_result_at(4U, 364U, 0U, "an idle axis is accepted meanwhile");
    expect_u32(fake_x_bench_starts, 1U, "X is not restarted");
    expect_u32(fake_y.bench_starts, 1U, "Y starts only with the command that does not name X");

    /* The printing stream plays blocks: it holds every axis. */
    reset_receive_script();
    context = valid_motion_context();
    gen_print.busy = 1U;
    const CommandDTO_t home_y = with_sequence(home_command_for(MOTION_PLANNER_AXIS_Y_MASK), 365U);
    run_commands(&context, &home_y, 1U);
    expect_u32(test_result_count, 1U, "HOME during printing is answered once");
    expect_result_at(0U, 365U, TEST_ERROR_AXIS_BUSY, "HOME is refused while the printing stream plays");
    expect_u32(fake_y.bench_starts, 0U, "the refused HOME does not move");

    /* The hardware of the axis moves although its stream is idle. */
    for (uint32_t calibrate = 0U; calibrate < 2U; ++calibrate) {
        reset_receive_script();
        test_receive_keeps_generator_idle = 1U;
        context = valid_motion_context();
        script_axis_state(
            MOTION_AXIS_Z,
            0U,
            MOTION_AXIS_HARDWARE_STATUS_RUNNING,
            MOTION_AXIS_HARDWARE_STOP_REASON_NONE,
            0U
        );
        const CommandDTO_t command = calibrate != 0U
                                         ? with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_Z_MASK), 366U)
                                         : with_sequence(home_command_for(MOTION_PLANNER_AXIS_Z_MASK), 366U);
        run_commands(&context, &command, 1U);
        expect_u32(test_result_count, 1U, "a command for an active axis is answered once");
        expect_result_at(0U, 366U, TEST_ERROR_AXIS_BUSY, "an axis whose hardware moves is busy");
        expect_u32(fake_z.bench_starts, 0U, "nothing starts on an active axis");
    }
}

static void test_result_of_home_and_calibrate_with_wrong_axes(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t commands[] = {
        with_sequence(home_command_for(0x00U), 370U),
        with_sequence(calibrate_command_for(0x00U), 371U),
        with_sequence(home_command_for(0x08U), 372U),
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK | 0x08U), 373U),
        with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK | 0x80U), 374U),
    };
    run_commands(&context, commands, 5U);
    expect_u32(test_result_count, 5U, "every command is answered once");
    for (uint32_t index = 0U; index < 5U; ++index) {
        expect_result_at(index, 370U + index, TEST_ERROR_INVALID_PARAMETER, "a mask without axes or with extra bits");
    }
    expect_u32(fake_x_bench_starts, 0U, "a refused mask starts no axis, even a valid one in it");
    expect_u32(context.machine_state.homed_axes_mask, MOTION_PLANNER_ALL_AXES_MASK, "a refused mask unhomes nothing");
}

static void test_result_of_home_and_calibrate_without_a_context(void) {
    reset_receive_script();
    const CommandDTO_t commands[] = {
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 380U),
        with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK), 381U),
    };
    run_commands(NULL, commands, 2U);
    expect_u32(test_result_count, 2U, "both commands are answered");
    expect_result_at(0U, 380U, TEST_ERROR_NOT_CONFIGURED, "HOME without a context is not configured");
    expect_result_at(1U, 381U, TEST_ERROR_NOT_CONFIGURED, "CALIBRATE without a context is not configured");
    expect_u32(fake_x_bench_starts, 0U, "nothing moves without a context");
}

static void test_result_of_home_and_calibrate_that_do_not_start(void) {
    /* Both switches of the axis are active. */
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    fake_x_end_limit_active = 1U;
    const CommandDTO_t conflicts[] = {
        with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 390U),
        with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK), 391U),
    };
    run_commands(&context, conflicts, 2U);
    expect_u32(test_result_count, 2U, "both commands are answered");
    expect_result_at(0U, 390U, TEST_ERROR_LIMITS_CONFLICT, "HOME with both switches active is a conflict");
    expect_result_at(1U, 391U, TEST_ERROR_LIMITS_CONFLICT, "CALIBRATE with both switches active is a conflict");

    /* The start switch alone is a normal start. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    const CommandDTO_t on_switch = with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 392U);
    run_commands(&context, &on_switch, 1U);
    expect_result_at(0U, 392U, 0U, "HOME on the start switch alone is accepted");

    /* The seek block cannot be built. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    const CommandDTO_t home_x = with_sequence(home_command_for(MOTION_PLANNER_AXIS_X_MASK), 393U);
    run_commands(&context, &home_x, 1U);
    expect_u32(test_result_count, 1U, "a HOME that does not start is answered once");
    expect_result_at(0U, 393U, TEST_ERROR_HOME_NOT_STARTED, "HOME whose seek cannot start is not started");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    const CommandDTO_t calibrate_x = with_sequence(calibrate_command_for(MOTION_PLANNER_AXIS_X_MASK), 394U);
    run_commands(&context, &calibrate_x, 1U);
    expect_u32(test_result_count, 1U, "a CALIBRATE that does not start is answered once");
    expect_result_at(0U, 394U, TEST_ERROR_HOME_NOT_STARTED, "CALIBRATE whose homing cannot start is not started");

    /* The first axis that fails gives the code; the other axes still start. */
    reset_receive_script();
    context = valid_motion_context();
    fake_x_failing_start = 1U;
    fake_y.start_limit_active = 1U;
    fake_y.end_limit_active = 1U;
    const CommandDTO_t home_xyz = with_sequence(home_command_for(MOTION_PLANNER_ALL_AXES_MASK), 395U);
    run_commands(&context, &home_xyz, 1U);
    expect_u32(test_result_count, 1U, "a command with failing axes is answered once");
    expect_result_at(0U, 395U, TEST_ERROR_HOME_NOT_STARTED, "the code of the first failing axis (X) is reported");
    expect_u32(fake_y.bench_starts, 0U, "Y with both switches active does not move");
    expect_u32(fake_z.bench_starts, 1U, "Z still starts although X and Y did not");

    reset_receive_script();
    context = valid_motion_context();
    fake_x_start_limit_active = 1U;
    fake_x_end_limit_active = 1U;
    fake_y.failing_start = 1U;
    run_commands(&context, &home_xyz, 1U);
    expect_u32(test_result_count, 1U, "a command with failing axes is answered once");
    expect_result_at(0U, 395U, TEST_ERROR_LIMITS_CONFLICT, "the code of the first failing axis (X) is reported");
    expect_u32(fake_y.bench_starts, 1U, "Y tried to start although X did not");
    expect_u32(fake_z.bench_starts, 1U, "Z still starts although X and Y did not");
}

static void test_result_of_stop(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_READY);
    expect_u32(test_result_count, 1U, "STOP is answered once");
    expect_result_at(0U, 120U, 0U, "STOP with the axes at rest is answered with 0");

    /* An axis that keeps moving does not come to rest within the settle time. */
    reset_receive_script();
    context = valid_motion_context();
    run_single_stop(&context, MOTION_AXIS_HARDWARE_STATUS_RUNNING);
    expect_u32(test_result_count, 1U, "an unsettled STOP is answered once");
    expect_result_at(0U, 120U, TEST_ERROR_STOP_NOT_SETTLED, "STOP whose axes do not settle says so");
}

static void test_result_of_unsupported_commands(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t commands[] = {
        {
            .sequence = 400U,
            .code = COMMAND_CODE_SET_TEMPERATURE,
            .parameters.set_temperature = {.heater_id = 2U, .temperature = 215U},
        },
        {.sequence = 401U, .code = COMMAND_CODE_SET_OUTPUT, .parameters.set_output = {.output_id = 4U, .state = 1U}},
        {.sequence = 402U, .code = COMMAND_CODE_CHANGE_TOOL, .parameters.change_tool = {.tool_id = 7U}},
        {
            .sequence = 403U,
            .code = COMMAND_CODE_MOTION_OPERATION,
            .parameters.motion_operation = {.operation_flags = 0x04U, .spindle_pwm = 5000U},
        },
        {
            .sequence = 404U,
            .code = COMMAND_CODE_MOTION_OPERATION,
            .parameters.motion_operation = {.operation_flags = COMMAND_MOTION_OPERATION_FLAG_RELATIVE, .x = 500},
        },
    };
    run_commands(&context, commands, 5U);
    expect_u32(test_result_count, 5U, "every unsupported command is answered once");
    for (uint32_t index = 0U; index < 5U; ++index) {
        expect_result_at(index, 400U + index, TEST_ERROR_NOT_SUPPORTED, "a command MotionTask does not run");
    }
    expect_u32(motion_task_test_get_planner_call_count(), 0U, "unsupported commands are not planned");
}

static void test_stop_erases_unread_results_and_keeps_its_own(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t commands[] = {
        xyz_move_to(500U, 1000, 1000U),
        xyz_move_to(501U, 0, 0U),
        {.sequence = 502U, .code = COMMAND_CODE_SET_OUTPUT, .parameters.set_output = {.output_id = 4U, .state = 1U}},
        {.sequence = 503U, .code = COMMAND_CODE_STOP},
    };
    run_commands(&context, commands, 4U);

    expect_u32(test_discard_calls, 1U, "STOP discards the unread results once");
    expect_u32(test_discarded_results, 3U, "the answers of the three earlier commands were waiting");
    expect_u32(test_result_count, 1U, "only the answer of STOP is left");
    expect_done_at(0U, 503U, COMMAND_DONE_ACCEPTED, "STOP is answered with DONE ACCEPTED after the discard");
}

static void test_unsettled_stop_answers_after_discarding_earlier_results(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t commands[] = {
        xyz_move_to(510U, 1000, 1000U),
        {.sequence = 511U, .code = COMMAND_CODE_STOP},
    };
    /* The X axis keeps running through the whole settle time. */
    script_x_state(1U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_commands(&context, commands, 2U);

    expect_u32(test_discarded_results, 1U, "the answer of the move was erased");
    expect_u32(test_result_count, 1U, "only the answer of STOP is left");
    expect_result_at(0U, 511U, TEST_ERROR_STOP_NOT_SETTLED, "an unsettled STOP is answered with the error");
}

static void test_internal_stop_discards_results_and_posts_none(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t commands[] = {
        xyz_move_to(520U, 1000, 1000U),
        {.sequence = COMMAND_SEQUENCE_INTERNAL, .code = COMMAND_CODE_STOP},
    };
    run_commands(&context, commands, 2U);

    expect_u32(test_discard_calls, 1U, "the internal STOP discards the unread results");
    expect_u32(test_discarded_results, 1U, "the answer of the move was erased");
    expect_u32(test_result_count, 0U, "the internal STOP is not answered");
    expect_u32(test_generator_all_resumes, 1U, "the internal STOP still restarts the generators");
    expect_u32(queued_blocks(&test_plan_buffer), 0U, "the internal STOP still empties the printing queue");
    expect_u32(fake_x_stop_requests, 1U, "the internal STOP still stops the axes");
}

static void test_unsettled_internal_stop_posts_nothing(void) {
    reset_receive_script();
    MotionTaskContext_t context = valid_motion_context();
    const CommandDTO_t stop = {.sequence = COMMAND_SEQUENCE_INTERNAL, .code = COMMAND_CODE_STOP};
    /* The X axis keeps running through the whole settle time. */
    script_x_state(0U, MOTION_AXIS_HARDWARE_STATUS_RUNNING, MOTION_AXIS_HARDWARE_STOP_REASON_NONE, 0U);
    run_commands(&context, &stop, 1U);

    expect_u32(test_discard_calls, 1U, "the internal STOP discards the results");
    expect_u32(test_result_count, 0U, "an unsettled internal STOP is not answered");
    expect_u32(test_generator_all_resumes, 0U, "the generators stay halted after an unsettled STOP");
}

static void test_request_stop_drops_the_commands_behind_it_silently(void) {
    reset_receive_script();
    test_pending_commands[0] = (CommandDTO_t){.sequence = 210U, .code = COMMAND_CODE_HOME};
    test_pending_commands[1] = xyz_move_to(211U, 1000, 1000U);
    test_pending_count = 2U;

    motion_task_request_stop();
    expect_u32(test_result_count, 0U, "no command behind STOP is answered");
    expect_u32(test_discard_calls, 0U, "the immediate STOP does not touch the unread results");
    expect_u32(test_pending_receive_calls, 3U, "the queue is read until it is empty");
    expect_u32(test_pending_position, 2U, "both commands were taken out of the queue");
    expect_u32(test_pending_max_timeout, 0U, "STOP never waits for the queue");
    expect_u32(fake_x_stop_requests, 1U, "the pulses are stopped as well");
}

int main(void) {
    test_result_of_an_accepted_move_is_zero();
    test_result_of_a_planner_refusal_is_the_planner_code();
    test_result_of_a_move_to_the_current_position_is_done_at_target();
    test_result_of_a_move_into_a_full_plan_buffer();
    test_result_of_a_move_while_an_axis_is_busy();
    test_result_of_home_and_calibrate_that_start();
    test_result_of_home_and_calibrate_for_busy_axes();
    test_result_of_home_and_calibrate_with_wrong_axes();
    test_result_of_home_and_calibrate_without_a_context();
    test_result_of_home_and_calibrate_that_do_not_start();
    test_result_of_stop();
    test_result_of_unsupported_commands();
    test_stop_erases_unread_results_and_keeps_its_own();
    test_unsettled_stop_answers_after_discarding_earlier_results();
    test_internal_stop_discards_results_and_posts_none();
    test_unsettled_internal_stop_posts_nothing();
    test_request_stop_drops_the_commands_behind_it_silently();
    test_move_goes_to_the_queue_of_the_context();
    test_move_without_a_queue_is_dropped();
    test_stored_calibration_sets_planner_limits();
    test_calibration_runs_after_homing_and_applies_result();
    test_calibration_failures();
    test_calibration_moves_play_through_the_stream();
    test_calibration_abort_flushes_the_stream();
    test_calibration_is_cancelled_and_guarded();
    test_x_hardware_is_initialized_before_first_command();
    test_stream_event_source_is_registered_for_every_axis();
    test_home_phase_outcomes();
    test_home_start_failures();
    test_x_homing_through_the_stream();
    test_commands_during_home();
    test_home_is_refused_when_it_cannot_start();
    test_home_and_calibrate_wait_for_the_planned_stream();
    test_step_timing_comes_from_the_axis_config();
    test_limit_deceleration_comes_from_the_axis_config();
    test_relative_move_is_planned_from_the_current_position();
    test_relative_move_applies_increments_to_every_axis();
    test_stop_fault_acknowledgement_edges();
    test_stop_clears_runtime_fault_and_x_homing();
    test_stop_without_fault_keeps_homing();
    test_stop_request_and_pipeline_reset();
    test_home_fast_seek_moves_toward_start_and_backs_off_on_pf10();
    test_home_fast_seek_aborts_on_end_limit();
    test_home_completes_only_on_pf10_during_approach();
    test_home_backoff_aborts_when_stopped_by_end_limit();
    test_home_backoff_continues_while_pf10_flickers();
    test_both_limits_active_refuse_homing_and_calibration();
    test_successful_receive_marks_dequeue();
    test_fifo_commands_are_dequeued_once();
    test_receive_error_does_not_count_as_dequeue();
    test_diagnostic_counters_saturate();
    test_xyz_command_reports_unconfigured_planner_inputs();
    test_xyz_command_without_position_change_counts_no_motion();
    test_relative_move_without_increment_is_already_at_target();
    test_full_plan_buffer_rejects_new_block_without_losing_existing_plans();
    test_xyz_command_with_null_context_is_not_configured();
    test_motion_operation_without_xyz_flag_does_not_call_planner();
    test_y_homing_through_the_stream();
    test_z_homing_through_the_stream();
    test_axes_of_one_command_start_together();
    test_home_of_three_axes_runs_in_parallel();
    test_failure_of_one_axis_does_not_end_the_homing_of_the_others();
    test_calibrate_of_two_axes_runs_in_parallel();
    test_stop_during_the_homing_of_three_axes();
    test_home_of_y_is_accepted_while_x_homes();
    test_xyz_move_is_refused_while_any_axis_homes();
    test_home_starts_once_the_printing_stream_is_idle();
    test_command_naming_an_active_axis_is_refused_whole();
    test_xyz_move_waits_for_the_instance_of_an_axis();
    test_calibration_whose_final_homing_cannot_start_fails();
    test_home_and_calibrate_masks();
    test_y_calibration_reports_on_y();
    test_axis_diagnostics_edges();
    test_z_home_and_calibration();
    test_planned_move_of_z_is_queued();
    test_planned_move_of_y_is_queued();
    test_stop_reaches_every_axis();
    test_home_and_calibrate_are_refused_while_the_hardware_is_active();
    test_stop_waits_for_active_hardware_of_any_axis();
    expect_u32(test_axis_bench_calls, 0U, "no axis uses the ramped driver start: all of them play through the stream");

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Motion task unit tests failed: %u\n", test_failures);
        return 1;
    }
    (void)printf("Motion task unit tests passed.\n");
    return 0;
}

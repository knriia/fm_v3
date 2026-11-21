#include "step_generator_channels.h"

#include "syringe_driver.h"

/* The driver of the axes X, Y and Z: the stream channel and the outputs, switches and status of motion_axis_hardware.
 */
static StepStreamAxis_t *step_generator_channels_axis_stream(uint32_t id) {
    return step_stream_axis_get((MotionAxis_t)id);
}

static uint8_t step_generator_channels_axis_set_direction(uint32_t id, uint8_t positive) {
    const MotionAxisHardwareStatus_t status = motion_axis_hardware_set_stream_outputs((MotionAxis_t)id, positive);
    /* A refusal returns the status of a busy or faulted axis; a write leaves a status the axis can start from. */
    return (uint8_t)(status == MOTION_AXIS_HARDWARE_STATUS_READY || status == MOTION_AXIS_HARDWARE_STATUS_COMPLETE ||
                     status == MOTION_AXIS_HARDWARE_STATUS_STOPPED);
}

static void step_generator_channels_axis_set_watched(uint32_t id, uint8_t watched) {
    motion_axis_hardware_set_stream_axis_watched((MotionAxis_t)id, watched);
}

static uint8_t step_generator_channels_axis_set_limit_listener(
    uint32_t id,
    MotionAxisHardwareStreamLimitListener_t listener,
    void *context
) {
    return motion_axis_hardware_set_stream_limit_listener((MotionAxis_t)id, listener, context);
}

static void step_generator_channels_axis_get_hardware(uint32_t id, uint32_t *status, uint32_t *stop_reason) {
    *status = (uint32_t)motion_axis_hardware_get_status((MotionAxis_t)id);
    *stop_reason = (uint32_t)motion_axis_hardware_get_last_stop_reason((MotionAxis_t)id);
}

const StepGeneratorChannelDriver_t step_generator_channels_axis_driver = {
    .stream = step_generator_channels_axis_stream,
    .set_direction = step_generator_channels_axis_set_direction,
    .set_watched = step_generator_channels_axis_set_watched,
    .set_limit_listener = step_generator_channels_axis_set_limit_listener,
    .get_hardware = step_generator_channels_axis_get_hardware,
};

/* The driver of the syringe: its own stream channel and DIR/ENA outputs; it has no switches and no hardware axis. */
static StepStreamAxis_t *step_generator_channels_syringe_stream(uint32_t id) {
    (void)id;
    return step_stream_axis_get_syringe();
}

static uint8_t step_generator_channels_syringe_set_direction(uint32_t id, uint8_t positive) {
    (void)id;
    return syringe_driver_set_stream_outputs(positive);
}

static void step_generator_channels_syringe_set_watched(uint32_t id, uint8_t watched) {
    (void)id;
    (void)watched;
}

static uint8_t step_generator_channels_syringe_set_limit_listener(
    uint32_t id,
    MotionAxisHardwareStreamLimitListener_t listener,
    void *context
) {
    (void)id;
    (void)listener;
    (void)context;
    return 1U;
}

static void step_generator_channels_syringe_get_hardware(uint32_t id, uint32_t *status, uint32_t *stop_reason) {
    (void)id;
    *status = 0U;
    *stop_reason = 0U;
}

const StepGeneratorChannelDriver_t step_generator_channels_syringe_driver = {
    .stream = step_generator_channels_syringe_stream,
    .set_direction = step_generator_channels_syringe_set_direction,
    .set_watched = step_generator_channels_syringe_set_watched,
    .set_limit_listener = step_generator_channels_syringe_set_limit_listener,
    .get_hardware = step_generator_channels_syringe_get_hardware,
};

const StepGeneratorMember_t step_generator_channels_print_members[STEP_GENERATOR_CHANNELS_PRINT_COUNT] = {
    {&step_generator_channels_axis_driver, MOTION_AXIS_X, MOTION_AXIS_X},
    {&step_generator_channels_axis_driver, MOTION_AXIS_Y, MOTION_AXIS_Y},
    {&step_generator_channels_axis_driver, MOTION_AXIS_Z, MOTION_AXIS_Z},
    {&step_generator_channels_syringe_driver, 0U, STEP_GENERATOR_CHANNELS_NO_AXIS},
};

const uint32_t step_generator_channels_print_planner_channels[STEP_GENERATOR_CHANNELS_PRINT_COUNT] = {
    0U,
    1U,
    2U,
    MOTION_PLAN_CHANNEL_SYRINGE,
};

const StepGeneratorMember_t step_generator_channels_axis_members[STEP_GENERATOR_CHANNELS_AXIS_COUNT] = {
    {&step_generator_channels_axis_driver, MOTION_AXIS_X, MOTION_AXIS_X},
    {&step_generator_channels_axis_driver, MOTION_AXIS_Y, MOTION_AXIS_Y},
    {&step_generator_channels_axis_driver, MOTION_AXIS_Z, MOTION_AXIS_Z},
};

const uint32_t step_generator_channels_axis_planner_channels[STEP_GENERATOR_CHANNELS_AXIS_COUNT] = {0U, 1U, 2U};

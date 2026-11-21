#ifndef FM_V3_STEP_GENERATOR_CHANNELS_H
#define FM_V3_STEP_GENERATOR_CHANNELS_H

#include "motion/motion_planner.h"
#include "motion_axis_hardware.h"
#include "step_stream_axis.h"

#include <stdint.h>

/* What a stream member needs besides its stream channel: the driver outputs and the switches that go with the motor.
 * The step generator depends on this interface and not on MotionAxis_t, so a motor without an axis (the syringe) is a
 * driver of its own and the generator never asks which kind a member is. `id` is whatever the driver needs to find its
 * motor. */
typedef struct {
    StepStreamAxis_t *(*stream)(uint32_t id);
    /* Enables the driver and writes DIR; 1 when written, 0 when the motor refuses (busy or faulted). */
    uint8_t (*set_direction)(uint32_t id, uint8_t positive);
    /* Whether the switches of the motor stop the running stream; a motor without switches ignores it. */
    void (*set_watched)(uint32_t id, uint8_t watched);
    /* Makes the callback the owner of the switches of the motor (see motion_axis_hardware_set_stream_limit_listener); a
     * motor without switches has nothing to own and always returns 1. */
    uint8_t (*set_limit_listener)(uint32_t id, MotionAxisHardwareStreamLimitListener_t listener, void *context);
    /* MotionAxisHardwareStatus_t and the last stop reason of the motor; both 0 for one without a hardware axis. */
    void (*get_hardware)(uint32_t id, uint32_t *status, uint32_t *stop_reason);
} StepGeneratorChannelDriver_t;

/* reported_axis of a member that is no MotionAxis_t. */
#define STEP_GENERATOR_CHANNELS_NO_AXIS 0xFFFFFFFFU

typedef struct {
    const StepGeneratorChannelDriver_t *driver;
    uint32_t id;
    /* MotionAxis_t that a halt blames and a switch report names, or STEP_GENERATOR_CHANNELS_NO_AXIS. */
    uint32_t reported_axis;
} StepGeneratorMember_t;

/* The drivers: an axis (X, Y or Z, `id` is the MotionAxis_t) and the syringe (`id` is not used). */
extern const StepGeneratorChannelDriver_t step_generator_channels_axis_driver;
extern const StepGeneratorChannelDriver_t step_generator_channels_syringe_driver;

/* The members of the printing stream: X, Y, Z and the syringe. */
#define STEP_GENERATOR_CHANNELS_PRINT_COUNT 4U
extern const StepGeneratorMember_t step_generator_channels_print_members[STEP_GENERATOR_CHANNELS_PRINT_COUNT];
/* The channel of the plan block (MOTION_PLAN_CHANNEL_*) behind each of them, in the same order. */
extern const uint32_t step_generator_channels_print_planner_channels[STEP_GENERATOR_CHANNELS_PRINT_COUNT];

/* The single member of the stream of one axis (HOME and CALIBRATE), indexed by MotionAxis_t. */
#define STEP_GENERATOR_CHANNELS_AXIS_COUNT 3U
extern const StepGeneratorMember_t step_generator_channels_axis_members[STEP_GENERATOR_CHANNELS_AXIS_COUNT];
extern const uint32_t step_generator_channels_axis_planner_channels[STEP_GENERATOR_CHANNELS_AXIS_COUNT];

#endif /* FM_V3_STEP_GENERATOR_CHANNELS_H */

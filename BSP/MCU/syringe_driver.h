#ifndef FM_V3_SYRINGE_DRIVER_H
#define FM_V3_SYRINGE_DRIVER_H

#include <stdint.h>

/* The driver outputs of the syringe motor: DIR on PC7 and ENA on PG8. STEP (TIM8 CH1, PC6) belongs to the stream
 * channel step_stream_axis_get_syringe(). The syringe has no limit switch and no homing, so this is all it needs. The
 * generator task is the only writer of both pins: it enables the driver and writes DIR when a stream opens. */
typedef struct {
    /* 1: the driver is enabled by a high ENA level. */
    uint8_t enable_active_high;
    /* 1: a high DIR level is the positive direction (feeding forward). */
    uint8_t positive_direction_high;
    /* The driver wants ENA this long before the first STEP. */
    uint32_t enable_setup_ns;
} SyringeDriverConfig_t;

/* Sets PC7 and PG8 up as outputs with the driver disabled and DIR positive. Call once before the scheduler starts.
 * Returns 1 when done, 0 for a NULL config (nothing is touched). */
uint8_t syringe_driver_init(const SyringeDriverConfig_t *config);

/* Enables the driver, waits the enable setup time and writes DIR. Returns 1 when written, 0 before syringe_driver_init.
 * The caller waits the DIR setup time before the first STEP. The driver stays enabled after the stream, as the axes do.
 */
uint8_t syringe_driver_set_stream_outputs(uint8_t positive_direction);

#endif /* FM_V3_SYRINGE_DRIVER_H */

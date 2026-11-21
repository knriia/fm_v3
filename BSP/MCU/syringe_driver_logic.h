#ifndef FM_V3_SYRINGE_DRIVER_LOGIC_H
#define FM_V3_SYRINGE_DRIVER_LOGIC_H

#include <stdint.h>

/* The pin levels of the driver outputs of the syringe, without any register. Every flag is taken as a boolean (non-zero
 * is true), so a configuration that holds 2 where 1 was meant still drives the pin the way it says. */

/* Level (1 high, 0 low) of ENA for an enabled or a disabled driver, given the level that enables it. */
uint8_t syringe_driver_logic_enable_level(uint8_t enabled, uint8_t enable_active_high);

/* Level of DIR for the positive or the negative direction, given whether a high level is the positive direction. */
uint8_t syringe_driver_logic_direction_level(uint8_t positive, uint8_t positive_direction_high);

#endif /* FM_V3_SYRINGE_DRIVER_LOGIC_H */

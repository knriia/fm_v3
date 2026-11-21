#include "syringe_driver_logic.h"

uint8_t syringe_driver_logic_enable_level(uint8_t enabled, uint8_t enable_active_high) {
    return (uint8_t)((enabled != 0U) == (enable_active_high != 0U));
}

uint8_t syringe_driver_logic_direction_level(uint8_t positive, uint8_t positive_direction_high) {
    return (uint8_t)((positive != 0U) == (positive_direction_high != 0U));
}

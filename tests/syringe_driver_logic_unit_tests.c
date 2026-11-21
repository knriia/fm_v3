#include "syringe_driver_logic.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* The two functions are the same equality of booleans; each has its own table so that a swap of them shows. */

static void test_enable_level_follows_the_polarity(void) {
    /* Active high: an enabled driver is driven high, a disabled one low. */
    assert(syringe_driver_logic_enable_level(1U, 1U) == 1U);
    assert(syringe_driver_logic_enable_level(0U, 1U) == 0U);
    /* Active low: an enabled driver is driven low, a disabled one high. */
    assert(syringe_driver_logic_enable_level(1U, 0U) == 0U);
    assert(syringe_driver_logic_enable_level(0U, 0U) == 1U);
}

static void test_direction_level_follows_the_polarity(void) {
    /* A high level is the positive direction. */
    assert(syringe_driver_logic_direction_level(1U, 1U) == 1U);
    assert(syringe_driver_logic_direction_level(0U, 1U) == 0U);
    /* A low level is the positive direction. */
    assert(syringe_driver_logic_direction_level(1U, 0U) == 0U);
    assert(syringe_driver_logic_direction_level(0U, 0U) == 1U);
}

static void test_any_non_zero_flag_is_true(void) {
    const uint8_t truthy[] = {1U, 2U, 255U};
    for (uint32_t first = 0U; first < sizeof(truthy); ++first) {
        for (uint32_t second = 0U; second < sizeof(truthy); ++second) {
            assert(syringe_driver_logic_enable_level(truthy[first], truthy[second]) == 1U);
            assert(syringe_driver_logic_direction_level(truthy[first], truthy[second]) == 1U);
        }
        assert(syringe_driver_logic_enable_level(truthy[first], 0U) == 0U);
        assert(syringe_driver_logic_enable_level(0U, truthy[first]) == 0U);
        assert(syringe_driver_logic_direction_level(truthy[first], 0U) == 0U);
        assert(syringe_driver_logic_direction_level(0U, truthy[first]) == 0U);
    }
}

static void test_result_is_a_level_not_the_flag(void) {
    /* The level is 0 or 1 whatever the flags are: it is written to a pin as is. */
    assert(syringe_driver_logic_enable_level(255U, 255U) == 1U);
    assert(syringe_driver_logic_direction_level(2U, 2U) == 1U);
    assert(syringe_driver_logic_enable_level(0U, 0U) == 1U);
}

int main(void) {
    test_enable_level_follows_the_polarity();
    test_direction_level_follows_the_polarity();
    test_any_non_zero_flag_is_true();
    test_result_is_a_level_not_the_flag();
    (void)printf("Syringe driver logic unit tests passed.\n");
    return 0;
}

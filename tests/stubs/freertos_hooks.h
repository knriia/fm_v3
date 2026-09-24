#ifndef FM_V3_TEST_FREERTOS_HOOKS_H
#define FM_V3_TEST_FREERTOS_HOOKS_H

#include <stdint.h>

void freertos_hooks_get_diagnostics(uint32_t *stack_overflow_count, uint32_t *malloc_failed_count);

#endif /* FM_V3_TEST_FREERTOS_HOOKS_H */

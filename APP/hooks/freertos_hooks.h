#ifndef FM_V3_FREERTOS_HOOKS_H
#define FM_V3_FREERTOS_HOOKS_H

#include <stdint.h>

void freertos_hooks_get_diagnostics(uint32_t *stack_overflows, uint32_t *malloc_failures);

#endif /* FM_V3_FREERTOS_HOOKS_H */

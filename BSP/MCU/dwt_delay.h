#ifndef FM_V3_DWT_DELAY_H
#define FM_V3_DWT_DELAY_H

#include <stdint.h>

/* Turns the DWT cycle counter on. Cortex-M7 keeps DWT locked until LAR gets the key; calling it again changes nothing.
 */
void dwt_delay_init(void);

/* Busy-waits at least `ns` nanoseconds by the cycle counter (after dwt_delay_init); only for the few microseconds that
 * a driver data sheet asks for. The wait is exact to the core clock and does not give up the CPU. */
void dwt_delay_ns(uint32_t ns);

#endif /* FM_V3_DWT_DELAY_H */

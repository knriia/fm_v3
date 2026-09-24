#ifndef FM_V3_ENCODER_H
#define FM_V3_ENCODER_H

#include <stdint.h>

void encoder_init(void);

int32_t encoder_ox_get_coordinate(void);
void encoder_ox_reset_coordinate(void);

int32_t encoder_oy_get_coordinate(void);
void encoder_oy_reset_coordinate(void);

int32_t encoder_oz_get_coordinate(void);
void encoder_oz_reset_coordinate(void);

#endif /* FM_V3_ENCODER_H */

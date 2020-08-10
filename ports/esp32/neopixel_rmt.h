#ifndef NEOPIXEL_RMT_H
#define NEOPIXEL_RMT_H
#include <stdint.h>

void pixel_init(void);
void showPixels(uint8_t  *pixels, uint16_t channel_in, uint16_t length);

#endif
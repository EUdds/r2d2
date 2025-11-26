#pragma once

#include <stdint.h>

#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH            320u
#endif

#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT           206u
#endif

void screen_init(void);
void screen_fill_color(uint16_t color);
void screen_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void screen_write_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pixels);

inline uint16_t screen_24bit_to_16bit_color(uint8_t r, uint8_t g, uint8_t b)
{
    // Scale the values rather than bitshift to preserve brightness
    uint16_t r5 = (uint16_t)((r * 31u) / 255u);
    uint16_t g6 = (uint16_t)((g * 63u) / 255u);
    uint16_t b5 = (uint16_t)((b * 31u) / 255u);
    return (uint16_t)((r5 << 11u) | (g6 << 5u) | b5);
}
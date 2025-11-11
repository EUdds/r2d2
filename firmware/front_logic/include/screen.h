#pragma once

#include <stdint.h>

void screen_init(void);
void screen_fill_color(uint16_t color);
void screen_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void screen_write_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pixels);

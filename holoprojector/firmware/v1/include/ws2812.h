#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "hardware/pio.h"

typedef enum
{
    HOLOPROJECTOR_STRIP = 0,
    FRONT_PSI_STRIP = 1,
    NUM_STRIPS = 2,
} ws2812_strip_E;

typedef struct 
{
    PIO pio;
    uint sm;
} ws2812_handle_t;

typedef struct
{
    uint32_t pin;
    uint32_t num_pixels;
    uint32_t frequency_hz;
    bool is_rgbw;
} ws2812_config_t;

ws2812_handle_t* init_ws2812(void);
void put_pixel(ws2812_strip_E strip, uint32_t pixel_grb);
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);

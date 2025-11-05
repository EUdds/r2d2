/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ws2812.h"
#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

static const ws2812_config_t ws2812_config[NUM_STRIPS] = {
        [HOLOPROJECTOR_STRIP] = {
                .pin = 28,
                .num_pixels = 16,
                .frequency_hz = 800000,
                .is_rgbw = false,
        },
        [FRONT_PSI_STRIP] = {
                .pin = 27,
                .num_pixels = 16,
                .frequency_hz = 800000,
                .is_rgbw = false,
        },
};

static ws2812_handle_t ws2812_handle[NUM_STRIPS] = {0};
static bool g_ws2812_initialized = false;

static inline ws2812_handle_t *ws2812_resolve_handle(ws2812_strip_E strip) {
    if (!g_ws2812_initialized) {
        return NULL;
    }
    if (strip >= NUM_STRIPS) {
        return NULL;
    }
    return &ws2812_handle[strip];
}

void put_pixel(ws2812_strip_E strip, uint32_t pixel_grb) {
    ws2812_handle_t* handle = ws2812_resolve_handle(strip);
    if (!handle) {
        return;
    }
    pio_sm_put_blocking(handle->pio, handle->sm, pixel_grb << 8u);
}

void fill_pixels(ws2812_strip_E strip, uint32_t pixel_grb) {
    ws2812_handle_t* handle = ws2812_resolve_handle(strip);
    const ws2812_config_t* config = &ws2812_config[strip];
    if (!handle || !config) {
        return;
    }
    for (uint16_t i = 0; i < config->num_pixels; ++i) {
        pio_sm_put_blocking(handle->pio, handle->sm, pixel_grb << 8u);
    }
}

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// void pattern_snakes(PIO pio, uint sm, uint len, uint t) {
//     for (uint i = 0; i < len; ++i) {
//         uint x = (i + (t >> 1)) % 64;
//         if (x < 10)
//             put_pixel(pio, sm, urgb_u32(0xff, 0, 0));
//         else if (x >= 15 && x < 25)
//             put_pixel(pio, sm, urgb_u32(0, 0xff, 0));
//         else if (x >= 30 && x < 40)
//             put_pixel(pio, sm, urgb_u32(0, 0, 0xff));
//         else
//             put_pixel(pio, sm, 0);
//     }
// }

// void pattern_random(PIO pio, uint sm, uint len, uint t) {
//     if (t % 8)
//         return;
//     for (uint i = 0; i < len; ++i)
//         put_pixel(pio, sm, rand());
// }

// void pattern_sparkle(PIO pio, uint sm, uint len, uint t) {
//     if (t % 8)
//         return;
//     for (uint i = 0; i < len; ++i)
//         put_pixel(pio, sm, rand() % 16 ? 0 : 0xffffffff);
// }

// void pattern_greys(PIO pio, uint sm, uint len, uint t) {
//     uint max = 100; // let's not draw too much current!
//     t %= max;
//     for (uint i = 0; i < len; ++i) {
//         put_pixel(pio, sm, t * 0x10101);
//         if (++t >= max) t = 0;
//     }
// }

// typedef void (*pattern)(PIO pio, uint sm, uint len, uint t);
// const struct {
//     pattern pat;
//     const char *name;
// } pattern_table[] = {
//         {pattern_snakes,  "Snakes!"},
//         {pattern_random,  "Random data"},
//         {pattern_sparkle, "Sparkles"},
//         {pattern_greys,   "Greys"},
// };

ws2812_handle_t* init_ws2812(void)
{
    bool success = true;
    for (int i = 0; i < NUM_STRIPS; ++i)
    {
        PIO pio;
        uint sm;
        uint offset;
        success &= pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, ws2812_config[i].pin, 1, true);
        hard_assert(success);
        ws2812_program_init(pio, sm, offset, ws2812_config[i].pin, ws2812_config[i].frequency_hz, ws2812_config[i].is_rgbw);
        ws2812_handle[i].pio = pio;
        ws2812_handle[i].sm = sm;
    }

    g_ws2812_initialized = true;

    


    return (ws2812_handle_t*)ws2812_handle;
}

ws2812_handle_t* ws2812_get_handle(ws2812_strip_E strip) {
    return ws2812_resolve_handle(strip);
}

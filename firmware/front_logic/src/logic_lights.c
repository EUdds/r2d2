#include "logic_lights.h"
#include "screen.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <pico/rand.h>

#define LED_MATRIX_ROWS       5
#define LED_MATRIX_COLS       8
#define LED_MATRIX_SPACING    15
#define LED_RADIUS            10

#define MATRIX_ANIMATION_DIVIDER 5 // 10Hz task / 5 = 2Hz animation update

#define LOGIC_LIGHTS_PERCENT_BASE 100U

#ifndef LOGIC_LIGHTS_BLUE_CHANCE_PERCENT
#define LOGIC_LIGHTS_BLUE_CHANCE_PERCENT 40U
#endif

#ifndef LOGIC_LIGHTS_WHITE_CHANCE_PERCENT
#define LOGIC_LIGHTS_WHITE_CHANCE_PERCENT 40U
#endif

#ifndef LOGIC_LIGHTS_BLACK_CHANCE_PERCENT
#define LOGIC_LIGHTS_BLACK_CHANCE_PERCENT 10U
#endif

#define LOGIC_LIGHTS_CONFIGURED_PERCENT_TOTAL (LOGIC_LIGHTS_BLUE_CHANCE_PERCENT + LOGIC_LIGHTS_WHITE_CHANCE_PERCENT + LOGIC_LIGHTS_BLACK_CHANCE_PERCENT)

#if LOGIC_LIGHTS_CONFIGURED_PERCENT_TOTAL > LOGIC_LIGHTS_PERCENT_BASE
#error "logic lights color selection percentages must be <= 100"
#endif

typedef enum
{
    SCREEN_ROTATION_0 = 0, // Wires down
    SCREEN_ROTATION_90 = 1,
    SCREEN_ROTATION_180 = 2, // Wires up
    SCREEN_ROTATION_270 = 3
} screen_rotation_E;

typedef struct
{
    screen_rotation_E rotation;
    uint16_t width;
    uint16_t height;
    int16_t x_offset;
    int16_t y_offset;
} screen_config_S;

static uint16_t pixelBuffer[SCREEN_HEIGHT][SCREEN_WIDTH] = {0};
static screen_config_S screen_config = {
    .rotation = SCREEN_ROTATION_180,
    .width = SCREEN_WIDTH,
    .height = SCREEN_HEIGHT,
    .x_offset = 0,
    .y_offset = 0
};

static const uint32_t matrix_colors[] = {0x0000FF, 0xFFFFFF, 0x000000}; // Blue, White, Off

static uint32_t led_matrix_colors[LED_MATRIX_ROWS][LED_MATRIX_COLS] = {0};

// ---------------------------------------------------------------------------
// Dirty region tracking (in screen / pixel-buffer coordinate space).
//
// Rather than blitting the full 320x206 buffer on every 100Hz tick, we track
// the bounding box of pixels that were actually modified.  Only that region is
// sent over SPI, saving ~98% of the pixel data on a typical animation step
// (one 21x21 LED circle vs. 320x206 full frame).
// ---------------------------------------------------------------------------
static bool     s_has_dirty = false;
static uint16_t s_dirty_x1  = SCREEN_WIDTH;
static uint16_t s_dirty_y1  = SCREEN_HEIGHT;
static uint16_t s_dirty_x2  = 0u;
static uint16_t s_dirty_y2  = 0u;

static void dirty_reset(void)
{
    s_has_dirty = false;
    s_dirty_x1  = SCREEN_WIDTH;
    s_dirty_y1  = SCREEN_HEIGHT;
    s_dirty_x2  = 0u;
    s_dirty_y2  = 0u;
}

static void dirty_include(uint16_t x, uint16_t y)
{
    if (x < s_dirty_x1) { s_dirty_x1 = x; }
    if (x > s_dirty_x2) { s_dirty_x2 = x; }
    if (y < s_dirty_y1) { s_dirty_y1 = y; }
    if (y > s_dirty_y2) { s_dirty_y2 = y; }
    s_has_dirty = true;
}

static void logic_lights_set_pixel_color(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= screen_config.width || y >= screen_config.height)
    {
        return;
    }
    switch (screen_config.rotation)
    {
    case SCREEN_ROTATION_0:
        // No rotation
        break;
    case SCREEN_ROTATION_90:
    {
        uint16_t temp = x;
        x = screen_config.width - 1u - y;
        y = temp;
        break;
    }
    case SCREEN_ROTATION_180:
        x = screen_config.width - 1u - x;
        y = screen_config.height - 1u - y;
        break;
    case SCREEN_ROTATION_270:
    {
        uint16_t temp = x;
        x = y;
        y = screen_config.height - 1u - temp;
        break;
    }
    default:
        break;
    }

    x += screen_config.x_offset;
    y += screen_config.y_offset;

    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT)
    {
        return;
    }

    dirty_include(x, y);
    pixelBuffer[y][x] = color;
}

/**
* Draw a circle of radius radius and color (r, g, b) at center position (x, y)
*/
static void logic_lights_draw_led(int32_t x, int32_t y, uint8_t radius, uint8_t r, uint8_t g, uint8_t b)
{
    int32_t minX = x - radius;
    int32_t maxX = x + radius;
    int32_t minY = y - radius;
    int32_t maxY = y + radius;

    if (maxX < 0 || maxY < 0)
    {
        return;
    }
    if (minX >= (int32_t)screen_config.width || minY >= (int32_t)screen_config.height)
    {
        return;
    }
    uint16_t color = screen_24bit_to_16bit_color(r, g, b);
    int16_t rsq = radius * radius;
    for (int16_t dy = -radius; dy <= radius; dy++)
    {
        for (int16_t dx = -radius; dx <= radius; dx++)
        {
            if (dx * dx + dy * dy <= rsq)
            {
                int32_t px = x + dx;
                int32_t py = y + dy;
                if (px >= 0 && px < (int32_t)screen_config.width && py >= 0 && py < (int32_t)screen_config.height)
                {
                    logic_lights_set_pixel_color((uint16_t)px, (uint16_t)py, color);
                }
            }
        }
    }
}

/**
* @brief Fill the screen with a matrix of LEDs
*/
static void logic_lights_create_led_matrix(uint8_t rows, uint8_t cols, uint8_t spacing, uint8_t radius, uint32_t* colors)
{
    if (rows == 0 || cols == 0)
    {
        return;
    }

    const int16_t padding = 10;
    int32_t startX = padding + radius;
    int32_t startY = padding + radius;

    for (uint8_t row = 0; row < rows; row++)
    {
        for (uint8_t col = 0; col < cols; col++)
        {
            int32_t x = startX + (int32_t)col * (spacing + radius * 2);
            int32_t y = startY + (int32_t)row * (spacing + radius * 2);
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            if (colors != NULL)
            {
                uint32_t color = colors[row * cols + col];
                r = (uint8_t)((color >> 16) & 0xFF);
                g = (uint8_t)((color >> 8) & 0xFF);
                b = (uint8_t)(color & 0xFF);
            }
            logic_lights_draw_led(x, y, radius, r, g, b);
        }
    }
}

/**
* @brief Redraw a single LED at (row, col) using its current color from led_matrix_colors.
*        Marks the affected screen region dirty.  Only call when the color has actually
*        changed — avoids redrawing the full matrix for a one-LED update.
*/
static void logic_lights_redraw_led(uint8_t row, uint8_t col)
{
    const int16_t padding = 10;
    int32_t x = (int32_t)(padding + LED_RADIUS) + (int32_t)col * (LED_MATRIX_SPACING + LED_RADIUS * 2);
    int32_t y = (int32_t)(padding + LED_RADIUS) + (int32_t)row * (LED_MATRIX_SPACING + LED_RADIUS * 2);
    uint32_t color32 = led_matrix_colors[row][col];
    uint8_t r = (uint8_t)((color32 >> 16) & 0xFF);
    uint8_t g = (uint8_t)((color32 >> 8) & 0xFF);
    uint8_t b = (uint8_t)(color32 & 0xFF);
    logic_lights_draw_led(x, y, LED_RADIUS, r, g, b);
}

static void logic_lights_write_led_color(uint8_t row, uint8_t col, uint32_t color)
{
    if (row >= LED_MATRIX_ROWS || col >= LED_MATRIX_COLS)
    {
        return;
    }
    led_matrix_colors[row][col] = color;
}

static bool logic_lights_select_weighted_color(uint32_t* next_color)
{
    if (next_color == NULL)
    {
        return false;
    }

    const uint32_t blue_threshold = LOGIC_LIGHTS_BLUE_CHANCE_PERCENT;
    const uint32_t white_threshold = blue_threshold + LOGIC_LIGHTS_WHITE_CHANCE_PERCENT;
    const uint32_t black_threshold = white_threshold + LOGIC_LIGHTS_BLACK_CHANCE_PERCENT;

    const uint32_t roll = get_rand_32() % LOGIC_LIGHTS_PERCENT_BASE;

    if (roll < blue_threshold)
    {
        *next_color = matrix_colors[0];
        return true;
    }
    if (roll < white_threshold)
    {
        *next_color = matrix_colors[1];
        return true;
    }
    if (roll < black_threshold)
    {
        *next_color = matrix_colors[2];
        return true;
    }

    return false;
}

void logic_lights_init(void)
{
    screen_init();
    uint32_t init_color = 0x0000FF; // Blue
    for (uint8_t row = 0; row < LED_MATRIX_ROWS; row++)
    {
        for (uint8_t col = 0; col < LED_MATRIX_COLS; col++)
        {
            led_matrix_colors[row][col] = init_color;
        }
    }
    logic_lights_create_led_matrix(LED_MATRIX_ROWS, LED_MATRIX_COLS, LED_MATRIX_SPACING, LED_RADIUS, led_matrix_colors);
}

void logic_lights_run_animation_step(void)
{
    static uint8_t animation_step_counter = 0;
    animation_step_counter++;
    if (animation_step_counter == (MATRIX_ANIMATION_DIVIDER - 1))
    {
        // Select a random LED and change its color
        uint8_t row = (uint8_t)(get_rand_32() % LED_MATRIX_ROWS);
        uint8_t col = (uint8_t)(get_rand_32() % LED_MATRIX_COLS);
        uint32_t color = 0;
        if (logic_lights_select_weighted_color(&color))
        {
            logic_lights_write_led_color(row, col, color);
            logic_lights_redraw_led(row, col); // Redraw only the changed LED
        }
        animation_step_counter = 0;
    }
}

void logic_lights_update_screen(void)
{
    if (!s_has_dirty)
    {
        return;
    }

    // Blit only the dirty bounding box, one row at a time.  Each row of
    // pixelBuffer is contiguous in memory so we can pass a pointer directly
    // into the buffer without copying.
    uint16_t w = s_dirty_x2 - s_dirty_x1 + 1u;
    for (uint16_t y = s_dirty_y1; y <= s_dirty_y2; y++)
    {
        screen_write_rect(s_dirty_x1, y, w, 1u, &pixelBuffer[y][s_dirty_x1]);
    }

    dirty_reset();
}

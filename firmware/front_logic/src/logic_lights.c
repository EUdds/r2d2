#include "logic_lights.h"
#include "screen.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
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

static uint16_t pixelBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
static screen_config_S screen_config = {
    .rotation = SCREEN_ROTATION_180,
    .width = SCREEN_WIDTH,
    .height = SCREEN_HEIGHT,
    .x_offset = 0,
    .y_offset = 0
};

static const uint32_t matrix_colors[] = {0x0000FF, 0xFFFFFF, 0x000000}; // Blue, White, Off

// Per-screen LED matrix state — each panel gets an independent color grid.
#define MAX_SCREENS 2u
static uint32_t led_matrix_colors[MAX_SCREENS][LED_MATRIX_ROWS][LED_MATRIX_COLS];

static void logic_lights_set_pixel_color(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= screen_config.width || y >= screen_config.height)
    {
        return;
    }
    switch (screen_config.rotation)
    {
    case SCREEN_ROTATION_0:
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

    pixelBuffer[y][x] = color;
}

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

static void logic_lights_create_led_matrix(uint8_t rows, uint8_t cols, uint8_t spacing, uint8_t radius, uint32_t *colors)
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

static bool logic_lights_select_weighted_color(uint32_t *next_color)
{
    if (next_color == NULL)
    {
        return false;
    }

    const uint32_t blue_threshold  = LOGIC_LIGHTS_BLUE_CHANCE_PERCENT;
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
    uint8_t num_panels = screen_num_panels();
    for (uint8_t panel = 0; panel < num_panels && panel < MAX_SCREENS; panel++)
    {
        for (uint8_t row = 0; row < LED_MATRIX_ROWS; row++)
        {
            for (uint8_t col = 0; col < LED_MATRIX_COLS; col++)
            {
                led_matrix_colors[panel][row][col] = init_color;
            }
        }
    }
}

void logic_lights_run_animation_step(void)
{
    static uint8_t animation_step_counter = 0;
    animation_step_counter++;
    if (animation_step_counter < (MATRIX_ANIMATION_DIVIDER - 1))
    {
        return;
    }
    animation_step_counter = 0;

    // Update one independently-chosen random dot on each panel.
    uint8_t num_panels = screen_num_panels();
    for (uint8_t panel = 0; panel < num_panels && panel < MAX_SCREENS; panel++)
    {
        uint8_t row = (uint8_t)(get_rand_32() % LED_MATRIX_ROWS);
        uint8_t col = (uint8_t)(get_rand_32() % LED_MATRIX_COLS);
        uint32_t color = 0;
        if (logic_lights_select_weighted_color(&color))
        {
            led_matrix_colors[panel][row][col] = color;
        }
    }
}

void logic_lights_update_screen(void)
{
    uint8_t num_panels = screen_num_panels();
    for (uint8_t panel = 0; panel < num_panels && panel < MAX_SCREENS; panel++)
    {
        memset(pixelBuffer, 0, sizeof(pixelBuffer));
        logic_lights_create_led_matrix(LED_MATRIX_ROWS, LED_MATRIX_COLS, LED_MATRIX_SPACING, LED_RADIUS,
                                       (uint32_t *)led_matrix_colors[panel]);
        screen_write_rect_panel(panel, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, (const uint16_t *)pixelBuffer);
    }
}

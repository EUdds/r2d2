#include "screen.h"

#include "hardware.h"
#include "st7789.h"
#include <hardware/gpio.h>
#if PCB_REV < 2
#include <hardware/spi.h>
#endif
#include <stdbool.h>

#define SCREEN_SPI_BAUDRATE     (10u * 1000u * 1000u)

#ifndef SCREEN_MADCTL_VALUE
#define SCREEN_MADCTL_VALUE     (ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_ML)
#endif

// ============================================================================
// Per-revision display instances
// ============================================================================

#if PCB_REV >= 2

#ifndef SCREEN0_X_OFFSET
#define SCREEN0_X_OFFSET        0u
#endif
#ifndef SCREEN0_Y_OFFSET
#define SCREEN0_Y_OFFSET        8u
#endif
#ifndef SCREEN1_X_OFFSET
#define SCREEN1_X_OFFSET        0u
#endif
#ifndef SCREEN1_Y_OFFSET
#define SCREEN1_Y_OFFSET        0u
#endif

#define NUMBER_OF_SCREENS       2U

static st7789_t s_panels[NUMBER_OF_SCREENS];

#else  // PCB_REV < 2

#ifndef SCREEN_X_OFFSET
#define SCREEN_X_OFFSET         0u
#endif
#ifndef SCREEN_Y_OFFSET
#define SCREEN_Y_OFFSET         0u
#endif

static st7789_t s_display;

#endif  // PCB_REV >= 2

// ============================================================================
// Helpers
// ============================================================================

static bool screen_validate_bounds(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    if (width == 0u || height == 0u)                  return false;
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT)      return false;
    if ((uint32_t)x + width  > SCREEN_WIDTH)          return false;
    if ((uint32_t)y + height > SCREEN_HEIGHT)         return false;
    return true;
}

// ============================================================================
// Public API
// ============================================================================

void screen_init(void)
{
#if PCB_REV >= 2
    board_screen_spi_init(SCREEN_SPI_BAUDRATE);
#else
    spi_init(SCREEN_SPI, SCREEN_SPI_BAUDRATE);
    gpio_set_function(SPI0_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_CLK_PIN,  GPIO_FUNC_SPI);
#endif

#if PCB_REV >= 2
    s_panels[0] = (st7789_t){
        .spi       = SCREEN_SPI,
        .cs_pin    = SCREEN0_CS_PIN,
        .dc_pin    = SCREEN_DC_PIN,
        .reset_pin = SCREEN0_RESET_PIN,
        .x_off     = SCREEN0_X_OFFSET,
        .y_off     = SCREEN0_Y_OFFSET,
        .madctl    = SCREEN_MADCTL_VALUE,
    };
    s_panels[1] = (st7789_t){
        .spi       = SCREEN_SPI,
        .cs_pin    = SCREEN1_CS_PIN,
        .dc_pin    = SCREEN_DC_PIN,
        .reset_pin = SCREEN1_RESET_PIN,
        .x_off     = SCREEN1_X_OFFSET,
        .y_off     = SCREEN1_Y_OFFSET,
        .madctl    = SCREEN_MADCTL_VALUE,
    };
    st7789_init(&s_panels[0]);
    st7789_init(&s_panels[1]);

    gpio_init(SCREEN0_BKLT_PIN);
    gpio_set_dir(SCREEN0_BKLT_PIN, GPIO_OUT);
    gpio_put(SCREEN0_BKLT_PIN, 1);

    gpio_init(SCREEN1_BKLT_PIN);
    gpio_set_dir(SCREEN1_BKLT_PIN, GPIO_OUT);
    gpio_put(SCREEN1_BKLT_PIN, 1);

#else  // PCB_REV < 2
    s_display = (st7789_t){
        .spi       = SCREEN_SPI,
        .cs_pin    = SPI0_CS_PIN,
        .dc_pin    = SCREEN_DC_PIN,
        .reset_pin = SCREEN_RESET_PIN,
        .x_off     = SCREEN_X_OFFSET,
        .y_off     = SCREEN_Y_OFFSET,
        .madctl    = SCREEN_MADCTL_VALUE,
    };
    st7789_init(&s_display);
#endif

    screen_fill_color(0x0000);
}

void screen_fill_color(uint16_t color)
{
    screen_fill_rect(0u, 0u, SCREEN_WIDTH, SCREEN_HEIGHT, color);
}

void screen_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    if (!screen_validate_bounds(x, y, width, height)) {
        return;
    }
#if PCB_REV >= 2
    for (uint8_t i = 0u; i < 2u; i++) {
        st7789_fill_rect(&s_panels[i], x, y, width, height, color);
    }
#else
    st7789_fill_rect(&s_display, x, y, width, height, color);
#endif
}

void screen_write_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pixels)
{
    if (pixels == NULL || !screen_validate_bounds(x, y, width, height)) {
        return;
    }
#if PCB_REV >= 2
    for (uint8_t i = 0u; i < 2u; i++) {
        st7789_write_rect(&s_panels[i], x, y, width, height, pixels);
    }
#else
    st7789_write_rect(&s_display, x, y, width, height, pixels);
#endif
}

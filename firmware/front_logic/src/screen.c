#include "screen.h"

#include "hardware.h"
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SCREEN_SPI              spi0
#define SCREEN_SPI_BAUDRATE     (40u * 1000u * 1000u)
#define SCREEN_SPI_CPOL         SPI_CPOL_0
#define SCREEN_SPI_CPHA         SPI_CPHA_0
#define SCREEN_SPI_ORDER        SPI_MSB_FIRST


#ifndef SCREEN_X_OFFSET
#define SCREEN_X_OFFSET         0u
#endif

#ifndef SCREEN_Y_OFFSET
#define SCREEN_Y_OFFSET         0u
#endif

#define SCREEN_PIXEL_CHUNK      64u

#define ST7789_CMD_SWRESET      0x01
#define ST7789_CMD_SLPOUT       0x11
#define ST7789_CMD_NORON        0x13
#define ST7789_CMD_INVON        0x21
#define ST7789_CMD_DISPON       0x29
#define ST7789_CMD_CASET        0x2A
#define ST7789_CMD_RASET        0x2B
#define ST7789_CMD_RAMWR        0x2C
#define ST7789_CMD_MADCTL       0x36
#define ST7789_CMD_COLMOD       0x3A
#define ST7789_CMD_PORCTRL      0xB2
#define ST7789_CMD_GCTRL        0xB7
#define ST7789_CMD_VCOMS        0xBB
#define ST7789_CMD_LCMCTRL      0xC0
#define ST7789_CMD_VDVVRHEN     0xC2
#define ST7789_CMD_VRHS         0xC3
#define ST7789_CMD_VDVS         0xC4
#define ST7789_CMD_FRCTRL2      0xC6
#define ST7789_CMD_PWCTRL1      0xD0
#define ST7789_CMD_PVGAMCTRL    0xE0
#define ST7789_CMD_NVGAMCTRL    0xE1

#define ST7789_MADCTL_MY        0x80
#define ST7789_MADCTL_MX        0x40
#define ST7789_MADCTL_MV        0x20
#define ST7789_MADCTL_RGB       0x00
#define ST7789_MADCTL_BGR       0x08

#ifndef SCREEN_MADCTL_VALUE
#define SCREEN_MADCTL_VALUE     (ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB)
#endif

static uint screen_current_data_bits = 8u;


static inline void screen_select(void)
{
    gpio_put(SPI0_CS_PIN, 0);
}

static inline void screen_deselect(void)
{
    gpio_put(SPI0_CS_PIN, 1);
}

static void screen_spi_set_data_bits(uint data_bits)
{
    if (screen_current_data_bits == data_bits) {
        return;
    }
    spi_set_format(SCREEN_SPI, data_bits, SCREEN_SPI_CPOL, SCREEN_SPI_CPHA, SCREEN_SPI_ORDER);
    screen_current_data_bits = data_bits;
}

static void screen_write_command_with_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    screen_spi_set_data_bits(8u);
    screen_select();
    gpio_put(SCREEN_DC_PIN, 0);
    spi_write_blocking(SCREEN_SPI, &cmd, 1);
    if (len > 0u && data != NULL) {
        gpio_put(SCREEN_DC_PIN, 1);
        spi_write_blocking(SCREEN_SPI, data, len);
    }
    screen_deselect();
}

static inline void screen_write_command(uint8_t cmd)
{
    screen_write_command_with_data(cmd, NULL, 0u);
}

static void screen_write_pixels(const uint16_t *pixels, size_t count)
{
    if (pixels == NULL || count == 0u) {
        return;
    }
    screen_spi_set_data_bits(16u);
    screen_select();
    gpio_put(SCREEN_DC_PIN, 1);
    spi_write16_blocking(SCREEN_SPI, pixels, count);
    screen_deselect();
    screen_spi_set_data_bits(8u);
}

static bool screen_validate_bounds(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    if (width == 0u || height == 0u) {
        return false;
    }
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
        return false;
    }
    if ((uint32_t)x + width > SCREEN_WIDTH) {
        return false;
    }
    if ((uint32_t)y + height > SCREEN_HEIGHT) {
        return false;
    }
    return true;
}

static void screen_set_address_window(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    uint16_t x0 = x + (uint16_t)SCREEN_X_OFFSET;
    uint16_t y0 = y + (uint16_t)SCREEN_Y_OFFSET;
    uint16_t x1 = x0 + width - 1u;
    uint16_t y1 = y0 + height - 1u;

    const uint8_t caset[] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF),
    };
    const uint8_t raset[] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF),
    };

    screen_write_command_with_data(ST7789_CMD_CASET, caset, sizeof(caset));
    screen_write_command_with_data(ST7789_CMD_RASET, raset, sizeof(raset));
}

static void screen_run_init_sequence(void)
{
    static const uint8_t porch_ctrl[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    static const uint8_t gctrl[] = {0x35};
    static const uint8_t vcoms[] = {0x35};
    static const uint8_t lcmctrl[] = {0x2C};
    static const uint8_t vdvvrhen[] = {0x01};
    static const uint8_t vrhs[] = {0x11};
    static const uint8_t vdvs[] = {0x20};
    static const uint8_t frctrl2[] = {0x0F};
    static const uint8_t pwctrl1[] = {0xA4, 0xA1};
    static const uint8_t colmod[] = {0x05};
    static const uint8_t madctl[] = {0x70};
    static const uint8_t pvgam[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    static const uint8_t nvgam[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};

    screen_write_command(ST7789_CMD_SWRESET);
    sleep_ms(150);

    screen_write_command(ST7789_CMD_SLPOUT);
    sleep_ms(120);

    screen_write_command_with_data(ST7789_CMD_COLMOD, colmod, sizeof(colmod));
    screen_write_command_with_data(ST7789_CMD_PORCTRL, porch_ctrl, sizeof(porch_ctrl));
    screen_write_command_with_data(ST7789_CMD_GCTRL, gctrl, sizeof(gctrl));
    screen_write_command_with_data(ST7789_CMD_VCOMS, vcoms, sizeof(vcoms));
    screen_write_command_with_data(ST7789_CMD_LCMCTRL, lcmctrl, sizeof(lcmctrl));
    screen_write_command_with_data(ST7789_CMD_VDVVRHEN, vdvvrhen, sizeof(vdvvrhen));
    
    screen_write_command_with_data(ST7789_CMD_VRHS, vrhs, sizeof(vrhs));
    screen_write_command_with_data(ST7789_CMD_VDVS, vdvs, sizeof(vdvs));
    screen_write_command_with_data(ST7789_CMD_FRCTRL2, frctrl2, sizeof(frctrl2));
    screen_write_command_with_data(ST7789_CMD_PWCTRL1, pwctrl1, sizeof(pwctrl1));
    screen_write_command_with_data(ST7789_CMD_MADCTL, madctl, sizeof(madctl));
    screen_write_command_with_data(ST7789_CMD_PVGAMCTRL, pvgam, sizeof(pvgam));
    screen_write_command_with_data(ST7789_CMD_NVGAMCTRL, nvgam, sizeof(nvgam));

    screen_set_address_window(0u, 0u, SCREEN_WIDTH, SCREEN_HEIGHT);

    screen_write_command(ST7789_CMD_INVON);
    sleep_ms(10);
    screen_write_command(ST7789_CMD_NORON);
    sleep_ms(10);
    screen_write_command(ST7789_CMD_DISPON);
    sleep_ms(100);
}

void screen_init(void)
{
    spi_init(SCREEN_SPI, SCREEN_SPI_BAUDRATE);
    spi_set_format(SCREEN_SPI, 8u, SCREEN_SPI_CPOL, SCREEN_SPI_CPHA, SCREEN_SPI_ORDER);
    screen_current_data_bits = 8u;

    gpio_set_function(SPI0_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_CLK_PIN, GPIO_FUNC_SPI);

    gpio_init(SPI0_CS_PIN);
    gpio_set_dir(SPI0_CS_PIN, GPIO_OUT);
    gpio_put(SPI0_CS_PIN, 1);

    gpio_init(SCREEN_DC_PIN);
    gpio_set_dir(SCREEN_DC_PIN, GPIO_OUT);
    gpio_put(SCREEN_DC_PIN, 1);

    screen_run_init_sequence();

    screen_fill_color(0x00);
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

    uint16_t buffer[SCREEN_PIXEL_CHUNK];
    for (size_t i = 0; i < SCREEN_PIXEL_CHUNK; ++i) {
        buffer[i] = color;
    }

    screen_set_address_window(x, y, width, height);
    screen_write_command(ST7789_CMD_RAMWR);

    size_t remaining = (size_t)width * (size_t)height;
    while (remaining > 0u) {
        size_t chunk = remaining > SCREEN_PIXEL_CHUNK ? SCREEN_PIXEL_CHUNK : remaining;
        screen_write_pixels(buffer, chunk);
        remaining -= chunk;
    }
}

void screen_write_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pixels)
{
    if (pixels == NULL || !screen_validate_bounds(x, y, width, height)) {
        return;
    }

    screen_set_address_window(x, y, width, height);
    screen_write_command(ST7789_CMD_RAMWR);
    screen_write_pixels(pixels, (size_t)width * (size_t)height);
}

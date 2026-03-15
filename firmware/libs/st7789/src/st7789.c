#include "st7789.h"

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/time.h>
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// Registers
// ============================================================================

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

#define PIXEL_CHUNK             64u

// ============================================================================
// Low-level SPI helpers
// ============================================================================

static inline void cs_select(const st7789_t *dev)   { gpio_put(dev->cs_pin, 0); }
static inline void cs_deselect(const st7789_t *dev) { gpio_put(dev->cs_pin, 1); }

static void write_command(const st7789_t *dev, uint8_t cmd, const uint8_t *data, size_t len)
{
    spi_set_format(dev->spi, 8u, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    cs_select(dev);
    gpio_put(dev->dc_pin, 0);
    spi_write_blocking(dev->spi, &cmd, 1);
    if (len > 0u && data != NULL) {
        gpio_put(dev->dc_pin, 1);
        spi_write_blocking(dev->spi, data, len);
    }
    cs_deselect(dev);
}

static void write_pixels(const st7789_t *dev, const uint16_t *pixels, size_t count)
{
    spi_set_format(dev->spi, 16u, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    cs_select(dev);
    gpio_put(dev->dc_pin, 1);
    spi_write16_blocking(dev->spi, pixels, count);
    cs_deselect(dev);
}

static void set_address_window(const st7789_t *dev,
                               uint16_t x, uint16_t y,
                               uint16_t width, uint16_t height)
{
    uint16_t x0 = x + dev->x_off;
    uint16_t y0 = y + dev->y_off;
    uint16_t x1 = x0 + width  - 1u;
    uint16_t y1 = y0 + height - 1u;

    const uint8_t caset[] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFFu),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFFu),
    };
    const uint8_t raset[] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFFu),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFFu),
    };

    write_command(dev, ST7789_CMD_CASET, caset, sizeof(caset));
    write_command(dev, ST7789_CMD_RASET, raset, sizeof(raset));
}

// ============================================================================
// Initialization sequence
// ============================================================================

static void run_init_sequence(const st7789_t *dev)
{
    static const uint8_t porch_ctrl[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    static const uint8_t gctrl[]      = {0x35};
    static const uint8_t vcoms[]      = {0x35};
    static const uint8_t lcmctrl[]    = {0x2C};
    static const uint8_t vdvvrhen[]   = {0x01};
    static const uint8_t vrhs[]       = {0x11};
    static const uint8_t vdvs[]       = {0x20};
    static const uint8_t frctrl2[]    = {0x0F};
    static const uint8_t pwctrl1[]    = {0xA4, 0xA1};
    static const uint8_t colmod[]     = {0x05};  // 16-bit RGB565
    static const uint8_t pvgam[]      = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B,
                                         0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    static const uint8_t nvgam[]      = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C,
                                         0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    const uint8_t madctl[]            = {dev->madctl};

    write_command(dev, ST7789_CMD_SWRESET, NULL, 0u);
    sleep_ms(150);

    write_command(dev, ST7789_CMD_SLPOUT, NULL, 0u);
    sleep_ms(120);

    write_command(dev, ST7789_CMD_COLMOD,    colmod,     sizeof(colmod));
    write_command(dev, ST7789_CMD_PORCTRL,   porch_ctrl, sizeof(porch_ctrl));
    write_command(dev, ST7789_CMD_GCTRL,     gctrl,      sizeof(gctrl));
    write_command(dev, ST7789_CMD_VCOMS,     vcoms,      sizeof(vcoms));
    write_command(dev, ST7789_CMD_LCMCTRL,   lcmctrl,    sizeof(lcmctrl));
    write_command(dev, ST7789_CMD_VDVVRHEN,  vdvvrhen,   sizeof(vdvvrhen));
    write_command(dev, ST7789_CMD_VRHS,      vrhs,       sizeof(vrhs));
    write_command(dev, ST7789_CMD_VDVS,      vdvs,       sizeof(vdvs));
    write_command(dev, ST7789_CMD_FRCTRL2,   frctrl2,    sizeof(frctrl2));
    write_command(dev, ST7789_CMD_PWCTRL1,   pwctrl1,    sizeof(pwctrl1));
    write_command(dev, ST7789_CMD_MADCTL,    madctl,     sizeof(madctl));
    write_command(dev, ST7789_CMD_PVGAMCTRL, pvgam,      sizeof(pvgam));
    write_command(dev, ST7789_CMD_NVGAMCTRL, nvgam,      sizeof(nvgam));

    write_command(dev, ST7789_CMD_INVON,  NULL, 0u);
    sleep_ms(10);
    write_command(dev, ST7789_CMD_NORON,  NULL, 0u);
    sleep_ms(10);
    write_command(dev, ST7789_CMD_DISPON, NULL, 0u);
    sleep_ms(100);
}

// ============================================================================
// Public API
// ============================================================================

void st7789_init(st7789_t *dev)
{
    gpio_init(dev->cs_pin);
    gpio_set_dir(dev->cs_pin, GPIO_OUT);
    gpio_put(dev->cs_pin, 1);

    gpio_init(dev->dc_pin);
    gpio_set_dir(dev->dc_pin, GPIO_OUT);
    gpio_put(dev->dc_pin, 1);

    gpio_init(dev->reset_pin);
    gpio_set_dir(dev->reset_pin, GPIO_OUT);
    gpio_put(dev->reset_pin, 1);

    sleep_ms(10);

    run_init_sequence(dev);
}

void st7789_fill_rect(st7789_t *dev, uint16_t x, uint16_t y,
                      uint16_t width, uint16_t height, uint16_t color)
{
    uint16_t buf[PIXEL_CHUNK];
    for (size_t i = 0u; i < PIXEL_CHUNK; i++) {
        buf[i] = color;
    }

    set_address_window(dev, x, y, width, height);
    write_command(dev, ST7789_CMD_RAMWR, NULL, 0u);

    size_t remaining = (size_t)width * (size_t)height;
    while (remaining > 0u) {
        size_t chunk = remaining > PIXEL_CHUNK ? PIXEL_CHUNK : remaining;
        write_pixels(dev, buf, chunk);
        remaining -= chunk;
    }
}

void st7789_write_rect(st7789_t *dev, uint16_t x, uint16_t y,
                       uint16_t width, uint16_t height, const uint16_t *pixels)
{
    if (pixels == NULL) {
        return;
    }
    set_address_window(dev, x, y, width, height);
    write_command(dev, ST7789_CMD_RAMWR, NULL, 0u);
    write_pixels(dev, pixels, (size_t)width * (size_t)height);
}

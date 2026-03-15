#pragma once

#include <hardware/spi.h>
#include <stdint.h>

// MADCTL orientation flags
#define ST7789_MADCTL_MY    0x80u  // Row address order
#define ST7789_MADCTL_MX    0x40u  // Column address order
#define ST7789_MADCTL_MV    0x20u  // Row/column exchange
#define ST7789_MADCTL_ML    0x10u  // Vertical refresh order
#define ST7789_MADCTL_RGB   0x00u  // RGB order
#define ST7789_MADCTL_BGR   0x08u  // BGR order

typedef struct {
    spi_inst_t *spi;       // SPI instance — caller must call spi_init() before st7789_init()
    uint        cs_pin;    // Chip select (active low)
    uint        dc_pin;    // Data/command select
    uint        reset_pin; // Reset (active low, held high during operation)
    uint16_t    x_off;     // X pixel offset applied to every address window
    uint16_t    y_off;     // Y pixel offset applied to every address window
    uint8_t     madctl;    // MADCTL value — controls display orientation
} st7789_t;

// Configure GPIO pins for cs_pin, dc_pin, and reset_pin, then run the
// ST7789 initialization sequence. The caller is responsible for calling
// spi_init() and gpio_set_function() for the CLK and MOSI pins beforehand.
void st7789_init(st7789_t *dev);

// Fill a rectangle with a solid RGB565 color.
void st7789_fill_rect(st7789_t *dev, uint16_t x, uint16_t y,
                      uint16_t width, uint16_t height, uint16_t color);

// Write an RGB565 pixel buffer to a rectangle.
void st7789_write_rect(st7789_t *dev, uint16_t x, uint16_t y,
                       uint16_t width, uint16_t height, const uint16_t *pixels);

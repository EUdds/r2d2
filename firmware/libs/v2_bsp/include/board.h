#ifndef BOARD_H
#define BOARD_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

/**
 * @file board.h
 * @brief Board Support Package for custom RP2354 board
 *
 * Hardware abstraction layer providing pin definitions and
 * initialization functions for the board peripherals.
 */

// ============================================================================
// LED Pin Definitions
// ============================================================================

#define LED_RED_PIN     12
#define LED_GREEN_PIN   13
#define LED_BLUE_PIN    14

// ============================================================================
// LED Control Macros
// ============================================================================
// Active-low LEDs (0/false = ON, 1/true = OFF)

#define LED_RED_ON()     gpio_put(LED_RED_PIN, 0)
#define LED_RED_OFF()    gpio_put(LED_RED_PIN, 1)
#define LED_RED_TOGGLE() gpio_xor_mask(1u << LED_RED_PIN)

#define LED_GREEN_ON()     gpio_put(LED_GREEN_PIN, 0)
#define LED_GREEN_OFF()    gpio_put(LED_GREEN_PIN, 1)
#define LED_GREEN_TOGGLE() gpio_xor_mask(1u << LED_GREEN_PIN)

#define LED_BLUE_ON()     gpio_put(LED_BLUE_PIN, 0)
#define LED_BLUE_OFF()    gpio_put(LED_BLUE_PIN, 1)
#define LED_BLUE_TOGGLE() gpio_xor_mask(1u << LED_BLUE_PIN)

// ============================================================================
// MCP2515 CAN Controller Pin Definitions
// ============================================================================

// SPI Interface
#define CAN_MOSI_PIN    31
#define CAN_MISO_PIN    32
#define CAN_CLK_PIN     34
#define CAN_CS_PIN      33  // Chip select (active-low)

// Control Pins
#define CAN_RESETN_PIN  40  // Active-low reset

// Interrupt Pin
#define CAN_INTN_PIN    37  // Active-low interrupt

// RX Buffer Full Pins (optional, can be used as interrupts or GPIO)
#define CAN_RX0BFN_PIN  39  // RX Buffer 0 Full (active-low)
#define CAN_RX1BFN_PIN  38  // RX Buffer 1 Full (active-low)

// TX Request-to-Send Pins (optional, can be used to trigger TX or as GPIO)
#define CAN_TX0RTSN_PIN 36  // TX Buffer 0 Request-to-Send (active-low)
#define CAN_TX1RTSN_PIN 47  // TX Buffer 1 Request-to-Send (active-low)
#define CAN_TX2RTSN_PIN 46  // TX Buffer 2 Request-to-Send (active-low)

// ============================================================================
// Screen Pin Definitions
// ============================================================================

#define SCREEN_CLK_PIN      (26)
#define SCREEN_MOSI_PIN     (27)
#define SCREEN_DC_PIN       (4)
#define SCREEN0_CS_PIN      (25)
#define SCREEN1_CS_PIN      (29)

#define SCREEN0_RESET_PIN   (5)
#define SCREEN1_RESET_PIN   (6)

#define SCREEN0_BKLT_PIN    (7)
#define SCREEN1_BKLT_PIN    (28)


// ============================================================================
// Board Initialization Functions
// ============================================================================

/**
 * @brief Initialize all board peripherals
 *
 * Initializes GPIOs, clocks, and other board-specific hardware.
 * Call this function at the start of main() before using any peripherals.
 */
void board_init(void);

/**
 * @brief Initialize all LED pins
 *
 * Configures LED GPIO pins as outputs and turns them off.
 */
void board_led_init(void);

/**
 * @brief Set RGB LED color
 *
 * @param red Red LED state (true = on, false = off)
 * @param green Green LED state (true = on, false = off)
 * @param blue Blue LED state (true = on, false = off)
 */
void board_led_set_rgb(bool red, bool green, bool blue);

/**
 * @brief Turn off all LEDs
 */
void board_led_all_off(void);

/**
 * @brief Turn on all LEDs
 */
void board_led_all_on(void);

/**
 * @brief Initialize MCP2515 CAN controller GPIO pins
 *
 * Configures control and interrupt pins for the MCP2515:
 * - RESETN: Output, set HIGH (device active)
 * - TX0/1/2RTSN: Outputs, set HIGH (request-to-send inactive)
 * - INTN: Input with pull-up (interrupt line)
 * - RX0/1BFN: Inputs with pull-up (buffer full indicators)
 *
 * Note: SPI pins should be initialized separately by the SPI peripheral driver
 */
void board_can_init(void);

/**
 * @brief Initialize the screen SPI bus (SPI1) and its GPIO pins
 *
 * Calls spi_init() and configures the CLK and MOSI pins for SPI function.
 * CS, DC, and reset pins are left to the display driver.
 *
 * @param baudrate Desired SPI clock frequency in Hz
 * @return Actual baudrate set by the SPI peripheral
 */
uint board_screen_spi_init(uint32_t baudrate);

#endif // BOARD_H

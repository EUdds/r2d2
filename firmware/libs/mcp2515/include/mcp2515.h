#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

/**
 * @file mcp2515.h
 * @brief MCP2515 CAN Controller Driver
 *
 * Simple driver for the MCP2515 standalone CAN controller.
 * Supports basic operations: init, reset, configuration, and message transmission.
 */

// MCP2515 SPI Commands
#define MCP2515_CMD_RESET       0xC0
#define MCP2515_CMD_READ        0x03
#define MCP2515_CMD_WRITE       0x02
#define MCP2515_CMD_RTS         0x80  // Request to Send
#define MCP2515_CMD_READ_STATUS 0xA0
#define MCP2515_CMD_BIT_MODIFY  0x05

// MCP2515 Registers
#define MCP2515_REG_CANSTAT     0x0E
#define MCP2515_REG_CANCTRL     0x0F
#define MCP2515_REG_CNF3        0x28
#define MCP2515_REG_CNF2        0x29
#define MCP2515_REG_CNF1        0x2A
#define MCP2515_REG_CANINTE     0x2B
#define MCP2515_REG_CANINTF     0x2C

// TX Buffer 0 registers
#define MCP2515_REG_TXB0CTRL    0x30
#define MCP2515_REG_TXB0SIDH    0x31
#define MCP2515_REG_TXB0SIDL    0x32
#define MCP2515_REG_TXB0DLC     0x35
#define MCP2515_REG_TXB0DATA    0x36

// RX Buffer 0 registers
#define MCP2515_REG_RXB0CTRL    0x60
#define MCP2515_REG_RXB0SIDH    0x61
#define MCP2515_REG_RXB0SIDL    0x62
#define MCP2515_REG_RXB0DLC     0x65
#define MCP2515_REG_RXB0DATA    0x66

// CANINTF bits
#define MCP2515_INTF_RX0IF      0x01
#define MCP2515_INTF_RX1IF      0x02

// CANCTRL Register Bits (Mode selection)
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_SLEEP      0x20
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_LISTENONLY 0x60
#define MCP2515_MODE_CONFIG     0x80

// CAN message structure
typedef struct {
    uint32_t id;           // CAN ID (11-bit or 29-bit)
    uint8_t data[8];       // Data bytes (max 8)
    uint8_t dlc;           // Data length code (0-8)
    bool extended;         // true = 29-bit ID, false = 11-bit ID
    bool rtr;              // Remote Transmission Request
} mcp2515_can_message_t;

// MCP2515 device handle
typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
    uint reset_pin;
} mcp2515_t;

/**
 * @brief Initialize MCP2515 device
 *
 * @param dev Device handle
 * @param spi SPI instance (spi0 or spi1)
 * @param cs_pin Chip select GPIO pin
 * @param reset_pin Reset GPIO pin
 * @return true on success, false on failure
 */
bool mcp2515_init(mcp2515_t *dev, spi_inst_t *spi, uint cs_pin, uint reset_pin);

/**
 * @brief Reset MCP2515 via hardware reset pin
 *
 * @param dev Device handle
 */
void mcp2515_reset(mcp2515_t *dev);

/**
 * @brief Configure CAN bitrate
 *
 * @param dev Device handle
 * @param bitrate CAN bitrate in bps (e.g., 500000 for 500kbps)
 * @param crystal_freq MCP2515 crystal frequency in Hz (typically 8000000 or 16000000)
 * @return true on success, false on failure
 */
bool mcp2515_set_bitrate(mcp2515_t *dev, uint32_t bitrate, uint32_t crystal_freq);

/**
 * @brief Set MCP2515 operating mode
 *
 * @param dev Device handle
 * @param mode Mode (MCP2515_MODE_NORMAL, MCP2515_MODE_LOOPBACK, etc.)
 * @return true on success, false on failure
 */
bool mcp2515_set_mode(mcp2515_t *dev, uint8_t mode);

/**
 * @brief Send a CAN message
 *
 * @param dev Device handle
 * @param msg CAN message to send
 * @return true on success, false on failure
 */
bool mcp2515_send_message(mcp2515_t *dev, const mcp2515_can_message_t *msg);

/**
 * @brief Read a register
 *
 * @param dev Device handle
 * @param reg Register address
 * @return Register value
 */
uint8_t mcp2515_read_register(mcp2515_t *dev, uint8_t reg);

/**
 * @brief Write a register
 *
 * @param dev Device handle
 * @param reg Register address
 * @param value Value to write
 */
void mcp2515_write_register(mcp2515_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Check if a CAN message is available in RX buffer 0
 * @param dev Device handle
 * @return true if message available
 */
bool mcp2515_message_available(mcp2515_t *dev);

/**
 * @brief Receive a CAN message from RX buffer 0
 * @param dev Device handle
 * @param msg Pointer to message struct to fill
 * @return true if a message was received
 */
bool mcp2515_receive_message(mcp2515_t *dev, mcp2515_can_message_t *msg);

#endif // MCP2515_H

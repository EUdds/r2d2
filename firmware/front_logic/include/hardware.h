#pragma once

// PCB_REV is set by the build system (-DPCB_REV=...).
// Default to 1 if not specified.
#ifndef PCB_REV
#define PCB_REV 1
#endif

#if PCB_REV == 0
#define SPI0_MOSI_PIN           19
#define SPI0_CLK_PIN            18
#define SPI0_CS_PIN             17
#define SCREEN_DC_PIN           16
#define SCREEN_SPI              spi0
#define RS485_UART              uart1
#elif PCB_REV == 1
#define SPI0_MOSI_PIN           3
#define SPI0_CLK_PIN            2
#define SPI0_CS_PIN             29
#define SCREEN_RESET_PIN        27
#define SCREEN_DC_PIN           26
#define SCREEN_SPI              spi0
#define UART_TX_PIN             0
#define UART_RX_PIN             1
#define RS485_DE_PIN            7
#define RS485_UART              uart0
#elif PCB_REV == 2
#include "board.h"  // v2_bsp pin definitions

// Screen SPI is on SPI1 (GPIO 26=CLK, 27=MOSI shared by both screens)
#define SCREEN_SPI              spi1
#define SPI0_MOSI_PIN           SCREEN_MOSI_PIN     // GPIO 27
#define SPI0_CLK_PIN            SCREEN_CLK_PIN      // GPIO 26
#define SPI0_CS_PIN             SCREEN0_CS_PIN      // GPIO 25
#define SCREEN_RESET_PIN        SCREEN0_RESET_PIN   // GPIO 5

// CAN SPI is on SPI0
#define CAN_SPI_INST            spi0
#define CAN_SPI_BAUD            500000
#define MCP2515_CRYSTAL         12000000
#define CAN_BITRATE             500000

#define NODE_ID                 0x20u
#else
#error "Unsupported PCB_REV"
#endif

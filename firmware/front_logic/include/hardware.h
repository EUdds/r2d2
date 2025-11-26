#pragma once

#define PCB_REV 1

#if PCB_REV == 0
#define SPI0_MOSI_PIN           19
#define SPI0_CLK_PIN            18
#define SPI0_CS_PIN             17
#define SCREEN_DC_PIN           16
#define RS485_UART              uart1 
#elif PCB_REV == 1
#define SPI0_MOSI_PIN 3
#define SPI0_CLK_PIN 2
#define SPI0_CS_PIN 29
#define SCREEN_DC_PIN 26
#define UART_TX_PIN             0
#define UART_RX_PIN             1
#define RS485_DE_PIN            7
#define RS485_UART              uart0
#else
#error "Unsupported PCB_REV"
#endif
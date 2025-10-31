#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/uart.h"
#include "pico/types.h"

#define RS485_DEFAULT_UART uart1
#define RS485_DEFAULT_TX_PIN 4u
#define RS485_DEFAULT_RX_PIN 5u
#define RS485_DEFAULT_DIR_PIN 6u

typedef struct {
    uart_inst_t *uart;
    uint tx_pin;
    uint rx_pin;
    uint dir_pin;
    uint baud_rate;
    uint data_bits;
    uint stop_bits;
    uart_parity_t parity;
} rs485_config_t;

typedef struct {
    uart_inst_t *uart;
    uint tx_pin;
    uint rx_pin;
    uint dir_pin;
} rs485_bus_t;

rs485_config_t rs485_config_default(uint32_t baud_rate);
bool rs485_init(const rs485_config_t *config, rs485_bus_t *bus);
void rs485_write_blocking(rs485_bus_t *bus, const uint8_t *data, size_t length);
void rs485_write_string(rs485_bus_t *bus, const char *str);
size_t rs485_read_nonblocking(rs485_bus_t *bus, uint8_t *data, size_t length);
int rs485_read_byte_timeout(rs485_bus_t *bus, uint32_t timeout_ms);
void rs485_set_direction(rs485_bus_t *bus, bool transmit);

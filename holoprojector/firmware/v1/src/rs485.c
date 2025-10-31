#include "rs485.h"

#include <string.h>

#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/time.h"

static void rs485_drive_direction(const rs485_bus_t *bus, bool transmit) {
    gpio_put(bus->dir_pin, transmit ? 1 : 0);
}

rs485_config_t rs485_config_default(uint32_t baud_rate) {
    rs485_config_t cfg = {
        .uart = RS485_DEFAULT_UART,
        .tx_pin = RS485_DEFAULT_TX_PIN,
        .rx_pin = RS485_DEFAULT_RX_PIN,
        .dir_pin = RS485_DEFAULT_DIR_PIN,
        .baud_rate = baud_rate,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_NONE,
    };
    return cfg;
}

bool rs485_init(const rs485_config_t *config, rs485_bus_t *bus) {
    if (!config || !bus || config->baud_rate == 0) {
        return false;
    }

    bus->uart = config->uart ? config->uart : RS485_DEFAULT_UART;
    bus->tx_pin = config->tx_pin ? config->tx_pin : RS485_DEFAULT_TX_PIN;
    bus->rx_pin = config->rx_pin ? config->rx_pin : RS485_DEFAULT_RX_PIN;
    bus->dir_pin = config->dir_pin ? config->dir_pin : RS485_DEFAULT_DIR_PIN;

    uart_init(bus->uart, config->baud_rate);
    uart_set_hw_flow(bus->uart, false, false);
    uart_set_format(bus->uart, config->data_bits, config->stop_bits, config->parity);
    uart_set_fifo_enabled(bus->uart, true);

    gpio_set_function(bus->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(bus->rx_pin, GPIO_FUNC_UART);

    gpio_init(bus->dir_pin);
    gpio_set_dir(bus->dir_pin, GPIO_OUT);
    gpio_put(bus->dir_pin, 0);

    return true;
}

void rs485_set_direction(rs485_bus_t *bus, bool transmit) {
    if (!bus) {
        return;
    }
    rs485_drive_direction(bus, transmit);
}

void rs485_write_blocking(rs485_bus_t *bus, const uint8_t *data, size_t length) {
    if (!bus || !data || length == 0) {
        return;
    }

    rs485_drive_direction(bus, true);
    uart_write_blocking(bus->uart, data, length);
    uart_tx_wait_blocking(bus->uart);
    rs485_drive_direction(bus, false);
}

void rs485_write_string(rs485_bus_t *bus, const char *str) {
    if (!str) {
        return;
    }
    rs485_write_blocking(bus, (const uint8_t *)str, strlen(str));
}

size_t rs485_read_nonblocking(rs485_bus_t *bus, uint8_t *data, size_t length) {
    if (!bus || !data || length == 0) {
        return 0;
    }

    rs485_drive_direction(bus, false);

    size_t read = 0;
    while (read < length && uart_is_readable(bus->uart)) {
        data[read++] = (uint8_t)uart_getc(bus->uart);
    }
    return read;
}

int rs485_read_byte_timeout(rs485_bus_t *bus, uint32_t timeout_ms) {
    if (!bus) {
        return -1;
    }

    rs485_drive_direction(bus, false);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (absolute_time_diff_us(deadline, get_absolute_time()) > 0) {
        if (uart_is_readable(bus->uart)) {
            return (int)uart_getc(bus->uart);
        }
    }
    return -1;
}

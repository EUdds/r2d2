#include "mcp2515.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

// CS pin control macros
#define CS_SELECT(dev)   gpio_put((dev)->cs_pin, 0)
#define CS_DESELECT(dev) gpio_put((dev)->cs_pin, 1)

// Helper function to write SPI command
static void mcp2515_spi_write(mcp2515_t *dev, const uint8_t *data, size_t len) {
    CS_SELECT(dev);
    spi_write_blocking(dev->spi, data, len);
    CS_DESELECT(dev);
}

// Helper function to read/write SPI
static void mcp2515_spi_transfer(mcp2515_t *dev, const uint8_t *tx_data, uint8_t *rx_data, size_t len) {
    CS_SELECT(dev);
    spi_write_read_blocking(dev->spi, tx_data, rx_data, len);
    CS_DESELECT(dev);
}

bool mcp2515_init(mcp2515_t *dev, spi_inst_t *spi, uint cs_pin, uint reset_pin) {
    if (!dev || !spi) return false;

    dev->spi = spi;
    dev->cs_pin = cs_pin;
    dev->reset_pin = reset_pin;

    // Initialize CS pin (active low)
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);  // Deselect
    printf("  CS pin initialized and deselected\n");

    // Initialize reset pin (active low)
    gpio_init(reset_pin);
    gpio_set_dir(reset_pin, GPIO_OUT);
    gpio_put(reset_pin, 1);  // Not in reset
    printf("  RESET pin initialized (high = not in reset)\n");

    // Perform hardware reset
    printf("  Performing hardware reset...\n");
    mcp2515_reset(dev);
    printf("  Reset complete\n");

    // Poll until device is in config mode (with timeout)
    printf("  Polling for CONFIG mode (0x80)...\n");
    for (int i = 0; i < 20; i++) {
        sleep_ms(10);
        uint8_t mode = mcp2515_read_register(dev, MCP2515_REG_CANSTAT);
        printf("    [%d] CANSTAT = 0x%02X (mode bits = 0x%02X)\n", i, mode, (mode & 0xE0));
        if ((mode & 0xE0) == MCP2515_MODE_CONFIG) {
            printf("  Device entered CONFIG mode!\n");
            return true;
        }
    }

    // Device didn't enter config mode after reset
    printf("  Timeout: Device did not enter CONFIG mode\n");
    return false;
}

void mcp2515_reset(mcp2515_t *dev) {
    // // Hardware reset via reset pin (hold low for at least 2us per datasheet)
    // gpio_put(dev->reset_pin, 0);
    // sleep_ms(10);  // Hold reset longer to ensure clean reset
    // gpio_put(dev->reset_pin, 1);
    // sleep_ms(20);  // Wait for device to come out of reset

    // Also send software reset command
    uint8_t cmd = MCP2515_CMD_RESET;
    mcp2515_spi_write(dev, &cmd, 1);
    sleep_ms(20);  // Wait for reset to complete
}

uint8_t mcp2515_read_register(mcp2515_t *dev, uint8_t reg) {
    uint8_t tx_data[3] = {MCP2515_CMD_READ, reg, 0x00};
    uint8_t rx_data[3];

    mcp2515_spi_transfer(dev, tx_data, rx_data, 3);

    return rx_data[2];
}

void mcp2515_write_register(mcp2515_t *dev, uint8_t reg, uint8_t value) {
    uint8_t tx_data[3] = {MCP2515_CMD_WRITE, reg, value};
    mcp2515_spi_write(dev, tx_data, 3);
}

// Helper function to set timing registers
// T1 = PROP_SEG + PS1 (must be >= 2)
// T2 = PS2 (must be >= 2)
// Total bit time = 1 (SYNC) + T1 + T2 TQ
static void mcp2515_set_timing(mcp2515_t *dev, uint8_t brp, uint8_t sjw, uint8_t t1, uint8_t t2) {
    // Split T1 into PROP_SEG and PS1
    // Optimize for best sample point (~75%)
    uint8_t prop_seg = (t1 >= 8) ? 7 : (t1 / 2);
    uint8_t ps1 = t1 - prop_seg;
    uint8_t ps2 = t2;

    // CNF1: SJW[7:6] | BRP[5:0]
    uint8_t cnf1 = ((sjw - 1) << 6) | (brp & 0x3F);

    // CNF2: BTLMODE[7] | SAM[6] | PS1[5:3] | PROP[2:0]
    uint8_t cnf2 = (1 << 7) | ((ps1 - 1) << 3) | (prop_seg - 1);

    // CNF3: SOF[7] | WAKFIL[6] | X | PS2[2:0]
    uint8_t cnf3 = (ps2 - 1);

    mcp2515_write_register(dev, MCP2515_REG_CNF1, cnf1);
    mcp2515_write_register(dev, MCP2515_REG_CNF2, cnf2);
    mcp2515_write_register(dev, MCP2515_REG_CNF3, cnf3);
}

bool mcp2515_set_bitrate(mcp2515_t *dev, uint32_t bitrate, uint32_t crystal_freq) {
    // Must be in configuration mode to set bitrate
    mcp2515_write_register(dev, MCP2515_REG_CANCTRL, MCP2515_MODE_CONFIG);

    // Poll until we're in config mode (with timeout)
    for (int i = 0; i < 10; i++) {
        sleep_ms(5);
        uint8_t mode = mcp2515_read_register(dev, MCP2515_REG_CANSTAT);
        if ((mode & 0xE0) == MCP2515_MODE_CONFIG) {
            break;
        }
        if (i == 9) {
            // Failed to enter config mode
            return false;
        }
    }

    // Timing parameters: BRP, SJW, T1, T2
    // T1 = PROP_SEG + PS1, T2 = PS2
    // Bit time = 1 + T1 + T2 TQ
    // TQ = 2 * (BRP + 1) / F_osc

    if (crystal_freq == 8000000 && bitrate == 500000) {
        // 500kbps @ 8MHz: BRP=0, TQ=250ns, 8 TQ
        // SYNC=1, T1=4, T2=3 (sample point 62.5%)
        mcp2515_set_timing(dev, 0, 1, 4, 3);
    } else if (crystal_freq == 12000000 && bitrate == 500000) {
        // 500kbps @ 12MHz: BRP=0, TQ=166.67ns, 12 TQ
        // SYNC=1, T1=7, T2=4 (sample point 66.7%)
        mcp2515_set_timing(dev, 0, 1, 7, 4);
    } else if (crystal_freq == 16000000 && bitrate == 500000) {
        // 500kbps @ 16MHz: BRP=0, TQ=125ns, 16 TQ
        // SYNC=1, T1=11, T2=4 (sample point 75%)
        mcp2515_set_timing(dev, 0, 1, 11, 4);
    } else {
        // Unsupported configuration
        return false;
    }

    return true;
}

bool mcp2515_set_mode(mcp2515_t *dev, uint8_t mode) {
    mcp2515_write_register(dev, MCP2515_REG_CANCTRL, mode);

    // Poll until mode changes (with timeout)
    // Increase timeout to 200ms (20 iterations x 10ms)
    for (int i = 0; i < 20; i++) {
        sleep_ms(10);
        uint8_t status = mcp2515_read_register(dev, MCP2515_REG_CANSTAT);
        if ((status & 0xE0) == mode) {
            return true;
        }
    }

    // Failed to change mode
    return false;
}

bool mcp2515_send_message(mcp2515_t *dev, const mcp2515_can_message_t *msg) {
    if (!msg || msg->dlc > 8) return false;

    // Use TX Buffer 0
    // Check if buffer is free
    uint8_t ctrl = mcp2515_read_register(dev, MCP2515_REG_TXB0CTRL);
    if (ctrl & 0x08) {
        // TX request pending, buffer busy
        return false;
    }

    // Write CAN ID (standard 11-bit)
    if (!msg->extended) {
        // Standard ID (11-bit)
        uint8_t sidh = (msg->id >> 3) & 0xFF;
        uint8_t sidl = (msg->id << 5) & 0xE0;

        mcp2515_write_register(dev, MCP2515_REG_TXB0SIDH, sidh);
        mcp2515_write_register(dev, MCP2515_REG_TXB0SIDL, sidl);
    } else {
        // Extended ID (29-bit) - not implemented in this simple example
        return false;
    }

    // Write DLC
    uint8_t dlc = msg->dlc & 0x0F;
    if (msg->rtr) dlc |= 0x40;  // Set RTR bit
    mcp2515_write_register(dev, MCP2515_REG_TXB0DLC, dlc);

    // Write data bytes
    for (uint8_t i = 0; i < msg->dlc; i++) {
        mcp2515_write_register(dev, MCP2515_REG_TXB0DATA + i, msg->data[i]);
    }

    // Request transmission (RTS for buffer 0)
    uint8_t rts_cmd = MCP2515_CMD_RTS | 0x01;  // RTS for TXB0
    mcp2515_spi_write(dev, &rts_cmd, 1);

    return true;
}

bool mcp2515_message_available(mcp2515_t *dev) {
    uint8_t intf = mcp2515_read_register(dev, MCP2515_REG_CANINTF);
    return (intf & MCP2515_INTF_RX0IF) != 0;
}

bool mcp2515_receive_message(mcp2515_t *dev, mcp2515_can_message_t *msg) {
    if (!msg) return false;

    uint8_t intf = mcp2515_read_register(dev, MCP2515_REG_CANINTF);
    if (!(intf & MCP2515_INTF_RX0IF)) {
        return false;
    }

    // Read CAN ID
    uint8_t sidh = mcp2515_read_register(dev, MCP2515_REG_RXB0SIDH);
    uint8_t sidl = mcp2515_read_register(dev, MCP2515_REG_RXB0SIDL);
    uint8_t dlc  = mcp2515_read_register(dev, MCP2515_REG_RXB0DLC);

    msg->extended = (sidl & 0x08) != 0;
    if (!msg->extended) {
        msg->id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
    } else {
        // Extended ID
        uint8_t eid8 = mcp2515_read_register(dev, MCP2515_REG_RXB0SIDH + 2);
        uint8_t eid0 = mcp2515_read_register(dev, MCP2515_REG_RXB0SIDH + 3);
        msg->id = ((uint32_t)(sidh) << 21) |
                  ((uint32_t)(sidl & 0xE0) << 13) |
                  ((uint32_t)(sidl & 0x03) << 16) |
                  ((uint32_t)eid8 << 8) |
                  eid0;
    }

    msg->rtr = (dlc & 0x40) != 0;
    msg->dlc = dlc & 0x0F;
    if (msg->dlc > 8) msg->dlc = 8;

    for (uint8_t i = 0; i < msg->dlc; i++) {
        msg->data[i] = mcp2515_read_register(dev, MCP2515_REG_RXB0DATA + i);
    }

    // Clear RX0IF flag using bit-modify
    uint8_t bm_cmd[4] = {MCP2515_CMD_BIT_MODIFY, MCP2515_REG_CANINTF, MCP2515_INTF_RX0IF, 0x00};
    CS_SELECT(dev);
    spi_write_blocking(dev->spi, bm_cmd, 4);
    CS_DESELECT(dev);

    return true;
}

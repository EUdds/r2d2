#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "board.h"
#include "mcp2515.h"
#include <stdio.h>

// SPI Configuration
#define CAN_SPI_INST    spi0
#define CAN_SPI_BAUD    500000   // 500 kHz SPI clock (reduced for reliability)

// MCP2515 Configuration
#define CAN_BITRATE     500000   // 500 kbps
#define MCP2515_CRYSTAL 12000000  // 12 MHz crystal

// Quick status read using READ STATUS instruction (0xA0)
static uint8_t mcp2515_read_status(mcp2515_t *dev) {
    uint8_t tx[2] = {MCP2515_CMD_READ_STATUS, 0x00};
    uint8_t rx[2] = {0};

    gpio_put(dev->cs_pin, 0);
    spi_write_read_blocking(dev->spi, tx, rx, 2);
    gpio_put(dev->cs_pin, 1);

    return rx[1];
}

int main() {
    // Initialize board (LEDs and CAN GPIO pins)
    board_init();

    // Wait for USB connection
    sleep_ms(2000);

    printf("\n\n=== R2CAN Boot - MCP2515 CAN Controller Test ===\n");
    printf("Board initialized\n");

    printf("CAN GPIO pins initialized\n");

    // Initialize SPI
    printf("Initializing SPI at %d Hz...\n", CAN_SPI_BAUD);
    spi_init(CAN_SPI_INST, CAN_SPI_BAUD);
    spi_set_format(CAN_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    printf("SPI configured: Mode 0, 8-bit, MSB first\n");

    // Configure SPI pins
    gpio_set_function(CAN_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAN_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAN_CLK_PIN, GPIO_FUNC_SPI);
    printf("SPI pins: MOSI=%d, MISO=%d, CLK=%d, CS=%d\n",
           CAN_MOSI_PIN, CAN_MISO_PIN, CAN_CLK_PIN, CAN_CS_PIN);

    // Initialize MCP2515
    printf("\nInitializing MCP2515 (CS=%d, RESET=%d)...\n", CAN_CS_PIN, CAN_RESETN_PIN);
    mcp2515_t can_dev;
    bool init_ok = mcp2515_init(&can_dev, CAN_SPI_INST, CAN_CS_PIN, CAN_RESETN_PIN);

    if (!init_ok) {
        printf("ERROR: MCP2515 init failed!\n");

        // Try to read CANSTAT to see what we got
        printf("Attempting to read CANSTAT register...\n");
        uint8_t canstat = mcp2515_read_register(&can_dev, MCP2515_REG_CANSTAT);
        printf("CANSTAT = 0x%02X\n", canstat);
        printf("Mode bits = 0x%02X\n", (canstat >> 5) & 0x07);

        LED_RED_ON();
        while (true) {
            tight_loop_contents();
        }
    }

    printf("MCP2515 initialized successfully!\n");
    LED_GREEN_ON();

    // Quick poll READ STATUS to see what we get
    printf("\n--- READ STATUS (0xA0) test ---\n");
    uint8_t status = mcp2515_read_status(&can_dev);
    printf("READ STATUS = 0x%02X\n", status);
    printf("  RX0IF=%d RX1IF=%d\n", (status >> 0) & 1, (status >> 1) & 1);
    printf("  TX0REQ=%d TX0IF=%d\n", (status >> 2) & 1, (status >> 3) & 1);
    printf("  TX1REQ=%d TX1IF=%d\n", (status >> 4) & 1, (status >> 5) & 1);
    printf("  TX2REQ=%d TX2IF=%d\n", (status >> 6) & 1, (status >> 7) & 1);

    // Configure bitrate
    printf("\nConfiguring CAN bitrate: %d bps @ %d Hz crystal...\n",
           CAN_BITRATE, MCP2515_CRYSTAL);

    if (!mcp2515_set_bitrate(&can_dev, CAN_BITRATE, MCP2515_CRYSTAL)) {
        printf("ERROR: Failed to set bitrate!\n");
        LED_RED_ON();
        while (true) { tight_loop_contents(); }
    }
    printf("Bitrate configured successfully\n");

    // Try to set operating mode
    printf("\nSetting MCP2515 mode...\n");
    bool mode_ok = false;

    if (mcp2515_set_mode(&can_dev, MCP2515_MODE_NORMAL)) {
        printf("Mode set to NORMAL\n");
        mode_ok = true;
        LED_BLUE_ON();
    } else {
        printf("NORMAL mode failed, trying LOOPBACK...\n");

        if (mcp2515_set_mode(&can_dev, MCP2515_MODE_LOOPBACK)) {
            printf("Mode set to LOOPBACK\n");
            mode_ok = true;
            LED_BLUE_ON();
        } else {
            printf("ERROR: Both NORMAL and LOOPBACK modes failed!\n");
            uint8_t canstat = mcp2515_read_register(&can_dev, MCP2515_REG_CANSTAT);
            printf("Current CANSTAT = 0x%02X\n", canstat);
            LED_RED_ON();
            while (true) { tight_loop_contents(); }
        }
    }

    printf("\n=== MCP2515 Configuration Complete ===\n");
    printf("Starting CAN message transmission...\n\n");

    // Prepare CAN message
    mcp2515_can_message_t msg = {
        .id = 0x123,
        .dlc = 8,
        .extended = false,
        .rtr = false,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
    };

    uint32_t msg_count = 0;

    // Main loop - send CAN message every second
    while (true) {
        // Update message counter in first two data bytes
        msg.data[0] = (msg_count & 0xFF);
        msg.data[1] = ((msg_count >> 8) & 0xFF);

        // Send CAN message
        if (mcp2515_send_message(&can_dev, &msg)) {
            printf("[%u] TX OK: ID=0x%03X DLC=%d Data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                   msg_count,
                   msg.id,
                   msg.dlc,
                   msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                   msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            LED_GREEN_ON();
            sleep_ms(50);
            LED_GREEN_OFF();
            msg_count++;
        } else {
            printf("[%u] TX FAILED!\n", msg_count);
            LED_RED_ON();
            sleep_ms(50);
            LED_RED_OFF();
        }

        sleep_ms(950);  // Rest of 1 second interval
    }

    return 0;
}

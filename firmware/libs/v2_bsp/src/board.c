#include "board.h"



void board_led_init(void) {
    // Initialize RED LED (active-low, start OFF = true/HIGH)
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, true);  // true = output
    gpio_put(LED_RED_PIN, true);      // true/HIGH = off (active-low)

    // Initialize GREEN LED (active-low, start OFF = true/HIGH)
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, true);
    gpio_put(LED_GREEN_PIN, true);

    // Initialize BLUE LED (active-low, start OFF = true/HIGH)
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, true);
    gpio_put(LED_BLUE_PIN, true);
}

void board_init(void) {
    // Initialize standard I/O (USB serial, etc.)
    stdio_init_all();

    // Initialize LED subsystem
    board_led_init();

    // board_can_init();

    // Add additional board initialization here as needed
    // Examples: ADC, I2C, SPI, UART, etc.
}

void board_led_set_rgb(bool red, bool green, bool blue) {
    gpio_put(LED_RED_PIN, red);
    gpio_put(LED_GREEN_PIN, green);
    gpio_put(LED_BLUE_PIN, blue);
}

void board_led_all_off(void) {
    LED_RED_OFF();
    LED_GREEN_OFF();
    LED_BLUE_OFF();
}

void board_led_all_on(void) {
    LED_RED_ON();
    LED_GREEN_ON();
    LED_BLUE_ON();
}

void board_can_init(void) {
    // Initialize CS pin (active-low, set HIGH to deselect)
    gpio_init(CAN_CS_PIN);
    gpio_set_dir(CAN_CS_PIN, true);  // Output
    gpio_put(CAN_CS_PIN, true);      // HIGH = deselected

    // Initialize RESETN pin (active-low, set HIGH to keep device active)
    gpio_init(CAN_RESETN_PIN);
    gpio_set_dir(CAN_RESETN_PIN, true);  // Output
    gpio_set_pulls(CAN_RESETN_PIN, true, false);
    gpio_put(CAN_RESETN_PIN, true);      // HIGH = not in reset

    // Initialize TX Request-to-Send pins (active-low, set HIGH = inactive)
    gpio_init(CAN_TX0RTSN_PIN);
    gpio_set_dir(CAN_TX0RTSN_PIN, true);  // Output
    gpio_put(CAN_TX0RTSN_PIN, true);      // HIGH = inactive

    gpio_init(CAN_TX1RTSN_PIN);
    gpio_set_dir(CAN_TX1RTSN_PIN, true);  // Output
    gpio_put(CAN_TX1RTSN_PIN, true);      // HIGH = inactive

    gpio_init(CAN_TX2RTSN_PIN);
    gpio_set_dir(CAN_TX2RTSN_PIN, true);  // Output
    gpio_put(CAN_TX2RTSN_PIN, true);      // HIGH = inactive

    // Initialize interrupt pin (active-low input from MCP2515)
    gpio_init(CAN_INTN_PIN);
    gpio_set_dir(CAN_INTN_PIN, false);  // Input
    gpio_pull_up(CAN_INTN_PIN);         // Enable pull-up

    // Initialize RX Buffer Full pins (active-low inputs from MCP2515)
    gpio_init(CAN_RX0BFN_PIN);
    gpio_set_dir(CAN_RX0BFN_PIN, false);  // Input
    gpio_pull_up(CAN_RX0BFN_PIN);         // Enable pull-up

    gpio_init(CAN_RX1BFN_PIN);
    gpio_set_dir(CAN_RX1BFN_PIN, false);  // Input
    gpio_pull_up(CAN_RX1BFN_PIN);         // Enable pull-up

    sleep_ms(1000); // Wait for MCP2515 to power up and stabilize

    // Note: SPI pins (MOSI, MISO, CLK) should be initialized
    // by the SPI peripheral driver, not here
}

uint board_screen_spi_init(uint32_t baudrate) {
    gpio_set_function(SCREEN_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SCREEN_CLK_PIN,  GPIO_FUNC_SPI);
    return spi_init(spi1, baudrate);
}

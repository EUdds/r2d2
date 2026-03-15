#include "can_receiver.h"
#include "hardware.h"
#include "board.h"
#include "mcp2515.h"
#include <hardware/spi.h>

static mcp2515_t s_can;

bool can_receiver_init(void)
{
    spi_init(CAN_SPI_INST, CAN_SPI_BAUD);
    spi_set_format(CAN_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(CAN_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAN_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAN_CLK_PIN, GPIO_FUNC_SPI);

    if (!mcp2515_init(&s_can, CAN_SPI_INST, CAN_CS_PIN, CAN_RESETN_PIN)) {
        return false;
    }
    if (!mcp2515_set_bitrate(&s_can, CAN_BITRATE, MCP2515_CRYSTAL)) {
        return false;
    }
    return mcp2515_set_mode(&s_can, MCP2515_MODE_NORMAL);
}

void can_receiver_poll(void)
{
    // TODO: receive and dispatch incoming CAN messages
}

void can_send_heartbeat(void)
{
    mcp2515_can_message_t msg = {
        .id       = NODE_ID,
        .dlc      = 0,
        .extended = false,
        .rtr      = false,
        .data     = {0},
    };
    mcp2515_send_message(&s_can, &msg);
}

/**
 * R2CAN Bootloader - Receives firmware over CAN and writes to flash.
 *
 * Protocol (host -> device): CAN ID 0x7E0
 * Protocol (device -> host): CAN ID 0x7E8
 *
 * CAN Transport Framing:
 *   Byte 0: [7]=is_last, [6:0]=seq_num (0-based fragment index)
 *   Bytes 1-7: payload fragment (up to 7 bytes)
 *
 * Logical Packet Format (after reassembly):
 *   Byte 0:   CMD
 *   Byte 1:   VERSION (must be 1)
 *   Byte 2-3: PAYLOAD_LENGTH (little-endian uint16)
 *   Byte 4-7: CRC32 of payload (little-endian uint32)
 *   Byte 8+:  PAYLOAD
 *
 * Commands:
 *   0x01 CMD_PING  - no payload
 *   0x02 CMD_INFO  - no payload; ACK payload = struct info
 *   0x10 CMD_START - payload = [size:4][crc32:4]
 *   0x11 CMD_DATA  - payload = [offset:4][data...]
 *   0x12 CMD_DONE  - no payload; ACK payload = [written:4][crc32:4]
 *   0x13 CMD_BOOT  - no payload
 *   0x14 CMD_ABORT - no payload
 *   0x80 CMD_ACK   - payload = [acked_cmd:1][status:1][extra...]
 *
 * Flash layout:
 *   0x10000000 .. 0x1001FFFF : bootloader (128 KB)
 *   0x10020000 .. end        : application slot
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "board.h"
#include "mcp2515.h"
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------
#define CAN_SPI_INST      spi0
#define CAN_SPI_BAUD      500000
#define CAN_BITRATE       500000
#define MCP2515_CRYSTAL   12000000

#define CAN_ID_HOST_TO_DEV  0x7E0
#define CAN_ID_DEV_TO_HOST  0x7E8

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------
#define PROTOCOL_VERSION  1

#define CMD_PING    0x01
#define CMD_INFO    0x02
#define CMD_START   0x10
#define CMD_DATA    0x11
#define CMD_DONE    0x12
#define CMD_BOOT    0x13
#define CMD_ABORT   0x14
#define CMD_ACK     0x80

#define STATUS_OK            0
#define STATUS_BAD_STATE     1
#define STATUS_INVALID_ARG   2
#define STATUS_TOO_LARGE     3
#define STATUS_ALIGNMENT     4
#define STATUS_CRC_MISMATCH  5
#define STATUS_INTERNAL_ERR  6
#define STATUS_NO_APP        7
#define STATUS_INVALID_CMD   8

// ---------------------------------------------------------------------------
// Flash layout
// ---------------------------------------------------------------------------
#define FLASH_BASE          0x10000000u
#define APP_FLASH_OFFSET    0x00020000u   // 128 KB for bootloader
#define APP_MAX_SIZE        (2048 * 1024 - APP_FLASH_OFFSET)  // rest of 2MB flash

// ---------------------------------------------------------------------------
// Transport layer (CAN fragmentation)
// ---------------------------------------------------------------------------
#define MAX_PKT_SIZE   1024
#define MAX_FRAG_DATA  7

static uint8_t rx_pkt[MAX_PKT_SIZE];
static uint16_t rx_pkt_len = 0;

// ---------------------------------------------------------------------------
// CRC32
// ---------------------------------------------------------------------------
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
            else crc >>= 1;
        }
    }
    return ~crc;
}

// ---------------------------------------------------------------------------
// Transmit a logical packet over CAN, fragmented
// ---------------------------------------------------------------------------
static bool can_send_packet(mcp2515_t *can, const uint8_t *pkt, uint16_t pkt_len) {
    uint16_t offset = 0;
    uint8_t seq = 0;

    while (offset < pkt_len) {
        uint16_t remaining = pkt_len - offset;
        uint8_t frag_len = (remaining > MAX_FRAG_DATA) ? MAX_FRAG_DATA : (uint8_t)remaining;
        bool is_last = (offset + frag_len >= pkt_len);

        mcp2515_can_message_t msg;
        msg.id = CAN_ID_DEV_TO_HOST;
        msg.extended = false;
        msg.rtr = false;
        msg.dlc = 1 + frag_len;
        msg.data[0] = (is_last ? 0x80 : 0x00) | (seq & 0x7F);
        memcpy(&msg.data[1], &pkt[offset], frag_len);

        // Retry send up to 10 times
        for (int retry = 0; retry < 10; retry++) {
            if (mcp2515_send_message(can, &msg)) break;
            sleep_ms(1);
        }

        offset += frag_len;
        seq = (seq + 1) & 0x7F;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Try to receive one fragment and reassemble into rx_pkt
// Returns true when a complete logical packet is available in rx_pkt
// ---------------------------------------------------------------------------
static bool can_recv_fragment(mcp2515_t *can) {
    mcp2515_can_message_t msg;
    if (!mcp2515_receive_message(can, &msg)) return false;
    if (msg.id != CAN_ID_HOST_TO_DEV) return false;
    if (msg.dlc < 1) return false;

    uint8_t hdr = msg.data[0];
    uint8_t seq = hdr & 0x7F;
    bool is_last = (hdr & 0x80) != 0;
    uint8_t frag_len = msg.dlc - 1;

    // If seq==0 this is the start of a new packet; reset buffer
    if (seq == 0) {
        rx_pkt_len = 0;
    }

    if (rx_pkt_len + frag_len > MAX_PKT_SIZE) {
        rx_pkt_len = 0;
        return false;
    }

    memcpy(&rx_pkt[rx_pkt_len], &msg.data[1], frag_len);
    rx_pkt_len += frag_len;

    return is_last;
}

// ---------------------------------------------------------------------------
// Build and send an ACK packet
// ---------------------------------------------------------------------------
static void send_ack(mcp2515_t *can, uint8_t acked_cmd, uint8_t status,
                     const uint8_t *extra, uint8_t extra_len) {
    uint8_t payload[2 + 32];
    payload[0] = acked_cmd;
    payload[1] = status;
    if (extra && extra_len > 0) {
        memcpy(&payload[2], extra, extra_len);
    }
    uint16_t payload_len = 2 + extra_len;

    uint8_t pkt[8 + 32];
    uint32_t crc = crc32_update(0, payload, payload_len);
    pkt[0] = CMD_ACK;
    pkt[1] = PROTOCOL_VERSION;
    pkt[2] = payload_len & 0xFF;
    pkt[3] = (payload_len >> 8) & 0xFF;
    pkt[4] = (crc) & 0xFF;
    pkt[5] = (crc >> 8) & 0xFF;
    pkt[6] = (crc >> 16) & 0xFF;
    pkt[7] = (crc >> 24) & 0xFF;
    memcpy(&pkt[8], payload, payload_len);

    can_send_packet(can, pkt, 8 + payload_len);
}

// ---------------------------------------------------------------------------
// Parse a received logical packet header
// Returns pointer to payload, sets *payload_len and *cmd; NULL on error
// ---------------------------------------------------------------------------
static uint8_t *parse_packet(uint8_t *cmd, uint16_t *payload_len) {
    if (rx_pkt_len < 8) return NULL;
    *cmd = rx_pkt[0];
    uint8_t version = rx_pkt[1];
    *payload_len = (uint16_t)rx_pkt[2] | ((uint16_t)rx_pkt[3] << 8);
    if (version != PROTOCOL_VERSION) return NULL;
    if (8u + *payload_len > rx_pkt_len) return NULL;

    uint8_t *payload = &rx_pkt[8];
    uint32_t pkt_crc = (uint32_t)rx_pkt[4] |
                       ((uint32_t)rx_pkt[5] << 8) |
                       ((uint32_t)rx_pkt[6] << 16) |
                       ((uint32_t)rx_pkt[7] << 24);
    uint32_t calc_crc = crc32_update(0, payload, *payload_len);
    if (pkt_crc != calc_crc) return NULL;
    return payload;
}

// ---------------------------------------------------------------------------
// Application vector check: look for valid stack pointer at app_offset
// ---------------------------------------------------------------------------
static bool app_is_valid(void) {
    const uint32_t *app = (const uint32_t *)(FLASH_BASE + APP_FLASH_OFFSET);
    // Stack pointer must be in SRAM range: 0x20000000..0x20082000
    return (app[0] >= 0x20000000u && app[0] <= 0x20082000u);
}

// ---------------------------------------------------------------------------
// Jump to application
// ---------------------------------------------------------------------------
static void boot_app(void) __attribute__((noreturn));
static void boot_app(void) {
    const uint32_t *app = (const uint32_t *)(FLASH_BASE + APP_FLASH_OFFSET);
    uint32_t sp  = app[0];
    uint32_t entry = app[1];

    // Disable interrupts, reset peripherals
    __asm volatile ("cpsid i");

    // Set vector table to application
    // SCB->VTOR = FLASH_BASE + APP_FLASH_OFFSET
    *(volatile uint32_t *)0xE000ED08 = FLASH_BASE + APP_FLASH_OFFSET;

    // Set stack pointer and jump
    __asm volatile (
        "msr msp, %0\n"
        "bx  %1\n"
        :: "r"(sp), "r"(entry)
    );
    __builtin_unreachable();
}

// ---------------------------------------------------------------------------
// Bootloader state machine
// ---------------------------------------------------------------------------
typedef enum {
    STATE_IDLE,
    STATE_RECEIVING,
    STATE_DONE,
} bl_state_t;

static bl_state_t bl_state = STATE_IDLE;
static uint32_t expected_size = 0;
static uint32_t expected_crc = 0;
static uint32_t bytes_written = 0;

static void handle_ping(mcp2515_t *can) {
    send_ack(can, CMD_PING, STATUS_OK, NULL, 0);
}

static void handle_info(mcp2515_t *can) {
    uint8_t info[24];
    uint32_t proto   = PROTOCOL_VERSION;
    uint32_t app_off = APP_FLASH_OFFSET;
    uint32_t app_max = APP_MAX_SIZE;
    uint32_t present = app_is_valid() ? 1 : 0;
    uint32_t app_sz  = 0;
    uint32_t app_crc = 0;

    memcpy(&info[0],  &proto,   4);
    memcpy(&info[4],  &app_off, 4);
    memcpy(&info[8],  &app_max, 4);
    memcpy(&info[12], &present, 4);
    memcpy(&info[16], &app_sz,  4);
    memcpy(&info[20], &app_crc, 4);

    send_ack(can, CMD_INFO, STATUS_OK, info, sizeof(info));
}

static void handle_start(mcp2515_t *can, const uint8_t *payload, uint16_t payload_len) {
    if (payload_len < 8) {
        send_ack(can, CMD_START, STATUS_INVALID_ARG, NULL, 0);
        return;
    }

    uint32_t size, crc;
    memcpy(&size, &payload[0], 4);
    memcpy(&crc,  &payload[4], 4);

    if (size > APP_MAX_SIZE) {
        send_ack(can, CMD_START, STATUS_TOO_LARGE, NULL, 0);
        return;
    }

    expected_size = size;
    expected_crc  = crc;
    bytes_written  = 0;
    bl_state       = STATE_RECEIVING;

    LED_BLUE_ON();

    // Erase flash sectors for the application
    uint32_t erase_size = (size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    uint32_t saved_ints = save_and_disable_interrupts();
    flash_range_erase(APP_FLASH_OFFSET, erase_size);
    restore_interrupts(saved_ints);

    send_ack(can, CMD_START, STATUS_OK, NULL, 0);
}

static void handle_data(mcp2515_t *can, const uint8_t *payload, uint16_t payload_len) {
    if (bl_state != STATE_RECEIVING) {
        send_ack(can, CMD_DATA, STATUS_BAD_STATE, NULL, 0);
        return;
    }
    if (payload_len < 5) {
        send_ack(can, CMD_DATA, STATUS_INVALID_ARG, NULL, 0);
        return;
    }

    uint32_t offset;
    memcpy(&offset, &payload[0], 4);
    const uint8_t *data = &payload[4];
    uint16_t data_len = payload_len - 4;

    if (offset + data_len > APP_MAX_SIZE) {
        send_ack(can, CMD_DATA, STATUS_TOO_LARGE, NULL, 0);
        return;
    }
    if (offset % FLASH_PAGE_SIZE != 0) {
        send_ack(can, CMD_DATA, STATUS_ALIGNMENT, NULL, 0);
        return;
    }

    // Pad to page size if needed
    uint8_t page_buf[FLASH_PAGE_SIZE];
    uint16_t padded = ((data_len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    memset(page_buf, 0xFF, sizeof(page_buf));
    memcpy(page_buf, data, data_len < FLASH_PAGE_SIZE ? data_len : FLASH_PAGE_SIZE);

    // Program in page_size chunks
    uint32_t written = 0;
    while (written < data_len) {
        uint16_t chunk = (data_len - written > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : (data_len - written);
        uint8_t buf[FLASH_PAGE_SIZE];
        memset(buf, 0xFF, FLASH_PAGE_SIZE);
        memcpy(buf, data + written, chunk);

        uint32_t saved_ints = save_and_disable_interrupts();
        flash_range_program(APP_FLASH_OFFSET + offset + written, buf, FLASH_PAGE_SIZE);
        restore_interrupts(saved_ints);

        written += chunk;
    }

    bytes_written += written;

    uint8_t extra[4];
    memcpy(extra, &bytes_written, 4);
    send_ack(can, CMD_DATA, STATUS_OK, extra, 4);
}

static void handle_done(mcp2515_t *can) {
    if (bl_state != STATE_RECEIVING) {
        send_ack(can, CMD_DONE, STATUS_BAD_STATE, NULL, 0);
        return;
    }

    // Verify CRC of written firmware
    const uint8_t *app_flash = (const uint8_t *)(FLASH_BASE + APP_FLASH_OFFSET);
    uint32_t actual_crc = crc32_update(0, app_flash, expected_size);

    if (actual_crc != expected_crc) {
        bl_state = STATE_IDLE;
        send_ack(can, CMD_DONE, STATUS_CRC_MISMATCH, NULL, 0);
        return;
    }

    bl_state = STATE_DONE;
    LED_GREEN_ON();
    LED_BLUE_OFF();

    uint8_t extra[8];
    memcpy(&extra[0], &bytes_written, 4);
    memcpy(&extra[4], &actual_crc, 4);
    send_ack(can, CMD_DONE, STATUS_OK, extra, 8);
}

static void handle_boot(mcp2515_t *can) {
    if (bl_state != STATE_DONE) {
        send_ack(can, CMD_BOOT, STATUS_BAD_STATE, NULL, 0);
        return;
    }
    if (!app_is_valid()) {
        send_ack(can, CMD_BOOT, STATUS_NO_APP, NULL, 0);
        return;
    }
    send_ack(can, CMD_BOOT, STATUS_OK, NULL, 0);
    sleep_ms(10);  // let ACK transmit
    boot_app();
}

static void handle_abort(mcp2515_t *can) {
    bl_state = STATE_IDLE;
    bytes_written = 0;
    LED_BLUE_OFF();
    send_ack(can, CMD_ABORT, STATUS_OK, NULL, 0);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void) {
    board_init();
    sleep_ms(500);

    printf("\n\n=== R2CAN Bootloader ===\n");
    printf("App slot: 0x%08X, max %u bytes\n", FLASH_BASE + APP_FLASH_OFFSET, APP_MAX_SIZE);

    // Init SPI
    spi_init(CAN_SPI_INST, CAN_SPI_BAUD);
    spi_set_format(CAN_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(CAN_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAN_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CAN_CLK_PIN,  GPIO_FUNC_SPI);

    mcp2515_t can;
    if (!mcp2515_init(&can, CAN_SPI_INST, CAN_CS_PIN, CAN_RESETN_PIN)) {
        printf("ERROR: MCP2515 init failed\n");
        LED_RED_ON();
        while (true) tight_loop_contents();
    }

    if (!mcp2515_set_bitrate(&can, CAN_BITRATE, MCP2515_CRYSTAL)) {
        printf("ERROR: bitrate config failed\n");
        LED_RED_ON();
        while (true) tight_loop_contents();
    }

    if (!mcp2515_set_mode(&can, MCP2515_MODE_NORMAL)) {
        printf("ERROR: mode config failed\n");
        LED_RED_ON();
        while (true) tight_loop_contents();
    }

    printf("CAN initialized at %d bps\n", CAN_BITRATE);

    // If no pending update request and app is valid, boot immediately after 2s
    absolute_time_t boot_deadline = make_timeout_time_ms(2000);
    bool pending_update = false;

    printf("Waiting for CAN ping (2s)...\n");
    LED_GREEN_ON();

    while (true) {
        if (mcp2515_message_available(&can)) {
            if (can_recv_fragment(&can)) {
                uint8_t cmd;
                uint16_t payload_len;
                uint8_t *payload = parse_packet(&cmd, &payload_len);

                if (payload != NULL) {
                    if (!pending_update && cmd == CMD_PING) {
                        pending_update = true;
                        printf("Ping received, entering update mode\n");
                        LED_GREEN_OFF();
                        LED_BLUE_ON();
                    }

                    switch (cmd) {
                        case CMD_PING:  handle_ping(&can); break;
                        case CMD_INFO:  handle_info(&can); break;
                        case CMD_START: handle_start(&can, payload, payload_len); break;
                        case CMD_DATA:  handle_data(&can, payload, payload_len); break;
                        case CMD_DONE:  handle_done(&can); break;
                        case CMD_BOOT:  handle_boot(&can); break;
                        case CMD_ABORT: handle_abort(&can); break;
                        default:
                            send_ack(&can, cmd, STATUS_INVALID_CMD, NULL, 0);
                            break;
                    }
                }
            }
        }

        // Boot app if no update pending and deadline reached
        if (!pending_update && absolute_time_diff_us(get_absolute_time(), boot_deadline) <= 0) {
            if (app_is_valid()) {
                printf("Booting application...\n");
                boot_app();
            } else {
                // No valid app, stay in bootloader
                pending_update = true;
                printf("No valid application, waiting for CAN firmware\n");
                LED_RED_ON();
                LED_GREEN_OFF();
            }
        }
    }
}

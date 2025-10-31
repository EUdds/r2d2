#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rs485.h"

#define R2BUS_HEADER 0xAA55u
#define R2BUS_SYNC_BYTE_0 0x55u
#define R2BUS_SYNC_BYTE_1 0xAAu
#define R2BUS_MAX_DATA_LENGTH 255u
#define R2BUS_BROADCAST_ID 0xFFu
#define R2BUS_HOST_ID 0x00u

typedef enum {
    R2BUS_MSG_ECU_RESET = 0x01,
    R2BUS_MSG_PING = 0x02,
    R2BUS_MSG_PONG = 0x03,
    R2BUS_MSG_HEARTBEAT = 0x04,
    R2BUS_MSG_PSI_COLOR_REQ = 0x05,
    R2BUS_MSG_ACK = 0x7F,
} r2bus_msg_id_E;

typedef enum {
    R2BUS_STATUS_OK = 0x00,
    R2BUS_STATUS_ERROR = 0x01,
} r2bus_status_E;

typedef struct {
    uint8_t dest_id;
    uint8_t src_id;
    r2bus_msg_id_E msg_id;
    uint8_t length;
    uint8_t payload[R2BUS_MAX_DATA_LENGTH];
} r2bus_packet_t;

typedef void (*r2bus_packet_handler_t)(const r2bus_packet_t *packet, void *user_data);
typedef void (*r2bus_reset_handler_t)(void *user_data);

typedef struct {
    r2bus_packet_handler_t on_packet;
    r2bus_reset_handler_t on_reset;
} r2bus_handlers_t;

typedef enum {
    R2BUS_RX_SYNC0 = 0,
    R2BUS_RX_SYNC1,
    R2BUS_RX_DEST,
    R2BUS_RX_SRC,
    R2BUS_RX_ID,
    R2BUS_RX_LENGTH,
    R2BUS_RX_PAYLOAD,
    R2BUS_RX_CRC_LSB,
    R2BUS_RX_CRC_MSB,
} r2bus_rx_state_E;

typedef struct {
    rs485_bus_t *bus;
    uint8_t node_id;
    r2bus_handlers_t handlers;
    void *user_data;
    struct {
        r2bus_rx_state_E state;
        r2bus_packet_t packet;
        uint8_t payload_index;
        uint16_t crc;
        uint16_t rx_crc;
    } rx;
} r2bus_ctx_t;

bool r2bus_init(r2bus_ctx_t *ctx,
                rs485_bus_t *bus,
                uint8_t node_id,
                const r2bus_handlers_t *handlers,
                void *user_data);

void r2bus_poll(r2bus_ctx_t *ctx);
bool r2bus_send(r2bus_ctx_t *ctx,
                uint8_t dest_id,
                r2bus_msg_id_E msg_id,
                const uint8_t *payload,
                uint8_t length);

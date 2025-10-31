/**
 * Simple RS485 bus helper for RP2040 nodes.
 * Provides basic framing, CRC validation, and built-in ECU reset handling so
 * the host can reboot nodes for OTA updates.
 */

#include "r2bus.h"

#include <string.h>

#include "pico/platform.h"

#define R2BUS_RX_BUFFER_SIZE 64
#define R2BUS_CRC_SEED 0xFFFFu

static inline uint16_t r2bus_crc16_init(void) {
    return R2BUS_CRC_SEED;
}

static inline uint16_t r2bus_crc16_update(uint16_t crc, uint8_t data) {
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x8000u) {
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

static void r2bus_reset_rx(r2bus_ctx_t *ctx) {
    ctx->rx.state = R2BUS_RX_SYNC0;
    ctx->rx.payload_index = 0;
    ctx->rx.crc = r2bus_crc16_init();
    ctx->rx.rx_crc = 0;
}

static void r2bus_send_ack(r2bus_ctx_t *ctx,
                           uint8_t dest_id,
                           r2bus_status_E status,
                           r2bus_msg_id_E original_msg) {
    if (!ctx || !ctx->bus) {
        return;
    }
    if (dest_id == R2BUS_BROADCAST_ID) {
        return;  // No ACKs for broadcast frames
    }
    const uint8_t payload[2] = { (uint8_t)status, (uint8_t)original_msg };
    (void)r2bus_send(ctx, dest_id, R2BUS_MSG_ACK, payload, sizeof(payload));
}

static void r2bus_process_payload(r2bus_ctx_t *ctx) {
    const r2bus_packet_t *packet = &ctx->rx.packet;
    if (!ctx->bus) {
        return;
    }

    const bool is_targeted = (packet->dest_id == ctx->node_id) ||
                             (packet->dest_id == R2BUS_BROADCAST_ID);
    if (!is_targeted) {
        return;
    }

    switch (packet->msg_id) {
        case R2BUS_MSG_ECU_RESET:
            r2bus_send_ack(ctx, packet->src_id, R2BUS_STATUS_OK, packet->msg_id);
            if (ctx->handlers.on_reset) {
                ctx->handlers.on_reset(ctx->user_data);
            }
            break;
        case R2BUS_MSG_PING:
            // Respond directly to the caller
            if (packet->dest_id == ctx->node_id) {
                (void)r2bus_send(ctx,
                                 packet->src_id,
                                 R2BUS_MSG_PONG,
                                 packet->payload,
                                 packet->length);
            }
            break;
        default:
            break;
    }

    if (ctx->handlers.on_packet) {
        ctx->handlers.on_packet(packet, ctx->user_data);
    }
}

static void r2bus_feed_byte(r2bus_ctx_t *ctx, uint8_t byte) {
    switch (ctx->rx.state) {
        case R2BUS_RX_SYNC0:
            if (byte == R2BUS_SYNC_BYTE_0) {
                ctx->rx.state = R2BUS_RX_SYNC1;
            }
            break;
        case R2BUS_RX_SYNC1:
            if (byte == R2BUS_SYNC_BYTE_1) {
                ctx->rx.state = R2BUS_RX_DEST;
                ctx->rx.crc = r2bus_crc16_init();
            } else {
                ctx->rx.state = R2BUS_RX_SYNC0;
            }
            break;
        case R2BUS_RX_DEST:
            ctx->rx.packet.dest_id = byte;
            ctx->rx.crc = r2bus_crc16_update(ctx->rx.crc, byte);
            ctx->rx.state = R2BUS_RX_SRC;
            break;
        case R2BUS_RX_SRC:
            ctx->rx.packet.src_id = byte;
            ctx->rx.crc = r2bus_crc16_update(ctx->rx.crc, byte);
            ctx->rx.state = R2BUS_RX_ID;
            break;
        case R2BUS_RX_ID:
            ctx->rx.packet.msg_id = (r2bus_msg_id_E)byte;
            ctx->rx.crc = r2bus_crc16_update(ctx->rx.crc, byte);
            ctx->rx.state = R2BUS_RX_LENGTH;
            break;
        case R2BUS_RX_LENGTH:
            ctx->rx.packet.length = byte;
            ctx->rx.crc = r2bus_crc16_update(ctx->rx.crc, byte);
            ctx->rx.payload_index = 0;
            if (byte > R2BUS_MAX_DATA_LENGTH) {
                r2bus_reset_rx(ctx);
            } else if (byte == 0) {
                ctx->rx.state = R2BUS_RX_CRC_LSB;
            } else {
                ctx->rx.state = R2BUS_RX_PAYLOAD;
            }
            break;
        case R2BUS_RX_PAYLOAD:
            ctx->rx.packet.payload[ctx->rx.payload_index++] = byte;
            ctx->rx.crc = r2bus_crc16_update(ctx->rx.crc, byte);
            if (ctx->rx.payload_index >= ctx->rx.packet.length) {
                ctx->rx.state = R2BUS_RX_CRC_LSB;
            }
            break;
        case R2BUS_RX_CRC_LSB:
            ctx->rx.rx_crc = byte;
            ctx->rx.state = R2BUS_RX_CRC_MSB;
            break;
        case R2BUS_RX_CRC_MSB:
            ctx->rx.rx_crc |= (uint16_t)byte << 8;
            if (ctx->rx.rx_crc == ctx->rx.crc) {
                r2bus_process_payload(ctx);
            }
            r2bus_reset_rx(ctx);
            break;
    }
}

bool r2bus_init(r2bus_ctx_t *ctx,
                rs485_bus_t *bus,
                uint8_t node_id,
                const r2bus_handlers_t *handlers,
                void *user_data) {
    if (!ctx || !bus) {
        return false;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->bus = bus;
    ctx->node_id = node_id;

    if (handlers) {
        ctx->handlers = *handlers;
    }
    ctx->user_data = user_data;
    r2bus_reset_rx(ctx);
    return true;
}

void r2bus_poll(r2bus_ctx_t *ctx) {
    if (!ctx || !ctx->bus) {
        return;
    }
    uint8_t buffer[R2BUS_RX_BUFFER_SIZE];
    size_t read = rs485_read_nonblocking(ctx->bus, buffer, sizeof(buffer));
    for (size_t i = 0; i < read; ++i) {
        r2bus_feed_byte(ctx, buffer[i]);
    }
}

bool r2bus_send(r2bus_ctx_t *ctx,
                uint8_t dest_id,
                r2bus_msg_id_E msg_id,
                const uint8_t *payload,
                uint8_t length) {
    if (!ctx || !ctx->bus || length > R2BUS_MAX_DATA_LENGTH) {
        return false;
    }
    uint8_t frame[2 + 4 + R2BUS_MAX_DATA_LENGTH + 2];
    size_t index = 0;
    uint16_t crc = r2bus_crc16_init();

    frame[index++] = R2BUS_SYNC_BYTE_0;
    frame[index++] = R2BUS_SYNC_BYTE_1;

    const uint8_t fields[4] = { dest_id, ctx->node_id, (uint8_t)msg_id, length };
    for (size_t i = 0; i < 4; ++i) {
        frame[index++] = fields[i];
        crc = r2bus_crc16_update(crc, fields[i]);
    }

    if (length > 0 && payload) {
        memcpy(&frame[index], payload, length);
        for (uint8_t i = 0; i < length; ++i) {
            crc = r2bus_crc16_update(crc, payload[i]);
        }
        index += length;
    }

    frame[index++] = (uint8_t)(crc & 0xFFu);
    frame[index++] = (uint8_t)((crc >> 8) & 0xFFu);

    rs485_write_blocking(ctx->bus, frame, index);
    return true;
}

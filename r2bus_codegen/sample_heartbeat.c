#include "r2bus_generated.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    uint32_t last_seen_ms;
    bool has_seen;
} node_state_t;

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}

static void sleep_ms(uint32_t ms) {
    struct timespec req;
    req.tv_sec = (time_t)(ms / 1000u);
    req.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&req, NULL);
}

static int node_index_from_id(uint8_t node_id) {
    for (size_t i = 0; i < R2BUS_GENERATED_NODE_COUNT; ++i) {
        if (r2bus_generated_nodes[i].node_id == node_id) {
            return (int)i;
        }
    }
    return -1;
}

typedef enum {
    RX_SYNC0 = 0,
    RX_SYNC1,
    RX_DEST,
    RX_SRC,
    RX_ID,
    RX_LENGTH,
    RX_PAYLOAD,
    RX_CRC_LSB,
    RX_CRC_MSB,
} rx_state_t;

typedef struct {
    rx_state_t state;
    r2bus_packet_t packet;
    uint8_t payload_index;
    uint16_t crc;
    uint16_t rx_crc;
} rx_state_ctx_t;

static uint16_t crc16_init(void) {
    return 0xFFFFu;
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
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

static bool read_packet(r2bus_packet_t *out) {
    static int fd = -1;
    static rx_state_ctx_t rx = {.state = RX_SYNC0};
    static uint8_t rx_buf[512];
    static ssize_t rx_len = 0;
    static ssize_t rx_pos = 0;

    if (fd < 0) {
        fd = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            return false;
        }
        struct termios tio;
        if (tcgetattr(fd, &tio) == 0) {
            cfmakeraw(&tio);
            cfsetispeed(&tio, B115200);
            cfsetospeed(&tio, B115200);
            tio.c_cflag |= (CLOCAL | CREAD);
            tio.c_cflag &= (tcflag_t)~CRTSCTS;
            tio.c_cc[VMIN] = 0;
            tio.c_cc[VTIME] = 0;
            (void)tcsetattr(fd, TCSANOW, &tio);
        }
    }

    if (rx_pos >= rx_len) {
        rx_len = read(fd, rx_buf, (size_t)sizeof(rx_buf));
        rx_pos = 0;
        if (rx_len <= 0) {
            return false;
        }
    }

    while (rx_pos < rx_len) {
        uint8_t byte = rx_buf[rx_pos++];
        switch (rx.state) {
            case RX_SYNC0:
                if (byte == 0x55u) {
                    rx.state = RX_SYNC1;
                }
                break;
            case RX_SYNC1:
                if (byte == 0xAAu) {
                    rx.state = RX_DEST;
                    rx.crc = crc16_init();
                } else {
                    rx.state = RX_SYNC0;
                }
                break;
            case RX_DEST:
                rx.packet.dest_id = byte;
                rx.crc = crc16_update(rx.crc, byte);
                rx.state = RX_SRC;
                break;
            case RX_SRC:
                rx.packet.src_id = byte;
                rx.crc = crc16_update(rx.crc, byte);
                rx.state = RX_ID;
                break;
            case RX_ID:
                rx.packet.msg_id = (r2bus_msg_id_E)byte;
                rx.crc = crc16_update(rx.crc, byte);
                rx.state = RX_LENGTH;
                break;
            case RX_LENGTH:
                rx.packet.length = byte;
                rx.crc = crc16_update(rx.crc, byte);
                rx.payload_index = 0;
                if (byte > R2BUS_GENERATED_MAX_PAYLOAD) {
                    rx.state = RX_SYNC0;
                } else if (byte == 0) {
                    rx.state = RX_CRC_LSB;
                } else {
                    rx.state = RX_PAYLOAD;
                }
                break;
            case RX_PAYLOAD:
                rx.packet.payload[rx.payload_index++] = byte;
                rx.crc = crc16_update(rx.crc, byte);
                if (rx.payload_index >= rx.packet.length) {
                    rx.state = RX_CRC_LSB;
                }
                break;
            case RX_CRC_LSB:
                rx.rx_crc = byte;
                rx.state = RX_CRC_MSB;
                break;
            case RX_CRC_MSB:
                rx.rx_crc |= (uint16_t)byte << 8;
                if (rx.rx_crc == rx.crc) {
                    *out = rx.packet;
                    rx.state = RX_SYNC0;
                    return true;
                }
                rx.state = RX_SYNC0;
                break;
        }
    }
    return false;
}

static void process_packet(const r2bus_packet_t *packet, uint32_t now_ms, node_state_t *states) {
    if (!r2bus_handle_packet(packet, now_ms)) {
        return;
    }
    if (packet->msg_id != R2BUS_MSG_HEARTBEAT) {
        return;
    }
    int index = node_index_from_id(packet->src_id);
    if (index < 0) {
        return;
    }
    states[(size_t)index].last_seen_ms = now_ms;
    states[(size_t)index].has_seen = true;
}

static void poll_bus(node_state_t *states) {
    r2bus_packet_t packet;
    while (read_packet(&packet)) {
        uint32_t rx_time = now_ms();
        process_packet(&packet, rx_time, states);
    }
}

int main(void) {
    node_state_t states[R2BUS_GENERATED_NODE_COUNT];
    memset(states, 0, sizeof(states));

    while (true) {
        poll_bus(states);

        uint32_t now = now_ms();
        printf("Heartbeat ages (ms):\n");
        for (size_t i = 0; i < R2BUS_GENERATED_NODE_COUNT; ++i) {
            const r2bus_generated_NodeInfo *node = &r2bus_generated_nodes[i];
            if (states[i].has_seen) {
                uint32_t age_ms = now - states[i].last_seen_ms;
                printf("  %s: %u\n", node->name, (unsigned)age_ms);
            } else {
                printf("  %s: never\n", node->name);
            }
        }
        printf("\n");
        sleep_ms(1000u);
    }
}

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "r2bus.h"
#include "rs485.h"
#include "ws2812.h"
#include <hardware/clocks.h>

#define NODE_ID 0x10u

static volatile bool g_reset_pending = false;
static absolute_time_t g_reset_deadline;

static void handle_ecu_reset(void *user_data) {
    (void)user_data;
    g_reset_pending = true;
    g_reset_deadline = make_timeout_time_ms(250);
}

static void handle_r2bus_packet(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    switch (packet->msg_id) {
        case R2BUS_MSG_ACK:
        case R2BUS_MSG_PING:
        case R2BUS_MSG_HEARTBEAT:
            // No application-specific handling yet.
            break;
        
        case R2BUS_MSG_PSI_COLOR_REQ:



            break;
        default:
            break;
    }
}

static void publish_heartbeat(r2bus_ctx_t *bus) {
    uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
    (void)r2bus_send(bus,
                     R2BUS_HOST_ID,
                     R2BUS_MSG_HEARTBEAT,
                     (const uint8_t *)&uptime_ms,
                     sizeof(uptime_ms));
}

static void fatal_blink(uint led_pin, uint32_t on_ms, uint32_t off_ms) {
    while (true) {
        gpio_put(led_pin, true);
        sleep_ms(on_ms);
        gpio_put(led_pin, false);
        sleep_ms(off_ms);
    }
}

int main(void) {
    const uint led_pin = PICO_DEFAULT_LED_PIN;
    set_sys_clock_48mhz();

    stdio_init_all();

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    ws2812_handle_t* ws2812 = init_ws2812();




    rs485_bus_t rs485;
    rs485_config_t cfg = rs485_config_default(115200);
    if (!rs485_init(&cfg, &rs485)) {
        fatal_blink(led_pin, 100, 100);
    }

    static r2bus_ctx_t r2bus;
    const r2bus_handlers_t handlers = {
        .on_packet = handle_r2bus_packet,
        .on_reset = handle_ecu_reset,
    };

    if (!r2bus_init(&r2bus, &rs485, NODE_ID, &handlers, &r2bus)) {
        fatal_blink(led_pin, 200, 200);
    }

    bool led_state = false;
    absolute_time_t next_led_toggle = make_timeout_time_ms(250);
    absolute_time_t next_heartbeat = make_timeout_time_ms(1000);

    while (true) {
        if (time_reached(next_led_toggle)) {
            led_state = !led_state;
            gpio_put(led_pin, led_state);
            next_led_toggle = delayed_by_ms(next_led_toggle, 250);
        }
        
        uint32_t color = urgb_u32(0x00,212, 255); // Blue

        for (int i = 0; i < 16; ++i)
        {
            put_pixel(HOLOPROJECTOR_STRIP, color); // Green
            put_pixel(FRONT_PSI_STRIP, color); // Green
        }

        r2bus_poll(&r2bus);

        if (time_reached(next_heartbeat)) {
            publish_heartbeat(&r2bus);
            next_heartbeat = delayed_by_ms(next_heartbeat, 1000);
        }

        if (g_reset_pending && time_reached(g_reset_deadline)) {
            watchdog_reboot(0, 0, 0);
            while (true) {
                tight_loop_contents();
            }
        }
    }
}

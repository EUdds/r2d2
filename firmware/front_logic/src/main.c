#include "r2bus.h"
#include "rs485.h"
#include "r2bus_receiver.h"
#include "status_led.h"
#include <hardware/watchdog.h>
#include <pico/time.h>

#include "tasks.h"

// Keep in sync with the dcfront bootloader node ID so ECU resets land correctly.
#define NODE_ID 0x10u

static volatile bool g_reset_pending = false;
static absolute_time_t g_reset_deadline;

static void handle_ecu_reset(void *user_data) {
    (void)user_data;
    g_reset_pending = true;
    g_reset_deadline = make_timeout_time_ms(250);
}


int main(void) {
    
    rs485_bus_t rs485;
    rs485_config_t cfg = rs485_config_default(115200);
    if (!rs485_init(&cfg, &rs485)) {
        fatal_blink(100);
    }

    static r2bus_ctx_t r2bus;
    const r2bus_handlers_t handlers = {
        .on_packet = handle_r2bus_packet_componentSpecific,
        .on_reset = handle_ecu_reset,
    };


    if (!r2bus_init(&r2bus, &rs485, NODE_ID, &handlers, &r2bus)) {
        fatal_blink(100);
    }

    task_setup();

    absolute_time_t next_led_toggle = make_timeout_time_ms(250);
    absolute_time_t next_heartbeat = make_timeout_time_ms(1000);

    absolute_time_t next_1hz_task = make_timeout_time_ms(1000);
    absolute_time_t next_10hz_task = make_timeout_time_ms(100);
    absolute_time_t next_100hz_task = make_timeout_time_ms(10);
    absolute_time_t next_1000hz_task = make_timeout_time_ms(1);

    while (true) {
        if (time_reached(next_1hz_task)) {
            task_1hz();
            next_1hz_task = delayed_by_ms(next_1hz_task, 1000);
        }
        if (time_reached(next_10hz_task)) {
            task_10hz();
            next_10hz_task = delayed_by_ms(next_10hz_task, 100);
        }
        if (time_reached(next_100hz_task)) {
            task_100hz();
            next_100hz_task = delayed_by_ms(next_100hz_task, 10);
        }
        if (time_reached(next_1000hz_task)) {
            task_1khz();
            next_1000hz_task = delayed_by_ms(next_1000hz_task, 1);
        }
        
        if (g_reset_pending && time_reached(g_reset_deadline)) {
            watchdog_reboot(0, 0, 0);
            while (true) {
                tight_loop_contents();
            }
        }
    }
}

#include "tasks.h"
#include "status_led.h"
#include "logic_lights.h"
#include "hardware.h"
#include <pico/stdio.h>
#include <stdint.h>

#if PCB_REV >= 2
#include "can_receiver.h"
#else
#include <r2bus.h>
#include <r2bus_transmitter.h>
#include "r2bus_receiver.h"
#endif


void task_setup(void)
{
    stdio_init_all();
    status_led_init();
    logic_lights_init();
}

void task_1hz(void)
{
#if PCB_REV >= 2
    // can_send_heartbeat();
#else
    r2bus_publish_heartbeat();
#endif
    status_led_toggle();
}

void task_10hz(void)
{
    logic_lights_run_animation_step();
}

void task_100hz(void)
{
    logic_lights_update_screen();
}

void task_1khz(void)
{
#if PCB_REV >= 2
    // can_receiver_poll();
#else
    r2bus_ctx_t* r2bus = r2bus_get_active_context();
    r2bus_poll(r2bus);
    r2bus_transmit(r2bus);
#endif
}

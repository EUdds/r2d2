#include "tasks.h"

#include <r2bus.h>
#include <r2bus_transmitter.h>
#include "r2bus_receiver.h"
#include "status_led.h"
#include <pico/stdio.h>
#include <stdint.h>
#include "logic_lights.h"


void task_setup(void)
{
    stdio_init_all();
    status_led_init();
    logic_lights_init();
}

void task_1hz(void)
{
    r2bus_publish_heartbeat();
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

void task_1khz(void) {
    r2bus_ctx_t* r2bus = r2bus_get_active_context();
    r2bus_poll(r2bus);
    r2bus_transmit(r2bus);
}

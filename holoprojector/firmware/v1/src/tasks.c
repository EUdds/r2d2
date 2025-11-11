#include "tasks.h"

#include "r2bus.h"
#include "r2bus_transmitter.h"
#include "r2bus_receiver.h"
#include "status_led.h"
#include "pico_servo.h"
#include "ws2812.h"
#include "holoprojector_lights.h"
#include "psi_light.h"
#include <pico/stdio.h>
#include <stdint.h>

typedef struct
{
    uint32_t az_deg;
    uint32_t el_deg;
} holo_state_t;

static holo_state_t g_holo_state = {
    .az_deg = 10,
    .el_deg = 25,
};


void task_setup(void) {
    stdio_init_all();
    status_led_init();
    pico_servo_init();
    pico_servo_home();
    init_ws2812();
    psi_light_init();
    holoprojector_lights_init();
}

void task_1hz(void) {
    r2bus_publish_heartbeat();
    status_led_toggle();
    
}

void task_10hz(void)
{
    holoprojector_lights_update();
}

void task_100hz(void) {
    psi_light_update();
}

void task_1khz(void) {
    r2bus_ctx_t* r2bus = r2bus_get_active_context();
    r2bus_poll(r2bus);
    r2bus_transmit(r2bus);
}

#include "holoprojector_lights.h"
#include "ws2812.h"
#include "pico_servo.h"

#include <pico/time.h>
#include <stdint.h>

typedef struct
{
    struct {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } holo_color_rgb;
    uint8_t holo_brightness;
    ws2812_handle_t* ws2812_handle;
    servo_E az_servo;
    uint8_t az_servo_angle;
    servo_E el_servo;
    uint8_t el_servo_angle;
} holo_lights_state_t;

static holo_lights_state_t g_holo_lights_state = {
    .holo_color_rgb = {
        .r = 0,
        .g = 0,
        .b = 0,
    },
    .holo_brightness = 127,
};

bool holoprojector_lights_init()
{
    ws2812_handle_t* handle = ws2812_get_handle(HOLOPROJECTOR_STRIP);
    if (!handle) {
        return false;
    }
    g_holo_lights_state.ws2812_handle = handle;

    g_holo_lights_state.az_servo = AZ_SERVO;
    g_holo_lights_state.el_servo = EL_SERVO;
    g_holo_lights_state.az_servo_angle = (uint8_t)pico_servo_get_home_angle(AZ_SERVO);
    g_holo_lights_state.el_servo_angle = (uint8_t)pico_servo_get_home_angle(EL_SERVO);

    // Flash red briefly on init
    fill_pixels(HOLOPROJECTOR_STRIP, urgb_u32(255, 0, 0));
    sleep_ms(100);
    fill_pixels(HOLOPROJECTOR_STRIP, urgb_u32(0, 0, 0));

    return true;
}

void holoprojector_lights_set_brightness(uint8_t brightness)
{
    g_holo_lights_state.holo_brightness = brightness;
}

void holoprojector_lights_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    g_holo_lights_state.holo_color_rgb.r = r;
    g_holo_lights_state.holo_color_rgb.g = g;
    g_holo_lights_state.holo_color_rgb.b = b;
}

void holoprojector_servo_set_angles(uint8_t az_angle, uint8_t el_angle)
{
    g_holo_lights_state.az_servo_angle = az_angle;
    g_holo_lights_state.el_servo_angle = el_angle;
}

void holoprojector_lights_update()
{
    if (!g_holo_lights_state.ws2812_handle) {
        return;
    }
    uint16_t scaled_r = (uint16_t)g_holo_lights_state.holo_color_rgb.r * g_holo_lights_state.holo_brightness / 255;
    uint16_t scaled_g = (uint16_t)g_holo_lights_state.holo_color_rgb.g * g_holo_lights_state.holo_brightness / 255;
    uint16_t scaled_b = (uint16_t)g_holo_lights_state.holo_color_rgb.b * g_holo_lights_state.holo_brightness / 255;
    fill_pixels(HOLOPROJECTOR_STRIP, urgb_u32(scaled_r, scaled_g, scaled_b));

    pico_servo_move_angle(AZ_SERVO, (float)g_holo_lights_state.az_servo_angle);
    pico_servo_move_angle(EL_SERVO, (float)g_holo_lights_state.el_servo_angle);
}
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool holoprojector_lights_init();
void holoprojector_lights_set_brightness(uint8_t brightness);
void holoprojector_lights_set_color(uint8_t r, uint8_t g, uint8_t b);
void holoprojector_servo_set_angles(uint8_t az_angle, uint8_t el_angle);
void holoprojector_lights_update();
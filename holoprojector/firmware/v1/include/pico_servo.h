#pragma once

#include <stdint.h>
#include <hardware/pio.h>

typedef enum
{
    AZ_SERVO = 0,
    EL_SERVO = 1,
    NUM_SERVOS = 2,
} servo_E;

typedef struct
{
    PIO pio;
    uint32_t sm;
    uint32_t offset;
    float freq_hz;
    uint32_t period;
    float last_angle_deg;
} pico_servo_handle_t;

void pico_servo_init(void);
void pico_servo_move_angle(servo_E servo, float angle);
void pico_servo_move_pulse_us(servo_E servo, float pulse_us);
void pico_servo_home(void);
void pico_servo_home_servo(servo_E servo);
void pico_servo_set_trim(servo_E servo, float trim_deg);
void pico_servo_set_pulse_range(servo_E servo, float min_us, float max_us);
float pico_servo_get_home_angle(servo_E servo);

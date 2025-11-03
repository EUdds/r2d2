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
} pico_servo_handle_t;

void pico_servo_init(void);
void pico_servo_move_angle(servo_E servo, float angle);
void pico_servo_home(void);

#include "pico_servo.h"

#include "pico_servo.pio.h"

#include <hardware/pio.h>
#include <hardware/clocks.h>

#define SERVO_1_PIN 17
#define SERVO_2_PIN 16

#define MIN_DC 650
#define MAX_DC 2250

#define SERVO_PWM_FREQ_HZ 50.0f
#define SERVO_CLK_DIV 64u

typedef struct
{
    uint32_t pin;
    uint32_t home_angle;
    uint32_t max_angle;
    uint32_t min_angle;
    bool positive_direction;
    PIO pio;
    uint32_t sm;
    uint32_t offset;
} servo_config_t;

static pico_servo_handle_t pico_servo_handles[NUM_SERVOS] = {0};

static const servo_config_t servo_config[NUM_SERVOS] = {
    [AZ_SERVO] = {
        .pin = SERVO_1_PIN,
        .home_angle = 5,
        .max_angle = 45,
        .min_angle = 0,
        .positive_direction = true,
    },
    [EL_SERVO] = {
        .pin = SERVO_2_PIN,
        .home_angle = 160,
        .max_angle = 180,
        .min_angle = 0,
        .positive_direction = false,
    },
};

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

static inline float pulse_to_deg(float pulse_us) {
    const float min_us = MIN_DC;
    const float max_us = MAX_DC;
    return (pulse_us - min_us) * 180.0f / (max_us - min_us);
}

static inline float deg_to_pulse(float deg) {
    const float min_us = MIN_DC;
    const float max_us = MAX_DC;
    return min_us + deg * (max_us - min_us) / 180.0f;
}


void pico_servo_init(void)
{
    for (int i = 0; i < NUM_SERVOS; ++i)
    {
        PIO pio;
        uint32_t sm;
        uint32_t offset;
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&pico_servo_pio_program, &pio, &sm, &offset, servo_config[i].pin, 1, true);
        (void)success;
        
        pico_servo_pio_program_init(pio, sm, offset, SERVO_CLK_DIV, servo_config[i].pin);

        uint32_t cycles = clock_get_hz(clk_sys) / (uint32_t)(SERVO_PWM_FREQ_HZ * SERVO_CLK_DIV);
        uint32_t period = (cycles - 3u) / 3u; // 20ms period

        pio_pwm_set_period(pio, sm, period);

        float angle = (float)servo_config[i].home_angle;
        float pulse_us = deg_to_pulse(angle);
        const float pwm_period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;
        uint32_t level = (uint32_t)((pulse_us / pwm_period_us) * period);
        pio_pwm_set_level(pio, sm, level);

        pico_servo_handles[i].pio = pio;
        pico_servo_handles[i].sm = sm;
        pico_servo_handles[i].offset = offset;
        pico_servo_handles[i].freq_hz = SERVO_PWM_FREQ_HZ;
        pico_servo_handles[i].period = period;
    }
}

void pico_servo_move_angle(servo_E servo, float angle)
{
    if (servo >= NUM_SERVOS)
        return;

    if (angle < (float)servo_config[servo].min_angle)
        angle = (float)servo_config[servo].min_angle;
    if (angle > (float)servo_config[servo].max_angle)
        angle = (float)servo_config[servo].max_angle;

    if (!servo_config[servo].positive_direction)
    {
        angle = (float)servo_config[servo].max_angle - angle;
    }

    float pulse_us = deg_to_pulse(angle);
    float freq_hz = pico_servo_handles[servo].freq_hz;
    uint32_t period = pico_servo_handles[servo].period;
    if (period == 0 || freq_hz <= 0.0f) {
        return;
    }
    const float pwm_period_us = 1000000.0f / freq_hz;
    uint32_t level = (uint32_t)((pulse_us / pwm_period_us) * (float)period);

    pio_pwm_set_level(pico_servo_handles[servo].pio, pico_servo_handles[servo].sm, level);
}

void pico_servo_home(void)
{
    for (int i = 0; i < NUM_SERVOS; ++i)
    {
        pico_servo_move_angle((servo_E)i, (float)servo_config[i].home_angle);
    }
}

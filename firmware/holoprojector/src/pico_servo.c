#include "pico_servo.h"

#include "pico_servo.pio.h"

#include <hardware/pio.h>
#include <hardware/clocks.h>

#define SERVO_1_PIN 17
#define SERVO_2_PIN 16

#define SERVO_PWM_FREQ_HZ 50.0f
#define SERVO_CLK_DIV 64u
#define SERVO_PULSE_HARD_MIN_US 300.0f
#define SERVO_PULSE_HARD_MAX_US 2700.0f

typedef struct
{
    uint32_t pin;
    float home_angle;
    float safe_min_angle;
    float safe_max_angle;
    float cal_min_angle;
    float cal_max_angle;
    bool positive_direction;
    float trim_deg;
    float pulse_min_us;
    float pulse_max_us;
} servo_config_t;

static pico_servo_handle_t pico_servo_handles[NUM_SERVOS] = {0};

static servo_config_t servo_config[NUM_SERVOS] = {
    [AZ_SERVO] = {
        .pin = SERVO_1_PIN,
        .home_angle = 10.0f,
        .safe_min_angle = 10.0f,
        .safe_max_angle = 30.0f,
        .cal_min_angle = 0.0f,
        .cal_max_angle = 170.0f,
        .positive_direction = false,
        .trim_deg = 0.0f,
        .pulse_min_us = 550.0f,
        .pulse_max_us = 2500.0f,
    },
    [EL_SERVO] = {
        .pin = SERVO_2_PIN,
        .home_angle = 25.0f,
        .safe_min_angle = 0.0f,
        .safe_max_angle = 110.0f,
        .cal_min_angle = 0.0f,
        .cal_max_angle = 170.0f,
        .positive_direction = false,
        .trim_deg = 0.0f,
        .pulse_min_us = 550.0f,
        .pulse_max_us = 2500.0f,
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

static inline float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static inline float servo_safe_angle_to_pulse_us(servo_E servo, float safe_angle_deg) {
    const servo_config_t *cfg = &servo_config[servo];
    float clamped_angle = clampf(safe_angle_deg, cfg->cal_min_angle, cfg->cal_max_angle);

    const float span_deg = cfg->cal_max_angle - cfg->cal_min_angle;
    float normalized = 0.0f;
    if (span_deg > 0.0f) {
        normalized = (clamped_angle - cfg->cal_min_angle) / span_deg;
    }
    normalized = clampf(normalized, 0.0f, 1.0f);

    if (!cfg->positive_direction) {
        normalized = 1.0f - normalized;
    }

    const float span_us = cfg->pulse_max_us - cfg->pulse_min_us;
    if (span_us <= 0.0f) {
        return cfg->pulse_min_us;
    }
    return cfg->pulse_min_us + normalized * span_us;
}

static inline float servo_pulse_us_to_angle(servo_E servo, float pulse_us) {
    const servo_config_t *cfg = &servo_config[servo];
    float clamped = clampf(pulse_us, cfg->pulse_min_us, cfg->pulse_max_us);
    const float span_us = cfg->pulse_max_us - cfg->pulse_min_us;
    float normalized = 0.0f;
    if (span_us > 0.0f) {
        normalized = (clamped - cfg->pulse_min_us) / span_us;
    }
    normalized = clampf(normalized, 0.0f, 1.0f);
    if (!cfg->positive_direction) {
        normalized = 1.0f - normalized;
    }
    const float span_deg = cfg->cal_max_angle - cfg->cal_min_angle;
    float angle = cfg->cal_min_angle + normalized * span_deg;
    return clampf(angle, cfg->safe_min_angle, cfg->safe_max_angle);
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

        const servo_config_t *cfg = &servo_config[i];
        float base_angle = clampf(cfg->home_angle, cfg->safe_min_angle, cfg->safe_max_angle);
        float safe_angle = clampf(base_angle + cfg->trim_deg, cfg->safe_min_angle, cfg->safe_max_angle);
        float pulse_us = servo_safe_angle_to_pulse_us((servo_E)i, safe_angle);
        const float pwm_period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;
        uint32_t level = (uint32_t)((pulse_us / pwm_period_us) * (float)period);
        pio_pwm_set_level(pio, sm, level);

        pico_servo_handles[i].pio = pio;
        pico_servo_handles[i].sm = sm;
        pico_servo_handles[i].offset = offset;
        pico_servo_handles[i].freq_hz = SERVO_PWM_FREQ_HZ;
        pico_servo_handles[i].period = period;
        pico_servo_handles[i].last_angle_deg = base_angle;
    }
}

void pico_servo_move_angle(servo_E servo, float angle)
{
    if (servo >= NUM_SERVOS)
        return;

    const servo_config_t *cfg = &servo_config[servo];
    float freq_hz = pico_servo_handles[servo].freq_hz;
    uint32_t period = pico_servo_handles[servo].period;
    if (period == 0 || freq_hz <= 0.0f) {
        return;
    }
    float base_angle = clampf(angle, cfg->safe_min_angle, cfg->safe_max_angle);
    float safe_angle = clampf(base_angle + cfg->trim_deg, cfg->safe_min_angle, cfg->safe_max_angle);
    float pulse_us = servo_safe_angle_to_pulse_us(servo, safe_angle);
    const float pwm_period_us = 1000000.0f / freq_hz;
    uint32_t level = (uint32_t)((pulse_us / pwm_period_us) * (float)period);

    pio_pwm_set_level(pico_servo_handles[servo].pio, pico_servo_handles[servo].sm, level);
    pico_servo_handles[servo].last_angle_deg = base_angle;
}

void pico_servo_move_pulse_us(servo_E servo, float pulse_us)
{
    if (servo >= NUM_SERVOS) {
        return;
    }

    const servo_config_t *cfg = &servo_config[servo];
    float freq_hz = pico_servo_handles[servo].freq_hz;
    uint32_t period = pico_servo_handles[servo].period;
    if (period == 0 || freq_hz <= 0.0f) {
        return;
    }

    float clamped_pulse = clampf(pulse_us, SERVO_PULSE_HARD_MIN_US, SERVO_PULSE_HARD_MAX_US);
    const float pwm_period_us = 1000000.0f / freq_hz;
    uint32_t level = (uint32_t)((clamped_pulse / pwm_period_us) * (float)period);

    pio_pwm_set_level(pico_servo_handles[servo].pio, pico_servo_handles[servo].sm, level);
    float mapped_pulse = clampf(clamped_pulse,
                                cfg->pulse_min_us,
                                cfg->pulse_max_us);
    float safe_angle = servo_pulse_us_to_angle(servo, mapped_pulse);
    float base_angle = clampf(safe_angle - cfg->trim_deg, cfg->safe_min_angle, cfg->safe_max_angle);
    pico_servo_handles[servo].last_angle_deg = base_angle;
}

void pico_servo_home(void)
{
    for (int i = 0; i < NUM_SERVOS; ++i)
    {
        pico_servo_move_angle((servo_E)i, (float)servo_config[i].home_angle);
    }
}

void pico_servo_home_servo(servo_E servo)
{
    if (servo >= NUM_SERVOS) {
        return;
    }
    pico_servo_move_angle(servo, (float)servo_config[servo].home_angle);
}

void pico_servo_set_trim(servo_E servo, float trim_deg)
{
    if (servo >= NUM_SERVOS) {
        return;
    }
    servo_config[servo].trim_deg = trim_deg;
    pico_servo_move_angle(servo, pico_servo_handles[servo].last_angle_deg);
}

void pico_servo_set_pulse_range(servo_E servo, float min_us, float max_us)
{
    if (servo >= NUM_SERVOS) {
        return;
    }
    if (min_us <= 0.0f || max_us <= 0.0f || max_us <= min_us) {
        return;
    }
    servo_config[servo].pulse_min_us = min_us;
    servo_config[servo].pulse_max_us = max_us;
    pico_servo_move_angle(servo, pico_servo_handles[servo].last_angle_deg);
}

float pico_servo_get_home_angle(servo_E servo)
{
    if (servo >= NUM_SERVOS) {
        return 0.0f;
    }
    return servo_config[servo].home_angle;
}

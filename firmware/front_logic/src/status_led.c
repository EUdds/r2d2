#include "status_led.h"
#include <hardware/gpio.h>
#include <pico/time.h>

typedef struct {
    uint32_t pin;
} status_led_t;

static const status_led_t status_led_app = {
    .pin = PICO_DEFAULT_LED_PIN,
};

void status_led_init(void)
{
    gpio_init(status_led_app.pin);
    gpio_set_dir(status_led_app.pin, GPIO_OUT);
}

void status_led_toggle(void)
{
    bool current_state = gpio_get(status_led_app.pin);
    gpio_put(status_led_app.pin, !current_state);

}

void fatal_blink(uint32_t period)
{
    while (true) {
        gpio_put(status_led_app.pin, true);
        sleep_ms(period / 2);
        gpio_put(status_led_app.pin, false);
        sleep_ms(period / 2);
    }
}
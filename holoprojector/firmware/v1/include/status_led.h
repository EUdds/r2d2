#pragma once

#include "stdint.h"


void status_led_init(void);
void status_led_toggle(void);
void fatal_blink(uint32_t period);
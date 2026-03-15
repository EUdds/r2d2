#pragma once

#include <stdbool.h>

bool can_receiver_init(void);
void can_receiver_poll(void);
void can_send_heartbeat(void);

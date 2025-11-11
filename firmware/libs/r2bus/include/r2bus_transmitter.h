#pragma once

#include "r2bus.h"

/**
 * Publish a heartbeat message using the active R2 bus context.
 * The call is ignored if the bus has not been initialised yet.
 */
void r2bus_publish_heartbeat(void);

/**
 * Attempt to flush any pending transmitter work (currently just heartbeats).
 * Passing NULL uses the active bus context resolved via r2bus_init().
 */
void r2bus_transmit(r2bus_ctx_t *ctx);

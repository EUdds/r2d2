#pragma once

#include "r2bus.h"

/**
 * Publish a heartbeat message using the active R2 bus context.
 * The call is ignored if the bus has not been initialised yet.
 */
void r2bus_publish_heartbeat(void);

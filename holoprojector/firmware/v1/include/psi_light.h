#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PSI_LIGHT_HAPPINESS_MIN 0u
#define PSI_LIGHT_HAPPINESS_MAX 100u

/**
 * Initialize the front PSI light animation state and underlying LED driver handle.
 * Returns true if initialization succeeded.
 */
bool psi_light_init(void);

/**
 * Advance the PSI light animation. Should be called periodically
 * at a reasonably high rate (e.g. 50-100 Hz) for smooth fades.
 */
void psi_light_update(void);

/**
 * Set the happiness level used to bias the red/blue mix.
 * Higher values shift the animation toward blue, lower toward red.
 */
void psi_light_set_happiness(uint32_t happiness_level);

/**
 * Retrieve the currently stored happiness level.
 */
uint32_t psi_light_get_happiness(void);

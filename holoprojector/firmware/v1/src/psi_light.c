#include "psi_light.h"

#include "ws2812.h"

#include <math.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdlib.h>

#define PSI_PIXEL_COUNT 16u
#define PSI_MAX_BRIGHT 100u

#define PSI_BLUE_R 10u
#define PSI_BLUE_G 40u
#define PSI_BLUE_B 255u

#define PSI_RED_R 255u
#define PSI_RED_G 20u
#define PSI_RED_B 10u

#define PSI_SHIMMER_AMPL 3
#define PSI_SHIMMER_RATE_MS 55u

#define PSI_FADE_UP_MS_BASE 1600u
#define PSI_FADE_DOWN_MS_BASE 1200u
#define PSI_HOLD_MIN_MS 200u
#define PSI_HOLD_MAX_MS 700u

#define PSI_VIGNETTE_STRENGTH 0.25f
#define PSI_PI 3.14159265358979323846f

#define PSI_HAPPINESS_BIAS_FACTOR 1.0f

#define PSI_LIGHT_HAPPINESS_DEFAULT ((PSI_LIGHT_HAPPINESS_MAX - PSI_LIGHT_HAPPINESS_MIN) / 2u)

typedef enum
{
    PSI_PHASE_FADE = 0,
    PSI_PHASE_HOLD,
} psi_phase_t;

typedef struct
{
    ws2812_handle_t *ws_handle;
    psi_phase_t phase;
    bool fading_to_blue;
    uint32_t phase_start_ms;
    uint32_t phase_duration_ms;
    uint32_t last_shimmer_ms;
    int8_t shimmer_offset;
    uint16_t fade_up_ms;
    uint16_t fade_down_ms;
    float vignette[PSI_PIXEL_COUNT];
    uint32_t happiness_level;
    bool initialized;
} psi_light_state_t;

static psi_light_state_t g_state = {
    .ws_handle = NULL,
    .phase = PSI_PHASE_FADE,
    .fading_to_blue = true,
    .phase_start_ms = 0u,
    .phase_duration_ms = PSI_FADE_UP_MS_BASE,
    .last_shimmer_ms = 0u,
    .shimmer_offset = 0,
    .fade_up_ms = PSI_FADE_UP_MS_BASE,
    .fade_down_ms = PSI_FADE_DOWN_MS_BASE,
    .vignette = {0},
    .happiness_level = PSI_LIGHT_HAPPINESS_DEFAULT,
    .initialized = false,
};

static inline float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static inline uint8_t clamp_to_u8(float value)
{
    value = clampf(value, 0.0f, 255.0f);
    return (uint8_t)(value + 0.5f);
}

static uint32_t psi_light_now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static float psi_light_happiness_norm(void)
{
    if (PSI_LIGHT_HAPPINESS_MAX == PSI_LIGHT_HAPPINESS_MIN)
    {
        return 0.5f;
    }
    float denom = (float)(PSI_LIGHT_HAPPINESS_MAX - PSI_LIGHT_HAPPINESS_MIN);
    float normalized = ((float)g_state.happiness_level - (float)PSI_LIGHT_HAPPINESS_MIN) / denom;
    return clampf(normalized, 0.0f, 1.0f);
}

static inline float psi_light_shimmer_gain(void)
{
    if (PSI_SHIMMER_AMPL == 0)
    {
        return 0.0f;
    }
    return (float)g_state.shimmer_offset / 255.0f;
}

static float psi_light_breath_curve(float t01, bool toward_blue)
{
    t01 = clampf(t01, 0.0f, 1.0f);
    float cosine_term = 0.5f * (1.0f - cosf(t01 * PSI_PI));
    if (toward_blue)
    {
        float eased = 4.0f * t01 * t01 * t01;
        cosine_term = 0.75f * cosine_term + 0.25f * eased;
    }
    else
    {
        cosine_term = 0.55f * cosine_term + 0.45f * t01;
    }
    return clampf(cosine_term, 0.0f, 1.0f);
}

static void psi_light_setup_vignette(void)
{
    float center = ((float)PSI_PIXEL_COUNT / 2.0f) + 0.5f;
    for (uint32_t i = 0; i < PSI_PIXEL_COUNT; ++i)
    {
        float distance = fabsf((float)i - center) / ((float)PSI_PIXEL_COUNT / 2.0f);
        distance = clampf(distance, 0.0f, 1.0f);
        g_state.vignette[i] = distance * distance;
    }
}

static void psi_light_pick_next_fade_durations(void)
{
    float jitter_up = 0.90f + (float)rand() / (float)RAND_MAX * 0.20f;
    float jitter_down = 0.90f + (float)rand() / (float)RAND_MAX * 0.20f;
    g_state.fade_up_ms = (uint16_t)((float)PSI_FADE_UP_MS_BASE * jitter_up);
    g_state.fade_down_ms = (uint16_t)((float)PSI_FADE_DOWN_MS_BASE * jitter_down);
    if (g_state.fade_up_ms == 0)
    {
        g_state.fade_up_ms = PSI_FADE_UP_MS_BASE;
    }
    if (g_state.fade_down_ms == 0)
    {
        g_state.fade_down_ms = PSI_FADE_DOWN_MS_BASE;
    }
}

static uint16_t psi_light_random_range_u16(uint16_t min_val, uint16_t max_val)
{
    if (max_val <= min_val)
    {
        return min_val;
    }
    uint32_t span = (uint32_t)max_val - (uint32_t)min_val + 1u;
    return (uint16_t)(min_val + (uint16_t)(rand() % span));
}

static void psi_light_begin_fade(bool toward_blue, uint32_t now_ms)
{
    g_state.fading_to_blue = toward_blue;
    g_state.phase = PSI_PHASE_FADE;
    g_state.phase_start_ms = now_ms;
    g_state.phase_duration_ms = toward_blue ? g_state.fade_up_ms : g_state.fade_down_ms;
    if (g_state.phase_duration_ms == 0)
    {
        g_state.phase_duration_ms = toward_blue ? PSI_FADE_UP_MS_BASE : PSI_FADE_DOWN_MS_BASE;
    }
}

static void psi_light_begin_hold(uint32_t now_ms)
{
    g_state.phase = PSI_PHASE_HOLD;
    g_state.phase_start_ms = now_ms;
    g_state.phase_duration_ms = psi_light_random_range_u16(PSI_HOLD_MIN_MS, PSI_HOLD_MAX_MS);
}

static void psi_light_apply_frame(float fade_mix)
{
    float happiness_norm = psi_light_happiness_norm();
    float bias = (happiness_norm - 0.5f) * PSI_HAPPINESS_BIAS_FACTOR;
    float mix = clampf(fade_mix + bias, 0.0f, 1.0f);

    float red = (float)PSI_RED_R * (1.0f - mix) + (float)PSI_BLUE_R * mix;
    float green = (float)PSI_RED_G * (1.0f - mix) + (float)PSI_BLUE_G * mix;
    float blue = (float)PSI_RED_B * (1.0f - mix) + (float)PSI_BLUE_B * mix;

    float brightness_base = (float)PSI_MAX_BRIGHT / 255.0f;
    float shimmer = psi_light_shimmer_gain();
    float brightness_scale = clampf(brightness_base + shimmer, 0.0f, 1.0f);

    for (uint32_t i = 0; i < PSI_PIXEL_COUNT; ++i)
    {
        float vignette_scale = clampf(1.0f - PSI_VIGNETTE_STRENGTH * g_state.vignette[i], 0.0f, 1.0f);
        float scale = clampf(brightness_scale * vignette_scale, 0.0f, 1.0f);
        uint8_t r = clamp_to_u8(red * scale);
        uint8_t g = clamp_to_u8(green * scale);
        uint8_t b = clamp_to_u8(blue * scale);
        put_pixel(FRONT_PSI_STRIP, urgb_u32(r, g, b));
    }
}

static void psi_light_update_shimmer(uint32_t now_ms)
{
    if (PSI_SHIMMER_AMPL == 0)
    {
        return;
    }
    if ((uint32_t)(now_ms - g_state.last_shimmer_ms) < PSI_SHIMMER_RATE_MS)
    {
        return;
    }
    g_state.last_shimmer_ms = now_ms;
    int step = (rand() % 3) - 1;
    int new_offset = g_state.shimmer_offset + step;
    if (new_offset > PSI_SHIMMER_AMPL)
    {
        new_offset = PSI_SHIMMER_AMPL;
    }
    if (new_offset < -PSI_SHIMMER_AMPL)
    {
        new_offset = -PSI_SHIMMER_AMPL;
    }
    g_state.shimmer_offset = (int8_t)new_offset;
}

bool psi_light_init(void)
{
    if (!g_state.initialized)
    {
        srand((unsigned int)time_us_64());
    }

    ws2812_handle_t *handle = ws2812_get_handle(FRONT_PSI_STRIP);
    if (!handle)
    {
        return false;
    }
    g_state.ws_handle = handle;

    psi_light_setup_vignette();
    psi_light_pick_next_fade_durations();

    uint32_t now_ms = psi_light_now_ms();
    g_state.phase_start_ms = now_ms;
    g_state.last_shimmer_ms = now_ms;
    g_state.shimmer_offset = 0;
    g_state.happiness_level = PSI_LIGHT_HAPPINESS_DEFAULT;

    psi_light_begin_fade(true, now_ms);
    psi_light_apply_frame(0.0f);

    g_state.initialized = true;
    return true;
}

void psi_light_update(void)
{
    if (!g_state.initialized || !g_state.ws_handle)
    {
        return;
    }

    uint32_t now_ms = psi_light_now_ms();
    psi_light_update_shimmer(now_ms);

    if (g_state.phase == PSI_PHASE_FADE)
    {
        uint32_t elapsed = now_ms - g_state.phase_start_ms;
        if (elapsed >= g_state.phase_duration_ms)
        {
            float mix = g_state.fading_to_blue ? 1.0f : 0.0f;
            psi_light_apply_frame(mix);
            psi_light_begin_hold(now_ms);
        }
        else
        {
            float t = (float)elapsed / (float)g_state.phase_duration_ms;
            float curve = psi_light_breath_curve(t, g_state.fading_to_blue);
            float mix = g_state.fading_to_blue ? curve : (1.0f - curve);
            psi_light_apply_frame(mix);
        }
    }
    else
    {
        uint32_t elapsed = now_ms - g_state.phase_start_ms;
        if (elapsed >= g_state.phase_duration_ms)
        {
            psi_light_pick_next_fade_durations();
            psi_light_begin_fade(!g_state.fading_to_blue, now_ms);
        }
        else
        {
            float mix = g_state.fading_to_blue ? 1.0f : 0.0f;
            psi_light_apply_frame(mix);
        }
    }
}

void psi_light_set_happiness(uint32_t happiness_level)
{
    if (happiness_level < PSI_LIGHT_HAPPINESS_MIN)
    {
        happiness_level = PSI_LIGHT_HAPPINESS_MIN;
    }
    if (happiness_level > PSI_LIGHT_HAPPINESS_MAX)
    {
        happiness_level = PSI_LIGHT_HAPPINESS_MAX;
    }
    g_state.happiness_level = happiness_level;
}

uint32_t psi_light_get_happiness(void)
{
    return g_state.happiness_level;
}

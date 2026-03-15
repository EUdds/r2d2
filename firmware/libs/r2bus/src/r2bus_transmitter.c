#include "r2bus_transmitter.h"

#include "r2bus.h"
#include "r2bus_proto.h"
#include <pico/time.h>

#define R2BUS_HEARTBEAT_IDLE_GUARD_US 2000u
#define R2BUS_HEARTBEAT_MIN_BACKOFF_US 500u
#define R2BUS_HEARTBEAT_MAX_BACKOFF_US 3000u

static bool g_heartbeat_pending = false;
static absolute_time_t g_heartbeat_ready_time;

static void publish_heartbeat_ctx(r2bus_ctx_t *bus) {
    r2bus_Heartbeat heartbeat = r2bus_Heartbeat_init_default;
    heartbeat.uptime_ms = to_ms_since_boot(get_absolute_time());
    (void)r2bus_send_proto(bus,
                           R2BUS_HOST_ID,
                           R2BUS_MSG_HEARTBEAT,
                           r2bus_Heartbeat_fields,
                           &heartbeat);
}

static uint32_t r2bus_rand32(void) {
    static uint32_t lfsr = 0;
    if (lfsr == 0) {
        lfsr = (uint32_t)time_us_64() | 1u;
    }
    lfsr ^= lfsr << 13;
    lfsr ^= lfsr >> 17;
    lfsr ^= lfsr << 5;
    return lfsr;
}

static absolute_time_t r2bus_next_attempt_deadline(void) {
    const uint32_t span = R2BUS_HEARTBEAT_MAX_BACKOFF_US - R2BUS_HEARTBEAT_MIN_BACKOFF_US;
    uint32_t jitter = R2BUS_HEARTBEAT_MIN_BACKOFF_US;
    if (span > 0) {
        jitter += r2bus_rand32() % (span + 1u);
    }
    return make_timeout_time_us(jitter);
}

static void r2bus_schedule_heartbeat_retry(void) {
    g_heartbeat_ready_time = r2bus_next_attempt_deadline();
}

static bool r2bus_bus_idle_for_guard(r2bus_ctx_t *ctx) {
    if (!ctx || !ctx->bus) {
        return false;
    }
    return rs485_bus_idle_for(ctx->bus, R2BUS_HEARTBEAT_IDLE_GUARD_US);
}

void r2bus_publish_heartbeat(void) {
    r2bus_ctx_t *ctx = r2bus_get_active_context();
    if (!ctx) {
        return;
    }
    g_heartbeat_pending = true;
    r2bus_schedule_heartbeat_retry();
    r2bus_transmit(ctx);
}



/**
 * Attempt to transmit pending heartbeat traffic once the bus has been idle for
 * a guard period. Re-schedules itself with a randomised backoff if the line
 * stays busy so firmware updates can run without collisions.
 */
void r2bus_transmit(r2bus_ctx_t *ctx) {
    if (!g_heartbeat_pending) {
        return;
    }
    r2bus_ctx_t *context = ctx ? ctx : r2bus_get_active_context();
    if (!context) {
        return;
    }
    if (!time_reached(g_heartbeat_ready_time)) {
        return;
    }
    if (!r2bus_bus_idle_for_guard(context)) {
        r2bus_schedule_heartbeat_retry();
        return;
    }
    publish_heartbeat_ctx(context);
    g_heartbeat_pending = false;
}

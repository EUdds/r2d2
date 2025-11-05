#include "r2bus_transmitter.h"

#include "r2bus_proto.h"
#include <pico/time.h>


static void publish_heartbeat_ctx(r2bus_ctx_t *bus) {
    r2bus_Heartbeat heartbeat = r2bus_Heartbeat_init_default;
    heartbeat.uptime_ms = to_ms_since_boot(get_absolute_time());
    (void)r2bus_send_proto(bus,
                           R2BUS_HOST_ID,
                           R2BUS_MSG_HEARTBEAT,
                           r2bus_Heartbeat_fields,
                           &heartbeat);
}

void r2bus_publish_heartbeat(void) {
    r2bus_ctx_t *ctx = r2bus_get_active_context();
    if (!ctx) {
        return;
    }
    publish_heartbeat_ctx(ctx);
}

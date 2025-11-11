#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pb.h"

#include "r2bus.h"

bool r2bus_send_proto(r2bus_ctx_t *ctx,
                      uint8_t dest_id,
                      r2bus_msg_id_E msg_id,
                      const pb_msgdesc_t *fields,
                      const void *src_struct);

bool r2bus_decode_proto(const r2bus_packet_t *packet,
                        const pb_msgdesc_t *fields,
                        void *dst_struct);

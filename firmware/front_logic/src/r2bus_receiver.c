#include "r2bus_receiver.h"
#include "r2bus.h"
#include "r2bus_proto.h"
#include "r2bus.pb.h"

void handle_r2bus_packet_componentSpecific(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    switch (packet->msg_id) {
        case R2BUS_MSG_ACK:
        case R2BUS_MSG_PING:
        case R2BUS_MSG_HEARTBEAT:
        default:
            break;
    }
}
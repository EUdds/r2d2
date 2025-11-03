#include "r2bus_receiver.h"

#include "r2bus.h"
#include "r2bus_proto.h"
#include "r2bus.pb.h"

#include "pico_servo.h"


static void handle_servo_move_command(const r2bus_packet_t *packet, void *user_data);

static void handle_servo_move_command(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    if (packet->msg_id != R2BUS_MSG_SERVO_MOVE_CMD) {
        return;
    }
    r2bus_ServoMoveCommand command = r2bus_ServoMoveCommand_init_default;
    if (!r2bus_decode_proto(packet, r2bus_ServoMoveCommand_fields, &command)) {
        return;
    }
    if (command.servo < NUM_SERVOS) {
        pico_servo_move_angle((servo_E)command.servo, command.position_deg);
    }
}

void handle_r2bus_packet_componentSpecific(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    switch (packet->msg_id) {
        case R2BUS_MSG_ACK:
        case R2BUS_MSG_PING:
        case R2BUS_MSG_HEARTBEAT:
        case R2BUS_MSG_PSI_COLOR_REQ:
            // No application-specific handling yet.
            break;
        case R2BUS_MSG_SERVO_MOVE_CMD:
            {
                handle_servo_move_command(packet, user_data);
            }
            break;
        default:
            break;
    }
}
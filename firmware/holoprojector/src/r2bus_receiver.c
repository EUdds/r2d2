#include "r2bus_receiver.h"

#include "holoprojector_lights.h"
#include "r2bus.h"
#include "r2bus_proto.h"
#include "r2bus.pb.h"

#include "pico_servo.h"
#include "psi_light.h"


static void handle_servo_move_command(const r2bus_packet_t *packet, void *user_data);
static void handle_servo_home_command(const r2bus_packet_t *packet, void *user_data);
static void handle_holo_color_request(const r2bus_packet_t *packet, void *user_data);
static void handle_psi_color_request(const r2bus_packet_t *packet, void *user_data);
static void handle_psi_modd_request(const r2bus_packet_t *packet, void *user_data);
static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value);

static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
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
        if (command.duration_ms > 0) {
            // Non-zero duration_ms is repurposed as a raw pulse width request (microseconds).
            pico_servo_move_pulse_us((servo_E)command.servo, (float)command.duration_ms);
        } else {
            pico_servo_move_angle((servo_E)command.servo, command.position_deg);
        }
    }
}

static void handle_servo_home_command(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    if (packet->msg_id != R2BUS_MSG_SERVO_HOME_CMD) {
        return;
    }
    r2bus_ServoHomeCommand command = r2bus_ServoHomeCommand_init_default;
    if (!r2bus_decode_proto(packet, r2bus_ServoHomeCommand_fields, &command)) {
        return;
    }
    if (command.all) {
        pico_servo_home();
        return;
    }
    if (command.servo < NUM_SERVOS) {
        pico_servo_home_servo((servo_E)command.servo);
    }
}

static void handle_holo_color_request(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    if (packet->msg_id != R2BUS_MSG_HOLO_COLOR_REQ) {
        return;
    }
    r2bus_HoloColorRequest color_req = r2bus_HoloColorRequest_init_default;
    if (!r2bus_decode_proto(packet, r2bus_HoloColorRequest_fields, &color_req)) {
        return;
    }

    holoprojector_lights_set_color(color_req.red, color_req.green, color_req.blue);
    holoprojector_lights_set_brightness(color_req.brightness);
}

static void handle_psi_color_request(const r2bus_packet_t *packet, void *user_data)
{
    (void)user_data;
    if (packet->msg_id != R2BUS_MSG_PSI_COLOR_REQ) {
        return;
    }

    r2bus_PsiColorRequest color_req = r2bus_PsiColorRequest_init_default;
    if (!r2bus_decode_proto(packet, r2bus_PsiColorRequest_fields, &color_req)) {
        return;
    }

    uint32_t red = color_req.red;
    uint32_t blue = color_req.blue;
    uint32_t total = red + blue;
    if (total == 0) {
        return;
    }

    uint32_t happiness = (uint32_t)((blue * PSI_LIGHT_HAPPINESS_MAX + (total / 2u)) / total);
    happiness = clamp_u32(happiness, PSI_LIGHT_HAPPINESS_MIN, PSI_LIGHT_HAPPINESS_MAX);
    psi_light_set_happiness(happiness);
}

void handle_r2bus_packet_componentSpecific(const r2bus_packet_t *packet, void *user_data) {
    (void)user_data;
    switch (packet->msg_id) {
        case R2BUS_MSG_ACK:
        case R2BUS_MSG_PING:
        case R2BUS_MSG_HEARTBEAT:
        case R2BUS_MSG_PSI_COLOR_REQ:
            handle_psi_color_request(packet, user_data);
            break;
        case R2BUS_MSG_SERVO_MOVE_CMD:
            {
                handle_servo_move_command(packet, user_data);
            }
            break;
        case R2BUS_MSG_SERVO_HOME_CMD:
            {
                handle_servo_home_command(packet, user_data);
            }
            break;
        
        case R2BUS_MSG_HOLO_COLOR_REQ:
            {
                handle_holo_color_request(packet, user_data);
            }
            break;
        case R2BUS_MSG_HOLO_POSITION_REQ:
            {
                r2bus_HoloPositionRequest pos_req = r2bus_HoloPositionRequest_init_default;
                if (!r2bus_decode_proto(packet, r2bus_HoloPositionRequest_fields, &pos_req)) {
                    return;
                }
                holoprojector_servo_set_angles((uint8_t)pos_req.azimuth_deg, (uint8_t)pos_req.elevation_deg);
            }
            break;
        case R2BUS_MSG_PSI_MOOD_REQ:
            {
                r2bus_PsiMoodRequest mood_req = r2bus_PsiMoodRequest_init_default;
                if (!r2bus_decode_proto(packet, r2bus_PsiMoodRequest_fields, &mood_req)) {
                    return;
                }
                uint32_t happiness = clamp_u32(mood_req.mood, PSI_LIGHT_HAPPINESS_MIN, PSI_LIGHT_HAPPINESS_MAX);
                psi_light_set_happiness(happiness);
            }
        default:
            break;
    }
}

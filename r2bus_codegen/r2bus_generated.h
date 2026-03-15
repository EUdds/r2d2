#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if __has_include("r2bus.h")
#include "r2bus.h"
#define R2BUS_GENERATED_HAS_R2BUS 1
#else
#define R2BUS_GENERATED_HAS_R2BUS 0
#endif

#define R2BUS_GENERATED_TX_QUEUE_SIZE 16
#define R2BUS_GENERATED_MAX_PAYLOAD 255u
#define R2BUS_GENERATED_NODE_COUNT 2u

typedef struct {
    const char *name;
    uint8_t node_id;
} r2bus_generated_NodeInfo;

extern const r2bus_generated_NodeInfo r2bus_generated_nodes[R2BUS_GENERATED_NODE_COUNT];

#if R2BUS_GENERATED_HAS_R2BUS
typedef r2bus_status_E r2bus_generated_Status_E;
typedef r2bus_msg_id_E r2bus_generated_MessageId_E;
#endif

#if !R2BUS_GENERATED_HAS_R2BUS
typedef enum {
    R2BUS_MSG_ECU_RESET = 1,
    R2BUS_MSG_PING = 2,
    R2BUS_MSG_PONG = 3,
    R2BUS_MSG_HEARTBEAT = 4,
    R2BUS_MSG_PSI_COLOR_REQ = 5,
    R2BUS_MSG_SERVO_MOVE_CMD = 6,
    R2BUS_MSG_SERVO_HOME_CMD = 7,
    R2BUS_MSG_HOLO_COLOR_REQ = 8,
    R2BUS_MSG_HOLO_POSITION_REQ = 9,
    R2BUS_MSG_PSI_MOOD_REQ = 10,
    R2BUS_MSG_ACK = 127,
} r2bus_msg_id_E;

typedef enum {
    R2BUS_STATUS_OK = 0,
    R2BUS_STATUS_ERROR = 1,
} r2bus_status_E;

typedef r2bus_status_E r2bus_generated_Status_E;
typedef r2bus_msg_id_E r2bus_generated_MessageId_E;

typedef struct {
    uint8_t dest_id;
    uint8_t src_id;
    r2bus_msg_id_E msg_id;
    uint8_t length;
    uint8_t payload[R2BUS_GENERATED_MAX_PAYLOAD];
} r2bus_packet_t;
#endif

typedef struct {
    uint8_t _unused;
} r2bus_generated_Empty;

typedef struct {
    size_t payload_size;
    uint8_t payload_data[255];
} r2bus_generated_Ping;

typedef struct {
    size_t payload_size;
    uint8_t payload_data[255];
} r2bus_generated_Pong;

typedef struct {
    uint32_t uptime_ms;
} r2bus_generated_Heartbeat;

typedef struct {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    uint32_t brightness;
} r2bus_generated_PsiColorRequest;

typedef struct {
    uint32_t servo;
    float position_deg;
    uint32_t duration_ms;
} r2bus_generated_ServoMoveCommand;

typedef struct {
    uint32_t servo;
    bool all;
} r2bus_generated_ServoHomeCommand;

typedef struct {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    uint32_t brightness;
} r2bus_generated_HoloColorRequest;

typedef struct {
    float azimuth_deg;
    float elevation_deg;
} r2bus_generated_HoloPositionRequest;

typedef struct {
    uint32_t mood;
} r2bus_generated_PsiMoodRequest;

typedef struct {
    r2bus_generated_Status_E status;
    r2bus_generated_MessageId_E original_msg;
} r2bus_generated_Ack;

typedef struct {
    uint8_t dest_id;
    r2bus_msg_id_E msg_id;
    uint8_t payload[R2BUS_GENERATED_MAX_PAYLOAD];
    size_t payload_len;
} r2bus_tx_frame_t;

bool r2bus_send_Empty(uint8_t dest_id, const r2bus_generated_Empty *payload);
bool r2bus_send_Ping(uint8_t dest_id, const r2bus_generated_Ping *payload);
bool r2bus_send_Pong(uint8_t dest_id, const r2bus_generated_Pong *payload);
bool r2bus_send_Heartbeat(uint8_t dest_id, const r2bus_generated_Heartbeat *payload);
bool r2bus_send_PsiColorRequest(uint8_t dest_id, const r2bus_generated_PsiColorRequest *payload);
bool r2bus_send_ServoMoveCommand(uint8_t dest_id, const r2bus_generated_ServoMoveCommand *payload);
bool r2bus_send_ServoHomeCommand(uint8_t dest_id, const r2bus_generated_ServoHomeCommand *payload);
bool r2bus_send_HoloColorRequest(uint8_t dest_id, const r2bus_generated_HoloColorRequest *payload);
bool r2bus_send_HoloPositionRequest(uint8_t dest_id, const r2bus_generated_HoloPositionRequest *payload);
bool r2bus_send_PsiMoodRequest(uint8_t dest_id, const r2bus_generated_PsiMoodRequest *payload);

bool r2bus_get_Empty(uint8_t src_id, r2bus_generated_Empty *out, uint32_t now_ms);
bool r2bus_get_Ping(uint8_t src_id, r2bus_generated_Ping *out, uint32_t now_ms);
bool r2bus_get_Pong(uint8_t src_id, r2bus_generated_Pong *out, uint32_t now_ms);
bool r2bus_get_Heartbeat(uint8_t src_id, r2bus_generated_Heartbeat *out, uint32_t now_ms);
bool r2bus_get_PsiColorRequest(uint8_t src_id, r2bus_generated_PsiColorRequest *out, uint32_t now_ms);
bool r2bus_get_ServoMoveCommand(uint8_t src_id, r2bus_generated_ServoMoveCommand *out, uint32_t now_ms);
bool r2bus_get_ServoHomeCommand(uint8_t src_id, r2bus_generated_ServoHomeCommand *out, uint32_t now_ms);
bool r2bus_get_HoloColorRequest(uint8_t src_id, r2bus_generated_HoloColorRequest *out, uint32_t now_ms);
bool r2bus_get_HoloPositionRequest(uint8_t src_id, r2bus_generated_HoloPositionRequest *out, uint32_t now_ms);
bool r2bus_get_PsiMoodRequest(uint8_t src_id, r2bus_generated_PsiMoodRequest *out, uint32_t now_ms);

bool r2bus_dcfront_getEmpty(r2bus_generated_Empty *out, uint32_t now_ms);
bool r2bus_dcfront_getPing(r2bus_generated_Ping *out, uint32_t now_ms);
bool r2bus_dcfront_getPong(r2bus_generated_Pong *out, uint32_t now_ms);
bool r2bus_dcfront_getHeartbeat(r2bus_generated_Heartbeat *out, uint32_t now_ms);
bool r2bus_dcfront_getPsiColorRequest(r2bus_generated_PsiColorRequest *out, uint32_t now_ms);
bool r2bus_dcfront_getServoMoveCommand(r2bus_generated_ServoMoveCommand *out, uint32_t now_ms);
bool r2bus_dcfront_getServoHomeCommand(r2bus_generated_ServoHomeCommand *out, uint32_t now_ms);
bool r2bus_dcfront_getHoloColorRequest(r2bus_generated_HoloColorRequest *out, uint32_t now_ms);
bool r2bus_dcfront_getHoloPositionRequest(r2bus_generated_HoloPositionRequest *out, uint32_t now_ms);
bool r2bus_dcfront_getPsiMoodRequest(r2bus_generated_PsiMoodRequest *out, uint32_t now_ms);
bool r2bus_dcrear_getEmpty(r2bus_generated_Empty *out, uint32_t now_ms);
bool r2bus_dcrear_getPing(r2bus_generated_Ping *out, uint32_t now_ms);
bool r2bus_dcrear_getPong(r2bus_generated_Pong *out, uint32_t now_ms);
bool r2bus_dcrear_getHeartbeat(r2bus_generated_Heartbeat *out, uint32_t now_ms);
bool r2bus_dcrear_getPsiColorRequest(r2bus_generated_PsiColorRequest *out, uint32_t now_ms);
bool r2bus_dcrear_getServoMoveCommand(r2bus_generated_ServoMoveCommand *out, uint32_t now_ms);
bool r2bus_dcrear_getServoHomeCommand(r2bus_generated_ServoHomeCommand *out, uint32_t now_ms);
bool r2bus_dcrear_getHoloColorRequest(r2bus_generated_HoloColorRequest *out, uint32_t now_ms);
bool r2bus_dcrear_getHoloPositionRequest(r2bus_generated_HoloPositionRequest *out, uint32_t now_ms);
bool r2bus_dcrear_getPsiMoodRequest(r2bus_generated_PsiMoodRequest *out, uint32_t now_ms);

bool r2bus_handle_packet(const r2bus_packet_t *packet, uint32_t now_ms);
bool r2bus_tx_pop(r2bus_tx_frame_t *out);

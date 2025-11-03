/* Hand-authored nanopb header matching r2bus.proto definitions. */

#ifndef PB_R2BUS_PROTO_R2BUS_PB_H_INCLUDED
#define PB_R2BUS_PROTO_R2BUS_PB_H_INCLUDED
#include "pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#define R2BUS_MAX_PROTO_PAYLOAD 255

/* Enum definitions */
typedef enum _r2bus_Status {
    r2bus_Status_STATUS_OK = 0,
    r2bus_Status_STATUS_ERROR = 1
} r2bus_Status;

typedef enum _r2bus_MessageId {
    r2bus_MessageId_MESSAGE_ID_UNKNOWN = 0,
    r2bus_MessageId_MESSAGE_ID_ECU_RESET = 0x01,
    r2bus_MessageId_MESSAGE_ID_PING = 0x02,
    r2bus_MessageId_MESSAGE_ID_PONG = 0x03,
    r2bus_MessageId_MESSAGE_ID_HEARTBEAT = 0x04,
    r2bus_MessageId_MESSAGE_ID_PSI_COLOR_REQ = 0x05,
    r2bus_MessageId_MESSAGE_ID_SERVO_MOVE_CMD = 0x06,
    r2bus_MessageId_MESSAGE_ID_ACK = 0x7F
} r2bus_MessageId;

/* Struct definitions */
typedef struct _r2bus_Ack {
    r2bus_Status status;
    r2bus_MessageId original_msg;
} r2bus_Ack;

typedef struct _r2bus_Ping {
    PB_BYTES_ARRAY_T(R2BUS_MAX_PROTO_PAYLOAD) payload;
} r2bus_Ping;

typedef struct _r2bus_Pong {
    PB_BYTES_ARRAY_T(R2BUS_MAX_PROTO_PAYLOAD) payload;
} r2bus_Pong;

typedef struct _r2bus_Heartbeat {
    uint32_t uptime_ms;
} r2bus_Heartbeat;

typedef struct _r2bus_PsiColorRequest {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
} r2bus_PsiColorRequest;

typedef struct _r2bus_ServoMoveCommand {
    uint32_t servo;
    float position_deg;
    uint32_t duration_ms;
} r2bus_ServoMoveCommand;

typedef struct _r2bus_Empty {
    uint8_t _dummy;
} r2bus_Empty;

#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define r2bus_Ack_init_default                {r2bus_Status_STATUS_OK, r2bus_MessageId_MESSAGE_ID_UNKNOWN}
#define r2bus_Ping_init_default               { {0, {0}} }
#define r2bus_Pong_init_default               { {0, {0}} }
#define r2bus_Heartbeat_init_default          {0}
#define r2bus_PsiColorRequest_init_default    {0, 0, 0}
#define r2bus_ServoMoveCommand_init_default   {0, 0.0f, 0}
#define r2bus_Empty_init_default              {0}

#define r2bus_Ack_init_zero                   {r2bus_Status_STATUS_OK, r2bus_MessageId_MESSAGE_ID_UNKNOWN}
#define r2bus_Ping_init_zero                  { {0, {0}} }
#define r2bus_Pong_init_zero                  { {0, {0}} }
#define r2bus_Heartbeat_init_zero             {0}
#define r2bus_PsiColorRequest_init_zero       {0, 0, 0}
#define r2bus_ServoMoveCommand_init_zero      {0, 0.0f, 0}
#define r2bus_Empty_init_zero                 {0}

/* Field tags (for use in manual encoding/decoding) */
#define r2bus_Ack_status_tag                  1
#define r2bus_Ack_original_msg_tag            2
#define r2bus_Ping_payload_tag                1
#define r2bus_Pong_payload_tag                1
#define r2bus_Heartbeat_uptime_ms_tag         1
#define r2bus_PsiColorRequest_red_tag         1
#define r2bus_PsiColorRequest_green_tag       2
#define r2bus_PsiColorRequest_blue_tag        3
#define r2bus_ServoMoveCommand_servo_tag      1
#define r2bus_ServoMoveCommand_position_deg_tag 2
#define r2bus_ServoMoveCommand_duration_ms_tag 3

/* Struct field encoding specification for nanopb */
#define r2bus_Ack_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, ENUM,     status,             1) \
X(a, STATIC,   SINGULAR, ENUM,     original_msg,       2)
#define r2bus_Ack_CALLBACK NULL
#define r2bus_Ack_DEFAULT NULL

#define r2bus_Ping_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BYTES,    payload,            1)
#define r2bus_Ping_CALLBACK NULL
#define r2bus_Ping_DEFAULT NULL

#define r2bus_Pong_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BYTES,    payload,            1)
#define r2bus_Pong_CALLBACK NULL
#define r2bus_Pong_DEFAULT NULL

#define r2bus_Heartbeat_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   uptime_ms,          1)
#define r2bus_Heartbeat_CALLBACK NULL
#define r2bus_Heartbeat_DEFAULT NULL

#define r2bus_PsiColorRequest_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   red,                1) \
X(a, STATIC,   SINGULAR, UINT32,   green,              2) \
X(a, STATIC,   SINGULAR, UINT32,   blue,               3)
#define r2bus_PsiColorRequest_CALLBACK NULL
#define r2bus_PsiColorRequest_DEFAULT NULL

#define r2bus_ServoMoveCommand_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   servo,              1) \
X(a, STATIC,   SINGULAR, FLOAT,    position_deg,       2) \
X(a, STATIC,   SINGULAR, UINT32,   duration_ms,        3)
#define r2bus_ServoMoveCommand_CALLBACK NULL
#define r2bus_ServoMoveCommand_DEFAULT NULL

#define r2bus_Empty_FIELDLIST(X, a)
#define r2bus_Empty_CALLBACK NULL
#define r2bus_Empty_DEFAULT NULL

extern const pb_msgdesc_t r2bus_Ack_msg;
extern const pb_msgdesc_t r2bus_Ping_msg;
extern const pb_msgdesc_t r2bus_Pong_msg;
extern const pb_msgdesc_t r2bus_Heartbeat_msg;
extern const pb_msgdesc_t r2bus_PsiColorRequest_msg;
extern const pb_msgdesc_t r2bus_ServoMoveCommand_msg;
extern const pb_msgdesc_t r2bus_Empty_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define r2bus_Ack_fields &r2bus_Ack_msg
#define r2bus_Ping_fields &r2bus_Ping_msg
#define r2bus_Pong_fields &r2bus_Pong_msg
#define r2bus_Heartbeat_fields &r2bus_Heartbeat_msg
#define r2bus_PsiColorRequest_fields &r2bus_PsiColorRequest_msg
#define r2bus_ServoMoveCommand_fields &r2bus_ServoMoveCommand_msg
#define r2bus_Empty_fields &r2bus_Empty_msg

/* Maximum encoded size of messages (where known) */
#define R2BUS_PROTO_R2BUS_PB_H_MAX_SIZE r2bus_Ping_size
#define r2bus_Ack_size                  4
#define r2bus_Ping_size                 (1 + 2 + R2BUS_MAX_PROTO_PAYLOAD)
#define r2bus_Pong_size                 (1 + 2 + R2BUS_MAX_PROTO_PAYLOAD)
#define r2bus_Heartbeat_size            6
#define r2bus_PsiColorRequest_size      15
#define r2bus_ServoMoveCommand_size     15
#define r2bus_Empty_size                0

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

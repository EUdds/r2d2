#include "r2bus_generated.h"

#include <string.h>

#define R2BUS_WIRE_VARINT 0
#define R2BUS_WIRE_64BIT 1
#define R2BUS_WIRE_LEN 2
#define R2BUS_WIRE_32BIT 5

static const uint8_t r2bus_generated_node_ids[R2BUS_GENERATED_NODE_COUNT] = {
    0x10,
    0x11,
};

const r2bus_generated_NodeInfo r2bus_generated_nodes[R2BUS_GENERATED_NODE_COUNT] = {
    {"dcfront", 0x10u},
    {"dcrear", 0x11u},
};

static int r2bus_generated_node_index(uint8_t node_id) {
    for (size_t i = 0; i < R2BUS_GENERATED_NODE_COUNT; ++i) {
        if (r2bus_generated_node_ids[i] == node_id) {
            return (int)i;
        }
    }
    return -1;
}

static bool r2bus_encode_varint(uint64_t value, uint8_t *out, size_t *offset, size_t max_len) {
    while (value >= 0x80) {
        if (*offset >= max_len) {
            return false;
        }
        out[(*offset)++] = (uint8_t)((value & 0x7Fu) | 0x80u);
        value >>= 7;
    }
    if (*offset >= max_len) {
        return false;
    }
    out[(*offset)++] = (uint8_t)value;
    return true;
}

static bool r2bus_decode_varint(const uint8_t *data, size_t len, size_t *offset, uint64_t *value) {
    uint64_t result = 0;
    uint32_t shift = 0;
    while (*offset < len && shift <= 63) {
        uint8_t byte = data[(*offset)++];
        result |= (uint64_t)(byte & 0x7Fu) << shift;
        if (!(byte & 0x80u)) {
            *value = result;
            return true;
        }
        shift += 7;
    }
    return false;
}

static uint32_t r2bus_encode_zigzag32(int32_t value) {
    return (uint32_t)((value << 1) ^ (value >> 31));
}

static uint64_t r2bus_encode_zigzag64(int64_t value) {
    return (uint64_t)((value << 1) ^ (value >> 63));
}

static int32_t r2bus_decode_zigzag32(uint32_t value) {
    return (int32_t)((value >> 1) ^ (uint32_t)(-(int32_t)(value & 1u)));
}

static int64_t r2bus_decode_zigzag64(uint64_t value) {
    return (int64_t)((value >> 1) ^ (uint64_t)(-(int64_t)(value & 1u)));
}

static bool r2bus_encode_tag(uint32_t field_number, uint8_t wire_type, uint8_t *out, size_t *offset, size_t max_len) {
    uint64_t tag = ((uint64_t)field_number << 3) | (uint64_t)wire_type;
    return r2bus_encode_varint(tag, out, offset, max_len);
}

static bool r2bus_skip_field(uint8_t wire_type, const uint8_t *data, size_t len, size_t *offset) {
    uint64_t value = 0;
    switch (wire_type) {
        case R2BUS_WIRE_VARINT:
            return r2bus_decode_varint(data, len, offset, &value);
        case R2BUS_WIRE_64BIT:
            if (*offset + 8 > len) {
                return false;
            }
            *offset += 8;
            return true;
        case R2BUS_WIRE_LEN:
            if (!r2bus_decode_varint(data, len, offset, &value)) {
                return false;
            }
            if (*offset + (size_t)value > len) {
                return false;
            }
            *offset += (size_t)value;
            return true;
        case R2BUS_WIRE_32BIT:
            if (*offset + 4 > len) {
                return false;
            }
            *offset += 4;
            return true;
        default:
            return false;
    }
}

static bool r2bus_encode_Empty(const r2bus_generated_Empty *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    *out_len = offset;
    return true;
}

static bool r2bus_decode_Empty(r2bus_generated_Empty *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_Ping(const r2bus_generated_Ping *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->payload_size > 255u) {
        return false;
    }
    if (msg->payload_size > 0) {
        if (!r2bus_encode_tag(1, R2BUS_WIRE_LEN, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint(msg->payload_size, out, &offset, max_len)) {
            return false;
        }
        if (offset + msg->payload_size > max_len) {
            return false;
        }
        memcpy(&out[offset], msg->payload_data, msg->payload_size);
        offset += msg->payload_size;
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_Ping(r2bus_generated_Ping *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 2) {
                    return false;
                }
                {
                    uint64_t length = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &length)) {
                        return false;
                    }
                    if (length > 255u) {
                        return false;
                    }
                    if (offset + (size_t)length > len) {
                        return false;
                    }
                    msg->payload_size = (size_t)length;
                    if (length > 0) {
                        memcpy(msg->payload_data, &data[offset], (size_t)length);
                    }
                    offset += (size_t)length;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_Pong(const r2bus_generated_Pong *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->payload_size > 255u) {
        return false;
    }
    if (msg->payload_size > 0) {
        if (!r2bus_encode_tag(1, R2BUS_WIRE_LEN, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint(msg->payload_size, out, &offset, max_len)) {
            return false;
        }
        if (offset + msg->payload_size > max_len) {
            return false;
        }
        memcpy(&out[offset], msg->payload_data, msg->payload_size);
        offset += msg->payload_size;
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_Pong(r2bus_generated_Pong *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 2) {
                    return false;
                }
                {
                    uint64_t length = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &length)) {
                        return false;
                    }
                    if (length > 255u) {
                        return false;
                    }
                    if (offset + (size_t)length > len) {
                        return false;
                    }
                    msg->payload_size = (size_t)length;
                    if (length > 0) {
                        memcpy(msg->payload_data, &data[offset], (size_t)length);
                    }
                    offset += (size_t)length;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_Heartbeat(const r2bus_generated_Heartbeat *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->uptime_ms != 0) {
        if (!r2bus_encode_tag(1, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->uptime_ms, out, &offset, max_len)) {
            return false;
        }
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_Heartbeat(r2bus_generated_Heartbeat *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->uptime_ms = (uint32_t)value;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_PsiColorRequest(const r2bus_generated_PsiColorRequest *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->red != 0) {
        if (!r2bus_encode_tag(1, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->red, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->green != 0) {
        if (!r2bus_encode_tag(2, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->green, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->blue != 0) {
        if (!r2bus_encode_tag(3, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->blue, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->brightness != 0) {
        if (!r2bus_encode_tag(4, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->brightness, out, &offset, max_len)) {
            return false;
        }
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_PsiColorRequest(r2bus_generated_PsiColorRequest *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->red = (uint32_t)value;
                }
                break;
            case 2:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->green = (uint32_t)value;
                }
                break;
            case 3:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->blue = (uint32_t)value;
                }
                break;
            case 4:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->brightness = (uint32_t)value;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_ServoMoveCommand(const r2bus_generated_ServoMoveCommand *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->servo != 0) {
        if (!r2bus_encode_tag(1, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->servo, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->position_deg != 0) {
        if (!r2bus_encode_tag(2, 5, out, &offset, max_len)) {
            return false;
        }
        if (offset + 4 > max_len) {
            return false;
        }
        memcpy(&out[offset], &msg->position_deg, 4);
        offset += 4;
    }
    if (msg->duration_ms != 0) {
        if (!r2bus_encode_tag(3, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->duration_ms, out, &offset, max_len)) {
            return false;
        }
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_ServoMoveCommand(r2bus_generated_ServoMoveCommand *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->servo = (uint32_t)value;
                }
                break;
            case 2:
                if (wire_type != 5) {
                    return false;
                }
                {
                    if (offset + 4 > len) {
                        return false;
                    }
                    memcpy(&msg->position_deg, &data[offset], 4);
                    offset += 4;
                }
                break;
            case 3:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->duration_ms = (uint32_t)value;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_ServoHomeCommand(const r2bus_generated_ServoHomeCommand *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->servo != 0) {
        if (!r2bus_encode_tag(1, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->servo, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->all) {
        if (!r2bus_encode_tag(2, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->all, out, &offset, max_len)) {
            return false;
        }
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_ServoHomeCommand(r2bus_generated_ServoHomeCommand *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->servo = (uint32_t)value;
                }
                break;
            case 2:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->all = (value != 0);
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_HoloColorRequest(const r2bus_generated_HoloColorRequest *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->red != 0) {
        if (!r2bus_encode_tag(1, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->red, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->green != 0) {
        if (!r2bus_encode_tag(2, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->green, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->blue != 0) {
        if (!r2bus_encode_tag(3, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->blue, out, &offset, max_len)) {
            return false;
        }
    }
    if (msg->brightness != 0) {
        if (!r2bus_encode_tag(4, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->brightness, out, &offset, max_len)) {
            return false;
        }
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_HoloColorRequest(r2bus_generated_HoloColorRequest *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->red = (uint32_t)value;
                }
                break;
            case 2:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->green = (uint32_t)value;
                }
                break;
            case 3:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->blue = (uint32_t)value;
                }
                break;
            case 4:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->brightness = (uint32_t)value;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_HoloPositionRequest(const r2bus_generated_HoloPositionRequest *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->azimuth_deg != 0) {
        if (!r2bus_encode_tag(1, 5, out, &offset, max_len)) {
            return false;
        }
        if (offset + 4 > max_len) {
            return false;
        }
        memcpy(&out[offset], &msg->azimuth_deg, 4);
        offset += 4;
    }
    if (msg->elevation_deg != 0) {
        if (!r2bus_encode_tag(2, 5, out, &offset, max_len)) {
            return false;
        }
        if (offset + 4 > max_len) {
            return false;
        }
        memcpy(&out[offset], &msg->elevation_deg, 4);
        offset += 4;
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_HoloPositionRequest(r2bus_generated_HoloPositionRequest *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 5) {
                    return false;
                }
                {
                    if (offset + 4 > len) {
                        return false;
                    }
                    memcpy(&msg->azimuth_deg, &data[offset], 4);
                    offset += 4;
                }
                break;
            case 2:
                if (wire_type != 5) {
                    return false;
                }
                {
                    if (offset + 4 > len) {
                        return false;
                    }
                    memcpy(&msg->elevation_deg, &data[offset], 4);
                    offset += 4;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

static bool r2bus_encode_PsiMoodRequest(const r2bus_generated_PsiMoodRequest *msg, uint8_t *out, size_t *out_len, size_t max_len) {
    size_t offset = 0;
    if (msg->mood != 0) {
        if (!r2bus_encode_tag(1, 0, out, &offset, max_len)) {
            return false;
        }
        if (!r2bus_encode_varint((uint64_t)msg->mood, out, &offset, max_len)) {
            return false;
        }
    }
    *out_len = offset;
    return true;
}

static bool r2bus_decode_PsiMoodRequest(r2bus_generated_PsiMoodRequest *msg, const uint8_t *data, size_t len) {
    memset(msg, 0, sizeof(*msg));
    size_t offset = 0;
    while (offset < len) {
        uint64_t tag = 0;
        if (!r2bus_decode_varint(data, len, &offset, &tag)) {
            return false;
        }
        uint32_t field_number = (uint32_t)(tag >> 3);
        uint8_t wire_type = (uint8_t)(tag & 0x07u);
        switch (field_number) {
            case 1:
                if (wire_type != 0) {
                    return false;
                }
                {
                    uint64_t value = 0;
                    if (!r2bus_decode_varint(data, len, &offset, &value)) {
                        return false;
                    }
                    msg->mood = (uint32_t)value;
                }
                break;
            default:
                if (!r2bus_skip_field(wire_type, data, len, &offset)) {
                    return false;
                }
                break;
        }
    }
    return true;
}

typedef struct {
    uint8_t dest_id;
    r2bus_msg_id_E msg_id;
    union {
        r2bus_generated_Empty Empty;
        r2bus_generated_Ping Ping;
        r2bus_generated_Pong Pong;
        r2bus_generated_Heartbeat Heartbeat;
        r2bus_generated_PsiColorRequest PsiColorRequest;
        r2bus_generated_ServoMoveCommand ServoMoveCommand;
        r2bus_generated_ServoHomeCommand ServoHomeCommand;
        r2bus_generated_HoloColorRequest HoloColorRequest;
        r2bus_generated_HoloPositionRequest HoloPositionRequest;
        r2bus_generated_PsiMoodRequest PsiMoodRequest;
    } payload;
} r2bus_tx_item_t;

static r2bus_tx_item_t g_tx_queue[R2BUS_GENERATED_TX_QUEUE_SIZE];
static size_t g_tx_head = 0;
static size_t g_tx_tail = 0;
static size_t g_tx_count = 0;

static bool r2bus_tx_enqueue(const r2bus_tx_item_t *item) {
    if (g_tx_count >= R2BUS_GENERATED_TX_QUEUE_SIZE) {
        return false;
    }
    g_tx_queue[g_tx_tail] = *item;
    g_tx_tail = (g_tx_tail + 1u) % R2BUS_GENERATED_TX_QUEUE_SIZE;
    ++g_tx_count;
    return true;
}

bool r2bus_send_Empty(uint8_t dest_id, const r2bus_generated_Empty *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_ECU_RESET;
    item.payload.Empty = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_Ping(uint8_t dest_id, const r2bus_generated_Ping *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_PING;
    item.payload.Ping = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_Pong(uint8_t dest_id, const r2bus_generated_Pong *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_PONG;
    item.payload.Pong = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_Heartbeat(uint8_t dest_id, const r2bus_generated_Heartbeat *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_HEARTBEAT;
    item.payload.Heartbeat = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_PsiColorRequest(uint8_t dest_id, const r2bus_generated_PsiColorRequest *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_PSI_COLOR_REQ;
    item.payload.PsiColorRequest = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_ServoMoveCommand(uint8_t dest_id, const r2bus_generated_ServoMoveCommand *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_SERVO_MOVE_CMD;
    item.payload.ServoMoveCommand = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_ServoHomeCommand(uint8_t dest_id, const r2bus_generated_ServoHomeCommand *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_SERVO_HOME_CMD;
    item.payload.ServoHomeCommand = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_HoloColorRequest(uint8_t dest_id, const r2bus_generated_HoloColorRequest *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_HOLO_COLOR_REQ;
    item.payload.HoloColorRequest = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_HoloPositionRequest(uint8_t dest_id, const r2bus_generated_HoloPositionRequest *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_HOLO_POSITION_REQ;
    item.payload.HoloPositionRequest = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_send_PsiMoodRequest(uint8_t dest_id, const r2bus_generated_PsiMoodRequest *payload) {
    if (!payload) {
        return false;
    }
    r2bus_tx_item_t item;
    item.dest_id = dest_id;
    item.msg_id = R2BUS_MSG_PSI_MOOD_REQ;
    item.payload.PsiMoodRequest = *payload;
    return r2bus_tx_enqueue(&item);
}

bool r2bus_tx_pop(r2bus_tx_frame_t *out) {
    if (!out || g_tx_count == 0) {
        return false;
    }
    r2bus_tx_item_t item = g_tx_queue[g_tx_head];
    g_tx_head = (g_tx_head + 1u) % R2BUS_GENERATED_TX_QUEUE_SIZE;
    --g_tx_count;
    out->dest_id = item.dest_id;
    out->msg_id = item.msg_id;
    out->payload_len = 0;
    switch (item.msg_id) {
        case R2BUS_MSG_ECU_RESET:
            if (!r2bus_encode_Empty(&item.payload.Empty, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_PING:
            if (!r2bus_encode_Ping(&item.payload.Ping, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_PONG:
            if (!r2bus_encode_Pong(&item.payload.Pong, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_HEARTBEAT:
            if (!r2bus_encode_Heartbeat(&item.payload.Heartbeat, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_PSI_COLOR_REQ:
            if (!r2bus_encode_PsiColorRequest(&item.payload.PsiColorRequest, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_SERVO_MOVE_CMD:
            if (!r2bus_encode_ServoMoveCommand(&item.payload.ServoMoveCommand, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_SERVO_HOME_CMD:
            if (!r2bus_encode_ServoHomeCommand(&item.payload.ServoHomeCommand, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_HOLO_COLOR_REQ:
            if (!r2bus_encode_HoloColorRequest(&item.payload.HoloColorRequest, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_HOLO_POSITION_REQ:
            if (!r2bus_encode_HoloPositionRequest(&item.payload.HoloPositionRequest, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        case R2BUS_MSG_PSI_MOOD_REQ:
            if (!r2bus_encode_PsiMoodRequest(&item.payload.PsiMoodRequest, out->payload, &out->payload_len, sizeof(out->payload))) {
                return false;
            }
            break;
        default:
            return false;
    }
    return true;
}

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_Empty value;
} r2bus_store_Empty_t;

static r2bus_store_Empty_t g_store_Empty[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_Empty[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_Ping value;
} r2bus_store_Ping_t;

static r2bus_store_Ping_t g_store_Ping[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_Ping[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_Pong value;
} r2bus_store_Pong_t;

static r2bus_store_Pong_t g_store_Pong[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_Pong[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_Heartbeat value;
} r2bus_store_Heartbeat_t;

static r2bus_store_Heartbeat_t g_store_Heartbeat[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_Heartbeat[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_PsiColorRequest value;
} r2bus_store_PsiColorRequest_t;

static r2bus_store_PsiColorRequest_t g_store_PsiColorRequest[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_PsiColorRequest[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_ServoMoveCommand value;
} r2bus_store_ServoMoveCommand_t;

static r2bus_store_ServoMoveCommand_t g_store_ServoMoveCommand[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_ServoMoveCommand[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_ServoHomeCommand value;
} r2bus_store_ServoHomeCommand_t;

static r2bus_store_ServoHomeCommand_t g_store_ServoHomeCommand[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_ServoHomeCommand[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_HoloColorRequest value;
} r2bus_store_HoloColorRequest_t;

static r2bus_store_HoloColorRequest_t g_store_HoloColorRequest[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_HoloColorRequest[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_HoloPositionRequest value;
} r2bus_store_HoloPositionRequest_t;

static r2bus_store_HoloPositionRequest_t g_store_HoloPositionRequest[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_HoloPositionRequest[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

typedef struct {
    bool has_value;
    uint32_t last_update_ms;
    r2bus_generated_PsiMoodRequest value;
} r2bus_store_PsiMoodRequest_t;

static r2bus_store_PsiMoodRequest_t g_store_PsiMoodRequest[R2BUS_GENERATED_NODE_COUNT];

static const uint32_t g_timeout_PsiMoodRequest[R2BUS_GENERATED_NODE_COUNT] = {
    0u,
    0u,
};

bool r2bus_handle_packet(const r2bus_packet_t *packet, uint32_t now_ms) {
    if (!packet) {
        return false;
    }
    int index = r2bus_generated_node_index(packet->src_id);
    if (index < 0) {
        return false;
    }
    switch (packet->msg_id) {
        case R2BUS_MSG_ECU_RESET:
            {
                r2bus_generated_Empty decoded;
                if (!r2bus_decode_Empty(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_Empty[(size_t)index].value = decoded;
                g_store_Empty[(size_t)index].last_update_ms = now_ms;
                g_store_Empty[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_PING:
            {
                r2bus_generated_Ping decoded;
                if (!r2bus_decode_Ping(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_Ping[(size_t)index].value = decoded;
                g_store_Ping[(size_t)index].last_update_ms = now_ms;
                g_store_Ping[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_PONG:
            {
                r2bus_generated_Pong decoded;
                if (!r2bus_decode_Pong(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_Pong[(size_t)index].value = decoded;
                g_store_Pong[(size_t)index].last_update_ms = now_ms;
                g_store_Pong[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_HEARTBEAT:
            {
                r2bus_generated_Heartbeat decoded;
                if (!r2bus_decode_Heartbeat(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_Heartbeat[(size_t)index].value = decoded;
                g_store_Heartbeat[(size_t)index].last_update_ms = now_ms;
                g_store_Heartbeat[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_PSI_COLOR_REQ:
            {
                r2bus_generated_PsiColorRequest decoded;
                if (!r2bus_decode_PsiColorRequest(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_PsiColorRequest[(size_t)index].value = decoded;
                g_store_PsiColorRequest[(size_t)index].last_update_ms = now_ms;
                g_store_PsiColorRequest[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_SERVO_MOVE_CMD:
            {
                r2bus_generated_ServoMoveCommand decoded;
                if (!r2bus_decode_ServoMoveCommand(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_ServoMoveCommand[(size_t)index].value = decoded;
                g_store_ServoMoveCommand[(size_t)index].last_update_ms = now_ms;
                g_store_ServoMoveCommand[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_SERVO_HOME_CMD:
            {
                r2bus_generated_ServoHomeCommand decoded;
                if (!r2bus_decode_ServoHomeCommand(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_ServoHomeCommand[(size_t)index].value = decoded;
                g_store_ServoHomeCommand[(size_t)index].last_update_ms = now_ms;
                g_store_ServoHomeCommand[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_HOLO_COLOR_REQ:
            {
                r2bus_generated_HoloColorRequest decoded;
                if (!r2bus_decode_HoloColorRequest(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_HoloColorRequest[(size_t)index].value = decoded;
                g_store_HoloColorRequest[(size_t)index].last_update_ms = now_ms;
                g_store_HoloColorRequest[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_HOLO_POSITION_REQ:
            {
                r2bus_generated_HoloPositionRequest decoded;
                if (!r2bus_decode_HoloPositionRequest(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_HoloPositionRequest[(size_t)index].value = decoded;
                g_store_HoloPositionRequest[(size_t)index].last_update_ms = now_ms;
                g_store_HoloPositionRequest[(size_t)index].has_value = true;
            }
            return true;
        case R2BUS_MSG_PSI_MOOD_REQ:
            {
                r2bus_generated_PsiMoodRequest decoded;
                if (!r2bus_decode_PsiMoodRequest(&decoded, packet->payload, packet->length)) {
                    return false;
                }
                g_store_PsiMoodRequest[(size_t)index].value = decoded;
                g_store_PsiMoodRequest[(size_t)index].last_update_ms = now_ms;
                g_store_PsiMoodRequest[(size_t)index].has_value = true;
            }
            return true;
        default:
            return false;
    }
}

bool r2bus_get_Empty(uint8_t src_id, r2bus_generated_Empty *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_Empty_t *entry = &g_store_Empty[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_Empty[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_Ping(uint8_t src_id, r2bus_generated_Ping *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_Ping_t *entry = &g_store_Ping[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_Ping[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_Pong(uint8_t src_id, r2bus_generated_Pong *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_Pong_t *entry = &g_store_Pong[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_Pong[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_Heartbeat(uint8_t src_id, r2bus_generated_Heartbeat *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_Heartbeat_t *entry = &g_store_Heartbeat[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_Heartbeat[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_PsiColorRequest(uint8_t src_id, r2bus_generated_PsiColorRequest *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_PsiColorRequest_t *entry = &g_store_PsiColorRequest[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_PsiColorRequest[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_ServoMoveCommand(uint8_t src_id, r2bus_generated_ServoMoveCommand *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_ServoMoveCommand_t *entry = &g_store_ServoMoveCommand[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_ServoMoveCommand[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_ServoHomeCommand(uint8_t src_id, r2bus_generated_ServoHomeCommand *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_ServoHomeCommand_t *entry = &g_store_ServoHomeCommand[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_ServoHomeCommand[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_HoloColorRequest(uint8_t src_id, r2bus_generated_HoloColorRequest *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_HoloColorRequest_t *entry = &g_store_HoloColorRequest[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_HoloColorRequest[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_HoloPositionRequest(uint8_t src_id, r2bus_generated_HoloPositionRequest *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_HoloPositionRequest_t *entry = &g_store_HoloPositionRequest[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_HoloPositionRequest[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_get_PsiMoodRequest(uint8_t src_id, r2bus_generated_PsiMoodRequest *out, uint32_t now_ms) {
    if (!out) {
        return false;
    }
    int index = r2bus_generated_node_index(src_id);
    if (index < 0) {
        return false;
    }
    const r2bus_store_PsiMoodRequest_t *entry = &g_store_PsiMoodRequest[(size_t)index];
    if (!entry->has_value) {
        return false;
    }
    *out = entry->value;
    uint32_t timeout_ms = g_timeout_PsiMoodRequest[(size_t)index];
    if (timeout_ms > 0u) {
        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);
        if (age > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool r2bus_dcfront_getEmpty(r2bus_generated_Empty *out, uint32_t now_ms) {
    return r2bus_get_Empty(0x10u, out, now_ms);
}

bool r2bus_dcfront_getPing(r2bus_generated_Ping *out, uint32_t now_ms) {
    return r2bus_get_Ping(0x10u, out, now_ms);
}

bool r2bus_dcfront_getPong(r2bus_generated_Pong *out, uint32_t now_ms) {
    return r2bus_get_Pong(0x10u, out, now_ms);
}

bool r2bus_dcfront_getHeartbeat(r2bus_generated_Heartbeat *out, uint32_t now_ms) {
    return r2bus_get_Heartbeat(0x10u, out, now_ms);
}

bool r2bus_dcfront_getPsiColorRequest(r2bus_generated_PsiColorRequest *out, uint32_t now_ms) {
    return r2bus_get_PsiColorRequest(0x10u, out, now_ms);
}

bool r2bus_dcfront_getServoMoveCommand(r2bus_generated_ServoMoveCommand *out, uint32_t now_ms) {
    return r2bus_get_ServoMoveCommand(0x10u, out, now_ms);
}

bool r2bus_dcfront_getServoHomeCommand(r2bus_generated_ServoHomeCommand *out, uint32_t now_ms) {
    return r2bus_get_ServoHomeCommand(0x10u, out, now_ms);
}

bool r2bus_dcfront_getHoloColorRequest(r2bus_generated_HoloColorRequest *out, uint32_t now_ms) {
    return r2bus_get_HoloColorRequest(0x10u, out, now_ms);
}

bool r2bus_dcfront_getHoloPositionRequest(r2bus_generated_HoloPositionRequest *out, uint32_t now_ms) {
    return r2bus_get_HoloPositionRequest(0x10u, out, now_ms);
}

bool r2bus_dcfront_getPsiMoodRequest(r2bus_generated_PsiMoodRequest *out, uint32_t now_ms) {
    return r2bus_get_PsiMoodRequest(0x10u, out, now_ms);
}

bool r2bus_dcrear_getEmpty(r2bus_generated_Empty *out, uint32_t now_ms) {
    return r2bus_get_Empty(0x11u, out, now_ms);
}

bool r2bus_dcrear_getPing(r2bus_generated_Ping *out, uint32_t now_ms) {
    return r2bus_get_Ping(0x11u, out, now_ms);
}

bool r2bus_dcrear_getPong(r2bus_generated_Pong *out, uint32_t now_ms) {
    return r2bus_get_Pong(0x11u, out, now_ms);
}

bool r2bus_dcrear_getHeartbeat(r2bus_generated_Heartbeat *out, uint32_t now_ms) {
    return r2bus_get_Heartbeat(0x11u, out, now_ms);
}

bool r2bus_dcrear_getPsiColorRequest(r2bus_generated_PsiColorRequest *out, uint32_t now_ms) {
    return r2bus_get_PsiColorRequest(0x11u, out, now_ms);
}

bool r2bus_dcrear_getServoMoveCommand(r2bus_generated_ServoMoveCommand *out, uint32_t now_ms) {
    return r2bus_get_ServoMoveCommand(0x11u, out, now_ms);
}

bool r2bus_dcrear_getServoHomeCommand(r2bus_generated_ServoHomeCommand *out, uint32_t now_ms) {
    return r2bus_get_ServoHomeCommand(0x11u, out, now_ms);
}

bool r2bus_dcrear_getHoloColorRequest(r2bus_generated_HoloColorRequest *out, uint32_t now_ms) {
    return r2bus_get_HoloColorRequest(0x11u, out, now_ms);
}

bool r2bus_dcrear_getHoloPositionRequest(r2bus_generated_HoloPositionRequest *out, uint32_t now_ms) {
    return r2bus_get_HoloPositionRequest(0x11u, out, now_ms);
}

bool r2bus_dcrear_getPsiMoodRequest(r2bus_generated_PsiMoodRequest *out, uint32_t now_ms) {
    return r2bus_get_PsiMoodRequest(0x11u, out, now_ms);
}

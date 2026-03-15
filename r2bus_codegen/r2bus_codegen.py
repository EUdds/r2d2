from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    from google.protobuf.descriptor import Descriptor, EnumDescriptor, FieldDescriptor
except ImportError:
    sys.exit("Google protobuf is required: pip install protobuf")

PROTO_DIR = Path("/home/eric/r2d2/firmware/libs/r2bus/proto").resolve()
if PROTO_DIR.exists():
    sys.path.insert(0, str(PROTO_DIR))

try:
    import r2bus_pb2
except ImportError as exc:  # pragma: no cover - import dependency hint
    sys.exit(f"Unable to import R2 bus protobuf module: {exc}")

FD = FieldDescriptor

DEFAULT_BYTES_MAX = 255


@dataclass(frozen=True)
class FieldInfo:
    name: str
    number: int
    c_type: str
    wire_type: int
    is_bytes: bool
    bytes_max: int
    is_enum: bool
    proto_type: int
    enum_values: Tuple[Tuple[str, int], ...]


@dataclass(frozen=True)
class MessageInfo:
    name: str
    msg_id: int
    msg_id_name: str
    fields: List[FieldInfo]
    rx_enabled: bool
    tx_enabled: bool


@dataclass(frozen=True)
class NodeInfo:
    name: str
    node_id: int
    c_name: str


def _camel_to_enum_name(name: str) -> str:
    snake = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name).upper()
    snake = snake.replace("REQUEST", "REQ").replace("COMMAND", "CMD")
    return f"MESSAGE_ID_{snake}"


def _build_message_map() -> Dict[int, Descriptor]:
    mapping: Dict[int, Descriptor] = {}
    for desc in r2bus_pb2.DESCRIPTOR.message_types_by_name.values():
        enum_name = (
            "MESSAGE_ID_ECU_RESET"
            if desc.name == "Empty"
            else _camel_to_enum_name(desc.name)
        )
        try:
            msg_id = r2bus_pb2.MessageId.Value(enum_name)
        except ValueError:
            continue
        mapping[msg_id] = desc
    return mapping


MESSAGE_BY_ID = _build_message_map()


def _normalize_key(value: str) -> str:
    return value.replace("_", "").replace("-", "").replace(" ", "").lower()


def _build_lookup() -> Dict[str, Descriptor]:
    lookup: Dict[str, Descriptor] = {}
    for msg_id, desc in MESSAGE_BY_ID.items():
        enum_name = r2bus_pb2.MessageId.Name(msg_id)
        aliases = {
            desc.name,
            desc.full_name,
            enum_name,
            enum_name.removeprefix("MESSAGE_ID_"),
        }
        if desc.name == "Empty":
            aliases.update({"ecu_reset", "reset", "empty"})
        for alias in aliases:
            lookup[_normalize_key(alias)] = desc
    return lookup


MESSAGE_LOOKUP = _build_lookup()


def _load_options(path: Path) -> Dict[str, int]:
    options: Dict[str, int] = {}
    if not path.exists():
        return options
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        match = re.match(r"(\S+)\s+max_(size|length):(\d+)", stripped)
        if not match:
            continue
        field_name, _, value = match.groups()
        options[field_name] = int(value)
    return options


def _load_config(path: Path) -> Dict[str, object]:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    return json.loads(path.read_text())


def _parse_nodes(config: Dict[str, object]) -> Tuple[Dict[int, Dict[str, int]], Optional[int]]:
    queue_size: Optional[int] = None
    nodes: Dict[int, Dict[str, int]] = {}
    for key, value in config.items():
        if key == "tx_queue_size":
            if not isinstance(value, int):
                raise ValueError("tx_queue_size must be an integer")
            queue_size = value
            continue
        if not isinstance(value, dict):
            raise ValueError(f"Node '{key}' must map to a message timeout object")
        node_id = int(key, 0) if isinstance(key, str) else int(key)
        if not 0 <= node_id <= 0xFF:
            raise ValueError(f"Node ID {key} is out of range 0x00-0xFF")
        timeouts: Dict[str, int] = {}
        for msg_key, timeout in value.items():
            if not isinstance(timeout, int):
                raise ValueError(
                    f"Timeout for node {key} message {msg_key} must be an integer"
                )
            desc = _resolve_message_key(msg_key)
            if desc.name == "Ack":
                continue
            timeouts[desc.name] = timeout
        nodes[node_id] = timeouts
    if not nodes:
        raise ValueError("No node entries found in config")
    return nodes, queue_size


def _parse_node_id(value: object) -> int:
    if isinstance(value, int):
        node_id = value
    elif isinstance(value, str):
        trimmed = value.strip()
        if trimmed.endswith(("u", "U")):
            trimmed = trimmed[:-1]
        node_id = int(trimmed, 0)
    else:
        raise ValueError(f"Node ID must be an int or string, got {type(value).__name__}")
    if not 0 <= node_id <= 0xFF:
        raise ValueError(f"Node ID {node_id} is out of range 0x00-0xFF")
    return node_id


def _sanitize_node_name(name: str) -> str:
    sanitized = re.sub(r"[^0-9A-Za-z]+", "_", name.strip()).lower()
    sanitized = sanitized.strip("_")
    if not sanitized:
        raise ValueError(f"Node name '{name}' does not contain valid characters")
    if sanitized[0].isdigit():
        sanitized = f"node_{sanitized}"
    return sanitized


def _c_escape_string(value: str) -> str:
    escaped: List[str] = []
    for ch in value:
        if ch == "\\":
            escaped.append("\\\\")
        elif ch == "\"":
            escaped.append("\\\"")
        elif ch == "\n":
            escaped.append("\\n")
        elif ch == "\r":
            escaped.append("\\r")
        elif ch == "\t":
            escaped.append("\\t")
        else:
            code = ord(ch)
            if 32 <= code <= 126:
                escaped.append(ch)
            else:
                escaped.append(f"\\x{code:02x}")
    return "".join(escaped)


def _load_nodes(path: Path) -> List[NodeInfo]:
    if not path.exists():
        raise FileNotFoundError(f"Nodes file not found: {path}")
    data = json.loads(path.read_text())
    raw_nodes: List[Tuple[str, object]] = []
    if isinstance(data, dict) and "nodes" in data:
        entries = data["nodes"]
        if not isinstance(entries, list):
            raise ValueError("nodes.json 'nodes' must be a list")
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError("nodes.json entries must be objects")
            name = entry.get("name")
            node_id = entry.get("id")
            if name is None or node_id is None:
                raise ValueError("nodes.json entries must include name and id")
            raw_nodes.append((name, node_id))
    elif isinstance(data, dict):
        raw_nodes = list(data.items())
    else:
        raise ValueError("nodes.json must be an object or {\"nodes\": [...]} list")

    if not raw_nodes:
        raise ValueError("No nodes found in nodes.json")

    nodes: List[NodeInfo] = []
    seen_names = set()
    seen_c_names = set()
    seen_ids = set()
    for name, node_id_value in raw_nodes:
        if not isinstance(name, str):
            raise ValueError("Node name must be a string")
        node_id = _parse_node_id(node_id_value)
        c_name = _sanitize_node_name(name)
        if name in seen_names:
            raise ValueError(f"Duplicate node name '{name}' in nodes.json")
        if c_name in seen_c_names:
            raise ValueError(
                f"Duplicate sanitized node name '{c_name}' in nodes.json"
            )
        if node_id in seen_ids:
            raise ValueError(f"Duplicate node ID 0x{node_id:02X} in nodes.json")
        seen_names.add(name)
        seen_c_names.add(c_name)
        seen_ids.add(node_id)
        nodes.append(NodeInfo(name=name, node_id=node_id, c_name=c_name))
    return nodes


def _enum_values(enum_desc: EnumDescriptor) -> List[Tuple[str, int]]:
    return [(value.name, value.number) for value in enum_desc.values]


def _proto_type_to_wire(field: FieldDescriptor) -> int:
    if field.type in {
        FD.TYPE_INT32,
        FD.TYPE_INT64,
        FD.TYPE_UINT32,
        FD.TYPE_UINT64,
        FD.TYPE_SINT32,
        FD.TYPE_SINT64,
        FD.TYPE_BOOL,
        FD.TYPE_ENUM,
    }:
        return 0
    if field.type in {FD.TYPE_FIXED64, FD.TYPE_SFIXED64, FD.TYPE_DOUBLE}:
        return 1
    if field.type in {FD.TYPE_BYTES, FD.TYPE_STRING}:
        return 2
    if field.type in {FD.TYPE_FIXED32, FD.TYPE_SFIXED32, FD.TYPE_FLOAT}:
        return 5
    raise ValueError(f"Unsupported field type {field.type} for {field.name}")


def _proto_type_to_c(field: FieldDescriptor, enum_name: str) -> str:
    if field.type == FD.TYPE_BOOL:
        return "bool"
    if field.type in {FD.TYPE_INT32, FD.TYPE_SINT32, FD.TYPE_SFIXED32}:
        return "int32_t"
    if field.type in {FD.TYPE_UINT32, FD.TYPE_FIXED32}:
        return "uint32_t"
    if field.type in {FD.TYPE_INT64, FD.TYPE_SINT64, FD.TYPE_SFIXED64}:
        return "int64_t"
    if field.type in {FD.TYPE_UINT64, FD.TYPE_FIXED64}:
        return "uint64_t"
    if field.type == FD.TYPE_FLOAT:
        return "float"
    if field.type == FD.TYPE_DOUBLE:
        return "double"
    if field.type == FD.TYPE_ENUM:
        return enum_name
    if field.type in {FD.TYPE_BYTES, FD.TYPE_STRING}:
        return "uint8_t"
    raise ValueError(f"Unsupported field type {field.type} for {field.name}")


def _ros_enum_type(enum_values: Tuple[Tuple[str, int], ...]) -> str:
    if not enum_values:
        return "uint32"
    values = [value for _, value in enum_values]
    min_value = min(values)
    max_value = max(values)
    if min_value < 0:
        if min_value >= -2147483648 and max_value <= 2147483647:
            return "int32"
        return "int64"
    if max_value <= 0xFF:
        return "uint8"
    if max_value <= 0xFFFF:
        return "uint16"
    if max_value <= 0xFFFFFFFF:
        return "uint32"
    return "uint64"


def _proto_type_to_ros(field: FieldInfo) -> str:
    if field.is_enum:
        return _ros_enum_type(field.enum_values)
    if field.proto_type == FD.TYPE_BOOL:
        return "bool"
    if field.proto_type in {FD.TYPE_INT32, FD.TYPE_SINT32, FD.TYPE_SFIXED32}:
        return "int32"
    if field.proto_type in {FD.TYPE_UINT32, FD.TYPE_FIXED32}:
        return "uint32"
    if field.proto_type in {FD.TYPE_INT64, FD.TYPE_SINT64, FD.TYPE_SFIXED64}:
        return "int64"
    if field.proto_type in {FD.TYPE_UINT64, FD.TYPE_FIXED64}:
        return "uint64"
    if field.proto_type == FD.TYPE_FLOAT:
        return "float32"
    if field.proto_type == FD.TYPE_DOUBLE:
        return "float64"
    if field.proto_type == FD.TYPE_STRING:
        return "string"
    if field.proto_type == FD.TYPE_BYTES:
        return "uint8[]"
    raise ValueError(f"Unsupported field type {field.proto_type} for {field.name}")


def _build_messages(options: Dict[str, int]) -> List[MessageInfo]:
    messages: List[MessageInfo] = []
    for msg_id in sorted(MESSAGE_BY_ID):
        desc = MESSAGE_BY_ID[msg_id]
        fields: List[FieldInfo] = []
        for field in desc.fields:
            full_name = f"{desc.full_name}.{field.name}"
            bytes_max = options.get(full_name, DEFAULT_BYTES_MAX)
            enum_values: Tuple[Tuple[str, int], ...] = ()
            enum_name = ""
            if field.type == FD.TYPE_ENUM:
                enum_name = f"r2bus_generated_{field.enum_type.name}_E"
                enum_values = tuple(_enum_values(field.enum_type))
            c_type = _proto_type_to_c(field, enum_name)
            fields.append(
                FieldInfo(
                    name=field.name,
                    number=field.number,
                    c_type=c_type,
                    wire_type=_proto_type_to_wire(field),
                    is_bytes=field.type in {FD.TYPE_BYTES, FD.TYPE_STRING},
                    bytes_max=bytes_max,
                    is_enum=field.type == FD.TYPE_ENUM,
                    proto_type=field.type,
                    enum_values=enum_values,
                )
            )
        messages.append(
            MessageInfo(
                name=desc.name,
                msg_id=msg_id,
                msg_id_name=r2bus_pb2.MessageId.Name(msg_id),
                fields=fields,
                rx_enabled=desc.name != "Ack",
                tx_enabled=desc.name != "Ack",
            )
        )
    return messages


def _resolve_message_key(key: str) -> Descriptor:
    normalized = _normalize_key(key)
    desc = MESSAGE_LOOKUP.get(normalized)
    if not desc:
        raise ValueError(f"Unknown message key '{key}' in config")
    return desc


def _format_enum_entries(entries: List[Tuple[str, int]], prefix: str) -> List[str]:
    lines = []
    for name, value in entries:
        lines.append(f"    {prefix}{name} = {value},")
    return lines


def _render_header(
    messages: List[MessageInfo],
    enums: List[EnumDescriptor],
    nodes: List[NodeInfo],
    tx_queue_size: int,
) -> str:
    lines: List[str] = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <stdbool.h>")
    lines.append("#include <stddef.h>")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("#if __has_include(\"r2bus.h\")")
    lines.append("#include \"r2bus.h\"")
    lines.append("#define R2BUS_GENERATED_HAS_R2BUS 1")
    lines.append("#else")
    lines.append("#define R2BUS_GENERATED_HAS_R2BUS 0")
    lines.append("#endif")
    lines.append("")
    lines.append(f"#define R2BUS_GENERATED_TX_QUEUE_SIZE {tx_queue_size}")
    lines.append(f"#define R2BUS_GENERATED_MAX_PAYLOAD {DEFAULT_BYTES_MAX}u")
    lines.append(f"#define R2BUS_GENERATED_NODE_COUNT {len(nodes)}u")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    const char *name;")
    lines.append("    uint8_t node_id;")
    lines.append("} r2bus_generated_NodeInfo;")
    lines.append("")
    lines.append("extern const r2bus_generated_NodeInfo r2bus_generated_nodes[R2BUS_GENERATED_NODE_COUNT];")
    lines.append("")

    for enum_desc in enums:
        if enum_desc.name in {"Status", "MessageId"}:
            continue
        enum_name = f"r2bus_generated_{enum_desc.name}_E"
        enum_entries = _enum_values(enum_desc)
        lines.append("typedef enum {")
        lines.extend(_format_enum_entries(enum_entries, "R2BUS_"))
        lines.append(f"}} {enum_name};")
        lines.append("")

    lines.append("#if R2BUS_GENERATED_HAS_R2BUS")
    lines.append("typedef r2bus_status_E r2bus_generated_Status_E;")
    lines.append("typedef r2bus_msg_id_E r2bus_generated_MessageId_E;")
    lines.append("#endif")
    lines.append("")

    lines.append("#if !R2BUS_GENERATED_HAS_R2BUS")
    lines.append("typedef enum {")
    for msg in messages:
        lines.append(f"    R2BUS_MSG_{msg.msg_id_name.removeprefix('MESSAGE_ID_')} = {msg.msg_id},")
    lines.append("} r2bus_msg_id_E;")
    lines.append("")
    lines.append("typedef enum {")
    status_enum = r2bus_pb2.Status.DESCRIPTOR
    for name, value in _enum_values(status_enum):
        lines.append(f"    R2BUS_{name} = {value},")
    lines.append("} r2bus_status_E;")
    lines.append("")
    lines.append("typedef r2bus_status_E r2bus_generated_Status_E;")
    lines.append("typedef r2bus_msg_id_E r2bus_generated_MessageId_E;")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    uint8_t dest_id;")
    lines.append("    uint8_t src_id;")
    lines.append("    r2bus_msg_id_E msg_id;")
    lines.append("    uint8_t length;")
    lines.append("    uint8_t payload[R2BUS_GENERATED_MAX_PAYLOAD];")
    lines.append("} r2bus_packet_t;")
    lines.append("#endif")
    lines.append("")

    for msg in messages:
        struct_name = f"r2bus_generated_{msg.name}"
        if not msg.fields:
            lines.append(f"typedef struct {{")
            lines.append("    uint8_t _unused;")
            lines.append(f"}} {struct_name};")
            lines.append("")
            continue
        lines.append(f"typedef struct {{")
        for field in msg.fields:
            if field.is_bytes:
                lines.append(f"    size_t {field.name}_size;")
                lines.append(f"    uint8_t {field.name}_data[{field.bytes_max}];")
            else:
                lines.append(f"    {field.c_type} {field.name};")
        lines.append(f"}} {struct_name};")
        lines.append("")

    lines.append("typedef struct {")
    lines.append("    uint8_t dest_id;")
    lines.append("    r2bus_msg_id_E msg_id;")
    lines.append("    uint8_t payload[R2BUS_GENERATED_MAX_PAYLOAD];")
    lines.append("    size_t payload_len;")
    lines.append("} r2bus_tx_frame_t;")
    lines.append("")

    for msg in messages:
        struct_name = f"r2bus_generated_{msg.name}"
        if msg.tx_enabled:
            lines.append(
                f"bool r2bus_send_{msg.name}(uint8_t dest_id, const {struct_name} *payload);"
            )
    lines.append("")
    for msg in messages:
        struct_name = f"r2bus_generated_{msg.name}"
        if msg.rx_enabled:
            lines.append(
                f"bool r2bus_get_{msg.name}(uint8_t src_id, {struct_name} *out, uint32_t now_ms);"
            )
    lines.append("")
    for node in nodes:
        for msg in messages:
            if not msg.rx_enabled:
                continue
            struct_name = f"r2bus_generated_{msg.name}"
            lines.append(
                f"bool r2bus_{node.c_name}_get{msg.name}({struct_name} *out, uint32_t now_ms);"
            )
    lines.append("")
    lines.append("bool r2bus_handle_packet(const r2bus_packet_t *packet, uint32_t now_ms);")
    lines.append("bool r2bus_tx_pop(r2bus_tx_frame_t *out);")
    lines.append("")

    return "\n".join(lines)


def _render_source(
    messages: List[MessageInfo],
    nodes: List[NodeInfo],
    node_timeouts: Dict[int, Dict[str, int]],
) -> str:
    lines: List[str] = []
    lines.append('#include "r2bus_generated.h"')
    lines.append("")
    lines.append("#include <string.h>")
    lines.append("")
    lines.append("#define R2BUS_WIRE_VARINT 0")
    lines.append("#define R2BUS_WIRE_64BIT 1")
    lines.append("#define R2BUS_WIRE_LEN 2")
    lines.append("#define R2BUS_WIRE_32BIT 5")
    lines.append("")
    lines.append("static const uint8_t r2bus_generated_node_ids[R2BUS_GENERATED_NODE_COUNT] = {")
    for node_id in nodes:
        lines.append(f"    0x{node_id.node_id:02X},")
    lines.append("};")
    lines.append("")
    lines.append("const r2bus_generated_NodeInfo r2bus_generated_nodes[R2BUS_GENERATED_NODE_COUNT] = {")
    for node_id in nodes:
        name = _c_escape_string(node_id.name)
        lines.append(f"    {{\"{name}\", 0x{node_id.node_id:02X}u}},")
    lines.append("};")
    lines.append("")

    lines.append("static int r2bus_generated_node_index(uint8_t node_id) {")
    lines.append("    for (size_t i = 0; i < R2BUS_GENERATED_NODE_COUNT; ++i) {")
    lines.append("        if (r2bus_generated_node_ids[i] == node_id) {")
    lines.append("            return (int)i;")
    lines.append("        }")
    lines.append("    }")
    lines.append("    return -1;")
    lines.append("}")
    lines.append("")

    lines.append("static bool r2bus_encode_varint(uint64_t value, uint8_t *out, size_t *offset, size_t max_len) {")
    lines.append("    while (value >= 0x80) {")
    lines.append("        if (*offset >= max_len) {")
    lines.append("            return false;")
    lines.append("        }")
    lines.append("        out[(*offset)++] = (uint8_t)((value & 0x7Fu) | 0x80u);")
    lines.append("        value >>= 7;")
    lines.append("    }")
    lines.append("    if (*offset >= max_len) {")
    lines.append("        return false;")
    lines.append("    }")
    lines.append("    out[(*offset)++] = (uint8_t)value;")
    lines.append("    return true;")
    lines.append("}")
    lines.append("")

    lines.append("static bool r2bus_decode_varint(const uint8_t *data, size_t len, size_t *offset, uint64_t *value) {")
    lines.append("    uint64_t result = 0;")
    lines.append("    uint32_t shift = 0;")
    lines.append("    while (*offset < len && shift <= 63) {")
    lines.append("        uint8_t byte = data[(*offset)++];")
    lines.append("        result |= (uint64_t)(byte & 0x7Fu) << shift;")
    lines.append("        if (!(byte & 0x80u)) {")
    lines.append("            *value = result;")
    lines.append("            return true;")
    lines.append("        }")
    lines.append("        shift += 7;")
    lines.append("    }")
    lines.append("    return false;")
    lines.append("}")
    lines.append("")

    lines.append("static uint32_t r2bus_encode_zigzag32(int32_t value) {")
    lines.append("    return (uint32_t)((value << 1) ^ (value >> 31));")
    lines.append("}")
    lines.append("")
    lines.append("static uint64_t r2bus_encode_zigzag64(int64_t value) {")
    lines.append("    return (uint64_t)((value << 1) ^ (value >> 63));")
    lines.append("}")
    lines.append("")
    lines.append("static int32_t r2bus_decode_zigzag32(uint32_t value) {")
    lines.append("    return (int32_t)((value >> 1) ^ (uint32_t)(-(int32_t)(value & 1u)));")
    lines.append("}")
    lines.append("")
    lines.append("static int64_t r2bus_decode_zigzag64(uint64_t value) {")
    lines.append("    return (int64_t)((value >> 1) ^ (uint64_t)(-(int64_t)(value & 1u)));")
    lines.append("}")
    lines.append("")

    lines.append("static bool r2bus_encode_tag(uint32_t field_number, uint8_t wire_type, uint8_t *out, size_t *offset, size_t max_len) {")
    lines.append("    uint64_t tag = ((uint64_t)field_number << 3) | (uint64_t)wire_type;")
    lines.append("    return r2bus_encode_varint(tag, out, offset, max_len);")
    lines.append("}")
    lines.append("")

    lines.append("static bool r2bus_skip_field(uint8_t wire_type, const uint8_t *data, size_t len, size_t *offset) {")
    lines.append("    uint64_t value = 0;")
    lines.append("    switch (wire_type) {")
    lines.append("        case R2BUS_WIRE_VARINT:")
    lines.append("            return r2bus_decode_varint(data, len, offset, &value);")
    lines.append("        case R2BUS_WIRE_64BIT:")
    lines.append("            if (*offset + 8 > len) {")
    lines.append("                return false;")
    lines.append("            }")
    lines.append("            *offset += 8;")
    lines.append("            return true;")
    lines.append("        case R2BUS_WIRE_LEN:")
    lines.append("            if (!r2bus_decode_varint(data, len, offset, &value)) {")
    lines.append("                return false;")
    lines.append("            }")
    lines.append("            if (*offset + (size_t)value > len) {")
    lines.append("                return false;")
    lines.append("            }")
    lines.append("            *offset += (size_t)value;")
    lines.append("            return true;")
    lines.append("        case R2BUS_WIRE_32BIT:")
    lines.append("            if (*offset + 4 > len) {")
    lines.append("                return false;")
    lines.append("            }")
    lines.append("            *offset += 4;")
    lines.append("            return true;")
    lines.append("        default:")
    lines.append("            return false;")
    lines.append("    }")
    lines.append("}")
    lines.append("")

    for msg in messages:
        struct_name = f"r2bus_generated_{msg.name}"
        encode_name = f"r2bus_encode_{msg.name}"
        decode_name = f"r2bus_decode_{msg.name}"
        if msg.tx_enabled:
            lines.append(f"static bool {encode_name}(const {struct_name} *msg, uint8_t *out, size_t *out_len, size_t max_len) {{")
            lines.append("    size_t offset = 0;")
            for field in msg.fields:
                field_ref = f"msg->{field.name}"
                if field.is_bytes:
                    lines.append(f"    if (msg->{field.name}_size > {field.bytes_max}u) {{")
                    lines.append("        return false;")
                    lines.append("    }")
                    lines.append(f"    if (msg->{field.name}_size > 0) {{")
                    lines.append(f"        if (!r2bus_encode_tag({field.number}, R2BUS_WIRE_LEN, out, &offset, max_len)) {{")
                    lines.append("            return false;")
                    lines.append("        }")
                    lines.append(f"        if (!r2bus_encode_varint(msg->{field.name}_size, out, &offset, max_len)) {{")
                    lines.append("            return false;")
                    lines.append("        }")
                    lines.append(f"        if (offset + msg->{field.name}_size > max_len) {{")
                    lines.append("            return false;")
                    lines.append("        }")
                    lines.append(f"        memcpy(&out[offset], msg->{field.name}_data, msg->{field.name}_size);")
                    lines.append(f"        offset += msg->{field.name}_size;")
                    lines.append("    }")
                    continue

                if field.proto_type == FD.TYPE_BOOL:
                    lines.append(f"    if ({field_ref}) {{")
                else:
                    lines.append(f"    if ({field_ref} != 0) {{")
                lines.append(f"        if (!r2bus_encode_tag({field.number}, {field.wire_type}, out, &offset, max_len)) {{")
                lines.append("            return false;")
                lines.append("        }")

                if field.proto_type in {
                    FD.TYPE_INT32,
                    FD.TYPE_INT64,
                    FD.TYPE_UINT32,
                    FD.TYPE_UINT64,
                    FD.TYPE_BOOL,
                    FD.TYPE_ENUM,
                }:
                    lines.append(f"        if (!r2bus_encode_varint((uint64_t){field_ref}, out, &offset, max_len)) {{")
                    lines.append("            return false;")
                    lines.append("        }")
                elif field.proto_type == FD.TYPE_SINT32:
                    lines.append(f"        if (!r2bus_encode_varint(r2bus_encode_zigzag32({field_ref}), out, &offset, max_len)) {{")
                    lines.append("            return false;")
                    lines.append("        }")
                elif field.proto_type == FD.TYPE_SINT64:
                    lines.append(f"        if (!r2bus_encode_varint(r2bus_encode_zigzag64({field_ref}), out, &offset, max_len)) {{")
                    lines.append("            return false;")
                    lines.append("        }")
                elif field.proto_type in {FD.TYPE_FIXED32, FD.TYPE_SFIXED32, FD.TYPE_FLOAT}:
                    lines.append("        if (offset + 4 > max_len) {")
                    lines.append("            return false;")
                    lines.append("        }")
                    if field.proto_type == FD.TYPE_FLOAT:
                        lines.append(f"        memcpy(&out[offset], &{field_ref}, 4);")
                    else:
                        lines.append(f"        uint32_t value = (uint32_t){field_ref};")
                        lines.append("        memcpy(&out[offset], &value, 4);")
                    lines.append("        offset += 4;")
                elif field.proto_type in {FD.TYPE_FIXED64, FD.TYPE_SFIXED64, FD.TYPE_DOUBLE}:
                    lines.append("        if (offset + 8 > max_len) {")
                    lines.append("            return false;")
                    lines.append("        }")
                    if field.proto_type == FD.TYPE_DOUBLE:
                        lines.append(f"        memcpy(&out[offset], &{field_ref}, 8);")
                    else:
                        lines.append(f"        uint64_t value = (uint64_t){field_ref};")
                        lines.append("        memcpy(&out[offset], &value, 8);")
                    lines.append("        offset += 8;")
                lines.append("    }")
            lines.append("    *out_len = offset;")
            lines.append("    return true;")
            lines.append("}")
            lines.append("")

        if msg.rx_enabled:
            lines.append(f"static bool {decode_name}({struct_name} *msg, const uint8_t *data, size_t len) {{")
            lines.append("    memset(msg, 0, sizeof(*msg));")
            lines.append("    size_t offset = 0;")
            lines.append("    while (offset < len) {")
            lines.append("        uint64_t tag = 0;")
            lines.append("        if (!r2bus_decode_varint(data, len, &offset, &tag)) {")
            lines.append("            return false;")
            lines.append("        }")
            lines.append("        uint32_t field_number = (uint32_t)(tag >> 3);")
            lines.append("        uint8_t wire_type = (uint8_t)(tag & 0x07u);")
            lines.append("        switch (field_number) {")
            for field in msg.fields:
                lines.append(f"            case {field.number}:")
                lines.append(f"                if (wire_type != {field.wire_type}) {{")
                lines.append("                    return false;")
                lines.append("                }")
                if field.is_bytes:
                    lines.append("                {")
                    lines.append("                    uint64_t length = 0;")
                    lines.append("                    if (!r2bus_decode_varint(data, len, &offset, &length)) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    lines.append(f"                    if (length > {field.bytes_max}u) {{")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    lines.append("                    if (offset + (size_t)length > len) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    lines.append(f"                    msg->{field.name}_size = (size_t)length;")
                    lines.append(f"                    if (length > 0) {{")
                    lines.append(f"                        memcpy(msg->{field.name}_data, &data[offset], (size_t)length);")
                    lines.append("                    }")
                    lines.append("                    offset += (size_t)length;")
                    lines.append("                }")
                elif field.proto_type in {
                    FD.TYPE_INT32,
                    FD.TYPE_INT64,
                    FD.TYPE_UINT32,
                    FD.TYPE_UINT64,
                    FD.TYPE_BOOL,
                    FD.TYPE_ENUM,
                }:
                    lines.append("                {")
                    lines.append("                    uint64_t value = 0;")
                    lines.append("                    if (!r2bus_decode_varint(data, len, &offset, &value)) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    if field.proto_type == FD.TYPE_BOOL:
                        lines.append(f"                    msg->{field.name} = (value != 0);")
                    else:
                        lines.append(f"                    msg->{field.name} = ({field.c_type})value;")
                    lines.append("                }")
                elif field.proto_type == FD.TYPE_SINT32:
                    lines.append("                {")
                    lines.append("                    uint64_t value = 0;")
                    lines.append("                    if (!r2bus_decode_varint(data, len, &offset, &value)) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    lines.append(f"                    msg->{field.name} = r2bus_decode_zigzag32((uint32_t)value);")
                    lines.append("                }")
                elif field.proto_type == FD.TYPE_SINT64:
                    lines.append("                {")
                    lines.append("                    uint64_t value = 0;")
                    lines.append("                    if (!r2bus_decode_varint(data, len, &offset, &value)) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    lines.append(f"                    msg->{field.name} = r2bus_decode_zigzag64(value);")
                    lines.append("                }")
                elif field.proto_type in {FD.TYPE_FIXED32, FD.TYPE_SFIXED32, FD.TYPE_FLOAT}:
                    lines.append("                {")
                    lines.append("                    if (offset + 4 > len) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    if field.proto_type == FD.TYPE_FLOAT:
                        lines.append(f"                    memcpy(&msg->{field.name}, &data[offset], 4);")
                    else:
                        lines.append("                    uint32_t value = 0;")
                        lines.append("                    memcpy(&value, &data[offset], 4);")
                        lines.append(f"                    msg->{field.name} = ({field.c_type})value;")
                    lines.append("                    offset += 4;")
                    lines.append("                }")
                elif field.proto_type in {FD.TYPE_FIXED64, FD.TYPE_SFIXED64, FD.TYPE_DOUBLE}:
                    lines.append("                {")
                    lines.append("                    if (offset + 8 > len) {")
                    lines.append("                        return false;")
                    lines.append("                    }")
                    if field.proto_type == FD.TYPE_DOUBLE:
                        lines.append(f"                    memcpy(&msg->{field.name}, &data[offset], 8);")
                    else:
                        lines.append("                    uint64_t value = 0;")
                        lines.append("                    memcpy(&value, &data[offset], 8);")
                        lines.append(f"                    msg->{field.name} = ({field.c_type})value;")
                    lines.append("                    offset += 8;")
                    lines.append("                }")
                lines.append("                break;")
            lines.append("            default:")
            lines.append("                if (!r2bus_skip_field(wire_type, data, len, &offset)) {")
            lines.append("                    return false;")
            lines.append("                }")
            lines.append("                break;")
            lines.append("        }")
            lines.append("    }")
            lines.append("    return true;")
            lines.append("}")
            lines.append("")

    lines.append("typedef struct {")
    lines.append("    uint8_t dest_id;")
    lines.append("    r2bus_msg_id_E msg_id;")
    lines.append("    union {")
    for msg in messages:
        if msg.tx_enabled:
            lines.append(f"        r2bus_generated_{msg.name} {msg.name};")
    lines.append("    } payload;")
    lines.append("} r2bus_tx_item_t;")
    lines.append("")

    lines.append("static r2bus_tx_item_t g_tx_queue[R2BUS_GENERATED_TX_QUEUE_SIZE];")
    lines.append("static size_t g_tx_head = 0;")
    lines.append("static size_t g_tx_tail = 0;")
    lines.append("static size_t g_tx_count = 0;")
    lines.append("")

    lines.append("static bool r2bus_tx_enqueue(const r2bus_tx_item_t *item) {")
    lines.append("    if (g_tx_count >= R2BUS_GENERATED_TX_QUEUE_SIZE) {")
    lines.append("        return false;")
    lines.append("    }")
    lines.append("    g_tx_queue[g_tx_tail] = *item;")
    lines.append("    g_tx_tail = (g_tx_tail + 1u) % R2BUS_GENERATED_TX_QUEUE_SIZE;")
    lines.append("    ++g_tx_count;")
    lines.append("    return true;")
    lines.append("}")
    lines.append("")

    for msg in messages:
        if not msg.tx_enabled:
            continue
        struct_name = f"r2bus_generated_{msg.name}"
        lines.append(
            f"bool r2bus_send_{msg.name}(uint8_t dest_id, const {struct_name} *payload) {{"
        )
        lines.append("    if (!payload) {")
        lines.append("        return false;")
        lines.append("    }")
        lines.append("    r2bus_tx_item_t item;")
        lines.append("    item.dest_id = dest_id;")
        lines.append(f"    item.msg_id = R2BUS_MSG_{msg.msg_id_name.removeprefix('MESSAGE_ID_')};")
        lines.append(f"    item.payload.{msg.name} = *payload;")
        lines.append("    return r2bus_tx_enqueue(&item);")
        lines.append("}")
        lines.append("")

    lines.append("bool r2bus_tx_pop(r2bus_tx_frame_t *out) {")
    lines.append("    if (!out || g_tx_count == 0) {")
    lines.append("        return false;")
    lines.append("    }")
    lines.append("    r2bus_tx_item_t item = g_tx_queue[g_tx_head];")
    lines.append("    g_tx_head = (g_tx_head + 1u) % R2BUS_GENERATED_TX_QUEUE_SIZE;")
    lines.append("    --g_tx_count;")
    lines.append("    out->dest_id = item.dest_id;")
    lines.append("    out->msg_id = item.msg_id;")
    lines.append("    out->payload_len = 0;")
    lines.append("    switch (item.msg_id) {")
    for msg in messages:
        if not msg.tx_enabled:
            continue
        encode_name = f"r2bus_encode_{msg.name}"
        lines.append(f"        case R2BUS_MSG_{msg.msg_id_name.removeprefix('MESSAGE_ID_')}:")
        lines.append(f"            if (!{encode_name}(&item.payload.{msg.name}, out->payload, &out->payload_len, sizeof(out->payload))) {{")
        lines.append("                return false;")
        lines.append("            }")
        lines.append("            break;")
    lines.append("        default:")
    lines.append("            return false;")
    lines.append("    }")
    lines.append("    return true;")
    lines.append("}")
    lines.append("")

    for msg in messages:
        if not msg.rx_enabled:
            continue
        struct_name = f"r2bus_generated_{msg.name}"
        lines.append(f"typedef struct {{")
        lines.append("    bool has_value;")
        lines.append("    uint32_t last_update_ms;")
        lines.append(f"    {struct_name} value;")
        lines.append(f"}} r2bus_store_{msg.name}_t;")
        lines.append("")
        lines.append(f"static r2bus_store_{msg.name}_t g_store_{msg.name}[R2BUS_GENERATED_NODE_COUNT];")
        lines.append("")
        lines.append(f"static const uint32_t g_timeout_{msg.name}[R2BUS_GENERATED_NODE_COUNT] = {{")
        for node_id in nodes:
            timeout = node_timeouts.get(node_id.node_id, {}).get(msg.name, 0)
            lines.append(f"    {timeout}u,")
        lines.append("};")
        lines.append("")

    lines.append("bool r2bus_handle_packet(const r2bus_packet_t *packet, uint32_t now_ms) {")
    lines.append("    if (!packet) {")
    lines.append("        return false;")
    lines.append("    }")
    lines.append("    int index = r2bus_generated_node_index(packet->src_id);")
    lines.append("    if (index < 0) {")
    lines.append("        return false;")
    lines.append("    }")
    lines.append("    switch (packet->msg_id) {")
    for msg in messages:
        if not msg.rx_enabled:
            continue
        struct_name = f"r2bus_generated_{msg.name}"
        decode_name = f"r2bus_decode_{msg.name}"
        lines.append(f"        case R2BUS_MSG_{msg.msg_id_name.removeprefix('MESSAGE_ID_')}:")
        lines.append("            {")
        lines.append(f"                {struct_name} decoded;")
        lines.append(f"                if (!{decode_name}(&decoded, packet->payload, packet->length)) {{")
        lines.append("                    return false;")
        lines.append("                }")
        lines.append(f"                g_store_{msg.name}[(size_t)index].value = decoded;")
        lines.append(f"                g_store_{msg.name}[(size_t)index].last_update_ms = now_ms;")
        lines.append(f"                g_store_{msg.name}[(size_t)index].has_value = true;")
        lines.append("            }")
        lines.append("            return true;")
    lines.append("        default:")
    lines.append("            return false;")
    lines.append("    }")
    lines.append("}")
    lines.append("")

    for msg in messages:
        if not msg.rx_enabled:
            continue
        struct_name = f"r2bus_generated_{msg.name}"
        lines.append(
            f"bool r2bus_get_{msg.name}(uint8_t src_id, {struct_name} *out, uint32_t now_ms) {{"
        )
        lines.append("    if (!out) {")
        lines.append("        return false;")
        lines.append("    }")
        lines.append("    int index = r2bus_generated_node_index(src_id);")
        lines.append("    if (index < 0) {")
        lines.append("        return false;")
        lines.append("    }")
        lines.append(f"    const r2bus_store_{msg.name}_t *entry = &g_store_{msg.name}[(size_t)index];")
        lines.append("    if (!entry->has_value) {")
        lines.append("        return false;")
        lines.append("    }")
        lines.append("    *out = entry->value;")
        lines.append(f"    uint32_t timeout_ms = g_timeout_{msg.name}[(size_t)index];")
        lines.append("    if (timeout_ms > 0u) {")
        lines.append("        uint32_t age = (uint32_t)(now_ms - entry->last_update_ms);")
        lines.append("        if (age > timeout_ms) {")
        lines.append("            return false;")
        lines.append("        }")
        lines.append("    }")
        lines.append("    return true;")
        lines.append("}")
        lines.append("")

    for node in nodes:
        for msg in messages:
            if not msg.rx_enabled:
                continue
            struct_name = f"r2bus_generated_{msg.name}"
            lines.append(
                f"bool r2bus_{node.c_name}_get{msg.name}({struct_name} *out, uint32_t now_ms) {{"
            )
            lines.append(
                f"    return r2bus_get_{msg.name}(0x{node.node_id:02X}u, out, now_ms);"
            )
            lines.append("}")
            lines.append("")

    return "\n".join(lines)


def _render_ros_message(msg: MessageInfo) -> str:
    lines: List[str] = []
    lines.append("# Auto-generated by r2bus_codegen.py. Do not edit.")

    constants: List[Tuple[str, str, int]] = []
    seen_constants: Dict[str, Tuple[str, int]] = {}
    for field in msg.fields:
        if not field.is_enum:
            continue
        enum_type = _ros_enum_type(field.enum_values)
        for name, value in field.enum_values:
            existing = seen_constants.get(name)
            if existing:
                if existing != (enum_type, value):
                    raise ValueError(
                        f"Enum constant conflict for {msg.name}: {name} "
                        f"{existing} vs {(enum_type, value)}"
                    )
                continue
            seen_constants[name] = (enum_type, value)
            constants.append((enum_type, name, value))

    for enum_type, name, value in constants:
        lines.append(f"{enum_type} {name}={value}")

    for field in msg.fields:
        ros_type = _proto_type_to_ros(field)
        lines.append(f"{ros_type} {field.name}")

    return "\n".join(lines).rstrip() + "\n"


def _render_ros_service(msg: MessageInfo) -> str:
    lines = [
        "# Auto-generated by r2bus_codegen.py. Do not edit.",
        "uint8 dest_id",
        f"{msg.name} payload",
        "---",
        "Ack ack",
    ]
    return "\n".join(lines).rstrip() + "\n"


def _write_ros_interfaces(messages: List[MessageInfo], out_dir: Path) -> None:
    msg_dir = out_dir / "msg"
    srv_dir = out_dir / "srv"
    msg_dir.mkdir(parents=True, exist_ok=True)
    srv_dir.mkdir(parents=True, exist_ok=True)

    for msg in messages:
        msg_path = msg_dir / f"{msg.name}.msg"
        msg_path.write_text(_render_ros_message(msg))

    for msg in messages:
        if not msg.tx_enabled:
            continue
        srv_path = srv_dir / f"Send{msg.name}.srv"
        srv_path.write_text(_render_ros_service(msg))


def generate(
    config_path: Path,
    out_dir: Path,
    options_path: Path,
    nodes_path: Path,
    tx_queue_override: Optional[int],
    ros_out_dir: Optional[Path],
) -> None:
    config = _load_config(config_path)
    node_timeouts, queue_size = _parse_nodes(config)
    if tx_queue_override is not None:
        queue_size = tx_queue_override
    if queue_size is None:
        raise ValueError("tx_queue_size must be provided in config or via --tx-queue-size")
    if queue_size <= 0:
        raise ValueError("tx_queue_size must be > 0")

    nodes = _load_nodes(nodes_path)
    config_node_ids = set(node_timeouts.keys())
    node_ids = {node.node_id for node in nodes}
    missing = node_ids - config_node_ids
    extra = config_node_ids - node_ids
    if missing:
        missing_list = ", ".join(f"0x{node_id:02X}" for node_id in sorted(missing))
        raise ValueError(f"nodes.json is missing nodes for IDs: {missing_list}")
    if extra:
        extra_list = ", ".join(f"0x{node_id:02X}" for node_id in sorted(extra))
        raise ValueError(f"nodes.json has no names for config IDs: {extra_list}")

    options = _load_options(options_path)
    messages = _build_messages(options)
    enums = list(r2bus_pb2.DESCRIPTOR.enum_types_by_name.values())

    header_text = _render_header(messages, enums, nodes, queue_size)
    source_text = _render_source(messages, nodes, node_timeouts)

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "r2bus_generated.h").write_text(header_text)
    (out_dir / "r2bus_generated.c").write_text(source_text)
    if ros_out_dir is not None:
        _write_ros_interfaces(messages, ros_out_dir)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate stdlib-only C helpers for r2bus protobuf messages.",
    )
    parser.add_argument(
        "--config",
        required=True,
        type=Path,
        help="JSON config containing node timeouts and tx_queue_size.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Output directory for generated C files.",
    )
    parser.add_argument(
        "--options",
        type=Path,
        default=PROTO_DIR / "r2bus.options",
        help="nanopb options file used for bytes sizing.",
    )
    parser.add_argument(
        "--nodes",
        type=Path,
        default=Path(__file__).resolve().parent / "nodes.json",
        help="JSON mapping node names to IDs or a nodes list.",
    )
    parser.add_argument(
        "--tx-queue-size",
        type=int,
        default=None,
        help="Override tx_queue_size from JSON config.",
    )
    parser.add_argument(
        "--ros-out",
        type=Path,
        default=None,
        help="Optional ROS 2 package directory to write msg/srv interfaces.",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    generate(
        args.config,
        args.out_dir,
        args.options,
        args.nodes,
        args.tx_queue_size,
        args.ros_out,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

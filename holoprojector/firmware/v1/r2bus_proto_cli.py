#!/usr/bin/env python3
"""Interactive helper for building and sending R2 bus protobuf messages."""

from __future__ import annotations

import argparse
import base64
import json
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Tuple, Type

try:
    from google.protobuf import json_format
    from google.protobuf.descriptor import FieldDescriptor
    from google.protobuf.message import Message
except ImportError:
    sys.exit("Google protobuf is required: pip install protobuf")

PROTO_DIR = Path(__file__).resolve().with_name("proto")
if PROTO_DIR.exists():
    sys.path.insert(0, str(PROTO_DIR))

try:
    import r2bus_pb2
except ImportError as exc:  # pragma: no cover - import dependency hint
    sys.exit(f"Unable to import R2 bus protobuf module: {exc}")

FD = FieldDescriptor


def _camel_to_enum_name(name: str) -> str:
    snake = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name).upper()
    snake = snake.replace("REQUEST", "REQ").replace("COMMAND", "CMD")
    return f"MESSAGE_ID_{snake}"


def _build_message_map() -> Dict[int, Type[Message]]:
    mapping: Dict[int, Type[Message]] = {}
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
        mapping[msg_id] = getattr(r2bus_pb2, desc.name)
    return mapping


MESSAGE_BY_ID: Dict[int, Type[Message]] = _build_message_map()
CLASS_TO_ID: Dict[str, int] = {
    cls.DESCRIPTOR.name: msg_id for msg_id, cls in MESSAGE_BY_ID.items()
}


def _normalize_key(value: str) -> str:
    return value.replace("_", "").replace("-", "").replace(" ", "").lower()


def _build_lookup() -> Dict[str, Tuple[int, Type[Message]]]:
    lookup: Dict[str, Tuple[int, Type[Message]]] = {}
    for msg_id, cls in MESSAGE_BY_ID.items():
        descriptor = cls.DESCRIPTOR
        enum_name = r2bus_pb2.MessageId.Name(msg_id)
        aliases = {
            descriptor.name,
            descriptor.full_name,
            enum_name,
            enum_name.removeprefix("MESSAGE_ID_"),
        }
        if descriptor.name == "Empty":
            aliases.update({"ecu_reset", "reset", "empty"})
        for alias in aliases:
            lookup[_normalize_key(alias)] = (msg_id, cls)
    return lookup


MESSAGE_LOOKUP = _build_lookup()

FIELD_TYPE_NAMES = {
    FD.TYPE_BOOL: "bool",
    FD.TYPE_BYTES: "bytes",
    FD.TYPE_DOUBLE: "double",
    FD.TYPE_ENUM: "enum",
    FD.TYPE_FIXED32: "fixed32",
    FD.TYPE_FIXED64: "fixed64",
    FD.TYPE_FLOAT: "float",
    FD.TYPE_INT32: "int32",
    FD.TYPE_INT64: "int64",
    FD.TYPE_SFIXED32: "sfixed32",
    FD.TYPE_SFIXED64: "sfixed64",
    FD.TYPE_SINT32: "sint32",
    FD.TYPE_SINT64: "sint64",
    FD.TYPE_STRING: "string",
    FD.TYPE_UINT32: "uint32",
    FD.TYPE_UINT64: "uint64",
}


def _parse_assignment(arg: str) -> Tuple[str, str]:
    if "=" not in arg:
        raise ValueError(f"Expected field=value assignment, got '{arg}'")
    field, value = arg.split("=", 1)
    field = field.strip()
    if not field:
        raise ValueError(f"Missing field name for assignment '{arg}'")
    return field, value


def _parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "on"}:
        return True
    if lowered in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Cannot parse boolean value '{value}'")


def _parse_enum(field: FieldDescriptor, raw: str) -> int:
    try:
        return int(raw, 0)
    except ValueError:
        pass
    try:
        return field.enum_type.values_by_name[raw.upper()].number
    except KeyError as exc:
        options = ", ".join(sorted(field.enum_type.values_by_name))
        raise ValueError(
            f"Invalid enum value '{raw}' for {field.name}. "
            f"Expected one of: {options}"
        ) from exc


def _parse_bytes(raw: str) -> bytes:
    trimmed = raw.strip()
    if trimmed.startswith("@"):
        with open(trimmed[1:], "rb") as handle:
            return handle.read()
    if trimmed.startswith("hex:"):
        cleaned = trimmed[4:].replace(" ", "")
        if len(cleaned) % 2:
            raise ValueError("Hex payload must have an even number of digits")
        return bytes.fromhex(cleaned)
    if trimmed.startswith("text:"):
        return trimmed[5:].encode("utf-8")
    if trimmed.startswith("base64:"):
        return base64.b64decode(trimmed[7:])
    if trimmed.startswith("[") and trimmed.endswith("]"):
        items = trimmed[1:-1].split(",")
        return bytes(int(item.strip(), 0) for item in items if item.strip())
    return trimmed.encode("utf-8")


def _parse_value(field: FieldDescriptor, raw: str):
    if field.type in {
        FD.TYPE_UINT32,
        FD.TYPE_UINT64,
        FD.TYPE_FIXED32,
        FD.TYPE_FIXED64,
    }:
        num = int(raw, 0)
        if num < 0:
            raise ValueError(f"Field {field.name} expects an unsigned value")
        return num
    if field.type in {
        FD.TYPE_INT32,
        FD.TYPE_INT64,
        FD.TYPE_SFIXED32,
        FD.TYPE_SFIXED64,
        FD.TYPE_SINT32,
        FD.TYPE_SINT64,
    }:
        return int(raw, 0)
    if field.type in {FD.TYPE_FLOAT, FD.TYPE_DOUBLE}:
        return float(raw)
    if field.type == FD.TYPE_BOOL:
        return _parse_bool(raw)
    if field.type == FD.TYPE_ENUM:
        return _parse_enum(field, raw)
    if field.type == FD.TYPE_STRING:
        return raw
    if field.type == FD.TYPE_BYTES:
        return _parse_bytes(raw)
    raise ValueError(f"Unsupported field type for {field.name}")


def _build_message(cls: Type[Message], assignments: Iterable[str]) -> Message:
    message = cls()
    descriptor = message.DESCRIPTOR
    if not descriptor.fields and list(assignments):
        raise ValueError(f"{descriptor.name} does not accept fields")
    for assignment in assignments:
        field_name, raw_value = _parse_assignment(assignment)
        try:
            field = descriptor.fields_by_name[field_name]
        except KeyError as exc:
            available = ", ".join(f.name for f in descriptor.fields)
            raise ValueError(
                f"Unknown field '{field_name}' for {descriptor.name}. "
                f"Available fields: {available or '-'}"
            ) from exc
        parsed = _parse_value(field, raw_value)
        if field.label == FD.LABEL_REPEATED:
            target = getattr(message, field.name)
            if field.type == FD.TYPE_MESSAGE:
                raise ValueError("Repeated message fields are not supported")
            target.append(parsed)
            continue
        setattr(message, field.name, parsed)
    return message


def _resolve_message(spec: str) -> Tuple[int, Type[Message]]:
    try:
        msg_id = int(spec, 0)
        cls = MESSAGE_BY_ID.get(msg_id)
        if cls:
            return msg_id, cls
    except ValueError:
        pass

    normalized = _normalize_key(spec)
    resolved = MESSAGE_LOOKUP.get(normalized)
    if resolved:
        return resolved

    enum_name = spec if spec.startswith("MESSAGE_ID_") else f"MESSAGE_ID_{spec.upper()}"
    try:
        msg_id = r2bus_pb2.MessageId.Value(enum_name)
    except ValueError as exc:
        raise ValueError(f"Unknown message specifier '{spec}'") from exc
    cls = MESSAGE_BY_ID.get(msg_id)
    if not cls:
        raise ValueError(f"No protobuf message bound to ID {enum_name}")
    return msg_id, cls


def _field_summary(field: FieldDescriptor) -> str:
    type_name = FIELD_TYPE_NAMES.get(field.type, str(field.type))
    if field.type == FD.TYPE_ENUM:
        choices = "/".join(value.name for value in field.enum_type.values)
        type_name = f"enum[{choices}]"
    label = "repeated " if field.label == FD.LABEL_REPEATED else ""
    return f"{field.name}:{label}{type_name}"


def list_messages(_: argparse.Namespace) -> int:
    for msg_id in sorted(MESSAGE_BY_ID):
        cls = MESSAGE_BY_ID[msg_id]
        descriptor = cls.DESCRIPTOR
        fields = ", ".join(_field_summary(f) for f in descriptor.fields) or "-"
        name = r2bus_pb2.MessageId.Name(msg_id)
        print(f"0x{msg_id:02X} {name:<24} -> {descriptor.name} ({fields})")
    return 0


def build_command(args: argparse.Namespace) -> int:
    try:
        msg_id, cls = _resolve_message(args.message)
        message = _build_message(cls, args.fields)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    payload = message.SerializeToString()
    name = r2bus_pb2.MessageId.Name(msg_id)
    print(f"{descriptor_name(message)} id={name} (0x{msg_id:02X})")
    if args.show_json:
        json_obj = json_format.MessageToDict(
            message,
            preserving_proto_field_name=True,
            including_default_value_fields=args.include_defaults,
        )
        print(json.dumps(json_obj, indent=2))
    print(f"payload[{len(payload)}]={format_hex(payload)}")
    if payload:
        print(f"base64:{base64.b64encode(payload).decode()}")
    return 0


def descriptor_name(message: Message) -> str:
    descriptor = message.DESCRIPTOR
    return descriptor.full_name or descriptor.name


def format_hex(payload: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in payload) or "-"


def _import_r2bus_tool():
    try:
        import r2bus_tool  # type: ignore
    except SystemExit as exc:  # pragma: no cover - propagate dependency hints
        raise RuntimeError(str(exc))
    except ImportError as exc:  # pragma: no cover - propagate dependency hints
        raise RuntimeError(f"Unable to import r2bus_tool: {exc}")
    return r2bus_tool


def _parse_node_id(value: str) -> int:
    node_id = int(value, 0)
    if not 0 <= node_id <= 0xFF:
        raise argparse.ArgumentTypeError("Node ID must be between 0 and 0xFF")
    return node_id


def send_command(args: argparse.Namespace) -> int:
    try:
        msg_id, cls = _resolve_message(args.message)
        message = _build_message(cls, args.fields)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    payload = message.SerializeToString()
    name = r2bus_pb2.MessageId.Name(msg_id)

    try:
        r2bus_tool = _import_r2bus_tool()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    if args.show_json:
        json_obj = json_format.MessageToDict(
            message,
            preserving_proto_field_name=True,
            including_default_value_fields=args.include_defaults,
        )
        print(json.dumps(json_obj, indent=2))

    print(
        f"Sending {descriptor_name(message)} "
        f"(id={name}, dest=0x{args.dest:02X}, src=0x{args.src:02X}) "
        f"payload[{len(payload)}]={format_hex(payload)}"
    )

    bus = r2bus_tool.R2BusSerial(args.port, args.baud, args.timeout)
    try:
        bus.send_frame(args.dest, msg_id, payload, src=args.src)
        if args.wait_ack and args.dest != r2bus_tool.BROADCAST_ID:
            ok = r2bus_tool.wait_for_ack(
                bus,
                dest=args.dest,
                msg_id=msg_id,
                timeout=args.ack_timeout,
            )
            if ok:
                print("ACK received")
                return 0
            print("No ACK received", file=sys.stderr)
            return 1
    finally:
        bus.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build and send R2 bus protobuf messages based on r2bus.proto",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    list_p = sub.add_parser("list", help="List available message IDs and fields")
    list_p.set_defaults(func=list_messages)

    def add_message_common(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "message",
            help="Message name or ID (e.g. Ping, psi_color_request, MESSAGE_ID_PING, 0x02)",
        )
        p.add_argument(
            "fields",
            nargs="*",
            help="Field assignments in the form field=value "
            "(bytes: use hex:AAFF, text:hello, base64:... or @[file])",
        )
        p.add_argument(
            "--show-json",
            action="store_true",
            help="Print the protobuf message as JSON",
        )
        p.add_argument(
            "--include-defaults",
            action="store_true",
            help="Include default-valued fields in JSON output",
        )

    build_p = sub.add_parser("build", help="Construct a message and show the payload")
    add_message_common(build_p)
    build_p.set_defaults(func=build_command)

    send_p = sub.add_parser("send", help="Build a message and send it over the bus")
    add_message_common(send_p)
    send_p.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: %(default)s)")
    send_p.add_argument("--baud", default=115200, type=int, help="Baud rate (default: %(default)s)")
    send_p.add_argument("--timeout", default=0.1, type=float, help="Serial IO timeout")
    send_p.add_argument("--src", default=0x00, type=_parse_node_id, help="Source node ID (default: host)")
    send_p.add_argument(
        "--dest",
        required=True,
        type=_parse_node_id,
        help="Destination node ID (e.g. 0x10)",
    )
    send_p.add_argument(
        "--wait-ack",
        action="store_true",
        help="Wait for an ACK from the destination",
    )
    send_p.add_argument(
        "--ack-timeout",
        default=1.0,
        type=float,
        help="Timeout in seconds when waiting for ACK (default: %(default)s)",
    )
    send_p.set_defaults(func=send_command)

    return parser


def main(argv: List[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

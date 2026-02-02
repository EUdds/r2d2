#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

try:
    import serial  # type: ignore
except ImportError:
    serial = None

try:
    from google.protobuf.message import DecodeError
except ImportError:
    DecodeError = None

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory

import interfaces.msg as ros_msgs
import interfaces.srv as ros_srvs

from .firmware_updater import FirmwareUpdater, FirmwarePackage, FirmwareEntry, UpdateResult, ProgressUpdate


def _find_proto_dir() -> Optional[Path]:
    env_dir = os.environ.get("R2BUS_PROTO_DIR")
    if env_dir:
        candidate = Path(env_dir)
        if (candidate / "r2bus_pb2.py").exists():
            return candidate

    try:
        share_dir = Path(get_package_share_directory("gateway"))
        candidate = share_dir / "proto"
        if (candidate / "r2bus_pb2.py").exists():
            return candidate
    except Exception:
        pass

    cwd_candidate = Path.cwd() / "firmware" / "libs" / "r2bus" / "proto"
    if (cwd_candidate / "r2bus_pb2.py").exists():
        return cwd_candidate

    repo_candidate = Path(__file__).resolve().parents[2] / "firmware" / "libs" / "r2bus" / "proto"
    if (repo_candidate / "r2bus_pb2.py").exists():
        return repo_candidate

    return None


def _find_nodes_path() -> Optional[Path]:
    env_path = os.environ.get("R2BUS_NODES_PATH")
    if env_path:
        candidate = Path(env_path)
        if candidate.exists():
            return candidate

    try:
        share_dir = Path(get_package_share_directory("gateway"))
        candidate = share_dir / "config" / "nodes.json"
        if candidate.exists():
            return candidate
    except Exception:
        pass

    cwd_candidate = Path.cwd() / "r2bus_codegen" / "nodes.json"
    if cwd_candidate.exists():
        return cwd_candidate

    repo_candidate = Path(__file__).resolve().parents[2] / "r2bus_codegen" / "nodes.json"
    if repo_candidate.exists():
        return repo_candidate

    return None


PROTO_DIR = _find_proto_dir()
if PROTO_DIR is not None:
    sys.path.insert(0, str(PROTO_DIR))

try:
    import r2bus_pb2
except ImportError as exc:  # pragma: no cover - environment setup
    r2bus_pb2 = None
    _PROTO_IMPORT_ERROR = exc
else:
    _PROTO_IMPORT_ERROR = None

SYNC0 = 0x55
SYNC1 = 0xAA
CRC_SEED = 0xFFFF
CRC_POLY = 0x1021

HOST_ID = 0x00
BROADCAST_ID = 0xFF


@dataclass
class Frame:
    dest: int
    src: int
    msg_id: int
    length: int
    payload: bytes
    crc: int


def crc16_ccitt(data: bytes, seed: int = CRC_SEED) -> int:
    crc = seed
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ CRC_POLY
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc & 0xFFFF


class R2BusSerial:
    def __init__(self, port: str, baud: int, timeout: float) -> None:
        if serial is None:
            raise RuntimeError("PySerial is required (python3-serial)")
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=timeout,
            write_timeout=timeout,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._buffer = bytearray()
        self._lock = threading.Lock()

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _extract_frame(self) -> Optional[Frame]:
        buf = self._buffer
        while True:
            if len(buf) < 2:
                return None
            if buf[0] != SYNC0:
                del buf[0]
                continue
            if buf[1] != SYNC1:
                del buf[0]
                continue
            if len(buf) < 6:
                return None
            dest = buf[2]
            src = buf[3]
            msg_id = buf[4]
            length = buf[5]
            frame_len = 2 + 4 + length + 2
            if len(buf) < frame_len:
                return None
            payload = bytes(buf[6 : 6 + length])
            crc_lsb = buf[6 + length]
            crc_msb = buf[7 + length]
            crc_rx = crc_lsb | (crc_msb << 8)
            crc_calc = crc16_ccitt(bytes(buf[2 : 6 + length]))
            del buf[:frame_len]
            if crc_rx != crc_calc:
                continue
            return Frame(
                dest=dest,
                src=src,
                msg_id=msg_id,
                length=length,
                payload=payload,
                crc=crc_rx,
            )

    def read_frame(self, timeout: Optional[float]) -> Optional[Frame]:
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            frame = self._extract_frame()
            if frame:
                return frame
            chunk = self._serial.read(128)
            if chunk:
                self._buffer.extend(chunk)
                continue
            if deadline is not None and time.monotonic() >= deadline:
                return None

    def send_frame(self, dest: int, msg_id: int, payload: bytes, *, src: int) -> None:
        if len(payload) > 255:
            raise ValueError("Payload too large (max 255 bytes)")
        header = bytes([dest, src, msg_id, len(payload)])
        crc = crc16_ccitt(header + payload)
        frame = bytes([SYNC0, SYNC1]) + header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        with self._lock:
            self._serial.write(frame)
            self._serial.flush()


@dataclass(frozen=True)
class MessageMeta:
    name: str
    msg_id: int
    proto_cls: object
    ros_cls: object


def _camel_to_enum_name(name: str) -> str:
    snake = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name).upper()
    snake = snake.replace("REQUEST", "REQ").replace("COMMAND", "CMD")
    return f"MESSAGE_ID_{snake}"


def _sanitize_node_name(name: str) -> str:
    sanitized = re.sub(r"[^0-9A-Za-z]+", "_", name.strip()).lower()
    sanitized = sanitized.strip("_")
    if not sanitized:
        return "node"
    if sanitized[0].isdigit():
        sanitized = f"node_{sanitized}"
    return sanitized


def _load_nodes(path: Optional[Path]) -> Dict[int, str]:
    if path is None or not path.exists():
        return {}
    data = json.loads(path.read_text())
    raw_nodes = []
    if isinstance(data, dict) and "nodes" in data:
        entries = data["nodes"]
        if isinstance(entries, list):
            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                name = entry.get("name")
                node_id = entry.get("id")
                if name is None or node_id is None:
                    continue
                raw_nodes.append((name, node_id))
    elif isinstance(data, dict):
        raw_nodes = list(data.items())
    node_map: Dict[int, str] = {}
    for name, node_id_value in raw_nodes:
        if not isinstance(name, str):
            continue
        if isinstance(node_id_value, int):
            node_id = node_id_value
        elif isinstance(node_id_value, str):
            node_id = int(node_id_value, 0)
        else:
            continue
        if 0 <= node_id <= 0xFF:
            node_map[node_id] = _sanitize_node_name(name)
    return node_map


def _build_message_map() -> Dict[int, MessageMeta]:
    if r2bus_pb2 is None:
        raise RuntimeError(f"Unable to import r2bus_pb2: {_PROTO_IMPORT_ERROR}")
    message_map: Dict[int, MessageMeta] = {}
    for desc in r2bus_pb2.DESCRIPTOR.message_types_by_name.values():
        name = desc.name
        if name == "Empty":
            msg_id = r2bus_pb2.MessageId.MESSAGE_ID_ECU_RESET
        else:
            enum_name = _camel_to_enum_name(name)
            try:
                msg_id = r2bus_pb2.MessageId.Value(enum_name)
            except ValueError:
                continue
        proto_cls = getattr(r2bus_pb2, name, None)
        ros_cls = getattr(ros_msgs, name, None)
        if proto_cls is None or ros_cls is None:
            continue
        message_map[msg_id] = MessageMeta(name=name, msg_id=msg_id, proto_cls=proto_cls, ros_cls=ros_cls)
    return message_map


def _proto_to_ros(proto_msg: object, ros_cls: object) -> object:
    ros_msg = ros_cls()
    for field in proto_msg.DESCRIPTOR.fields:
        value = getattr(proto_msg, field.name)
        if field.type == field.TYPE_BYTES:
            setattr(ros_msg, field.name, list(value))
        else:
            setattr(ros_msg, field.name, value)
    return ros_msg


def _ros_to_proto(ros_msg: object, proto_cls: object) -> object:
    proto_msg = proto_cls()
    for field in proto_msg.DESCRIPTOR.fields:
        value = getattr(ros_msg, field.name)
        if field.type == field.TYPE_BYTES:
            setattr(proto_msg, field.name, bytes(value))
        else:
            setattr(proto_msg, field.name, value)
    return proto_msg


class R2BusGateway(Node):
    def __init__(self) -> None:
        super().__init__("r2bus_gateway")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("host_id", HOST_ID)
        self.declare_parameter("dest_id", BROADCAST_ID)
        default_nodes_path = _find_nodes_path()
        self.declare_parameter(
            "nodes_path",
            str(default_nodes_path) if default_nodes_path else "",
        )
        self.declare_parameter("publish_per_node", True)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._host_id = int(self.get_parameter("host_id").get_parameter_value().integer_value)
        self._dest_id = int(self.get_parameter("dest_id").get_parameter_value().integer_value)
        nodes_path_value = self.get_parameter("nodes_path").get_parameter_value().string_value
        nodes_path = Path(nodes_path_value) if nodes_path_value else None
        self._per_node_topics = bool(
            self.get_parameter("publish_per_node").get_parameter_value().bool_value
        )

        if r2bus_pb2 is None or DecodeError is None:
            raise RuntimeError(f"protobuf is required: {_PROTO_IMPORT_ERROR}")

        self._messages = _build_message_map()
        self._node_names = _load_nodes(nodes_path) if nodes_path else {}

        self._rx_publishers: Dict[int, object] = {}
        self._per_node_rx_publishers: Dict[Tuple[int, int], object] = {}
        self._tx_subscriptions = []

        for msg_id, meta in self._messages.items():
            topic = f"r2bus/{meta.name}"
            self._rx_publishers[msg_id] = self.create_publisher(meta.ros_cls, topic, 10)

        self._bus = R2BusSerial(port=port, baud=baud, timeout=0.1)
        self.get_logger().info(f"Opened R2BUS serial on {port} @ {baud} baud")

        for meta in self._messages.values():
            topic = f"r2bus/{meta.name}/tx"
            self._tx_subscriptions.append(
                self.create_subscription(
                    meta.ros_cls,
                    topic,
                    lambda msg, meta=meta: self._handle_outbound(meta, self._dest_id, msg),
                    10,
                )
            )

        if self._per_node_topics and self._node_names:
            for node_id, node_name in self._node_names.items():
                for meta in self._messages.values():
                    topic = f"r2bus/{node_name}/{meta.name}/tx"
                    self._tx_subscriptions.append(
                        self.create_subscription(
                            meta.ros_cls,
                            topic,
                            lambda msg, meta=meta, dest_id=node_id: self._handle_outbound(
                                meta, dest_id, msg
                            ),
                            10,
                        )
                    )

        self._stop_event = threading.Event()
        
        self._update_lock = threading.Lock()
        self._update_in_progress = False
        
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()


        # Load firmware package from deployment directory
        self._firmware_package: Optional[FirmwarePackage] = None
        deploy_dir = Path("/opt/deploy")
        if deploy_dir.exists():
            try:
                self._firmware_package = FirmwarePackage.from_dir(deploy_dir)
                self.get_logger().info(
                    f"Loaded firmware package from {deploy_dir}: {len(self._firmware_package.entries)} entries"
                )
            except Exception as exc:
                self.get_logger().warning(f"Failed to load firmware package: {exc}")
        else:
            self.get_logger().warning(f"Deployment directory not found: {deploy_dir}")

        self._firmware_service_group = MutuallyExclusiveCallbackGroup()
        self._firmware_update_service = self.create_service(
            ros_srvs.FirmwareUpdate,
            "r2bus/firmware_update",
            self._handle_firmware_update,
            callback_group=self._firmware_service_group,
        )
        self.get_logger().info("Firmware update service ready on r2bus/firmware_update")
        
        self._firmware_info_service = self.create_service(
            ros_srvs.GetAvailableFirmware,
            "r2bus/get_available_firmware",
            self._handle_get_available_firmware,
            callback_group=self._firmware_service_group,
        )
        self.get_logger().info("Get available firmware service ready on r2bus/get_available_firmware")
        
        # Publisher for firmware update progress
        self._firmware_progress_publisher = self.create_publisher(
            ros_msgs.FirmwareProgress,
            "r2bus/firmware_progress",
            10,
        )
        self.get_logger().info("Firmware progress publisher ready on r2bus/firmware_progress")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self._bus:
            self._bus.close()
        return super().destroy_node()

    def _handle_outbound(self, meta: MessageMeta, dest_id: int, ros_msg: object) -> None:
        try:
            proto = _ros_to_proto(ros_msg, meta.proto_cls)
            payload = proto.SerializeToString()
            self._bus.send_frame(
                dest=int(dest_id),
                msg_id=meta.msg_id,
                payload=payload,
                src=self._host_id,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send {meta.name} to {dest_id:02X}: {exc}"
            )

    def _publish_for_node(self, msg_id: int, src: int, ros_msg: object) -> None:
        if not self._per_node_topics:
            return
        key = (msg_id, src)
        publisher = self._per_node_rx_publishers.get(key)
        if publisher is None:
            node_name = self._node_names.get(src, f"node_{src:02x}")
            topic = f"r2bus/{node_name}/{self._messages[msg_id].name}"
            publisher = self.create_publisher(self._messages[msg_id].ros_cls, topic, 10)
            self._per_node_rx_publishers[key] = publisher
        publisher.publish(ros_msg)

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set() and rclpy.ok():
            with self._update_lock:
                if self._update_in_progress:
                    time.sleep(0.1)
                    continue
            frame = self._bus.read_frame(timeout=0.1)
            if frame is None:
                continue
            meta = self._messages.get(frame.msg_id)
            if not meta:
                continue
            try:
                proto = meta.proto_cls()
                proto.ParseFromString(frame.payload)
            except DecodeError:
                self.get_logger().warning(
                    f"Failed to decode {meta.name} from {frame.src:02X}"
                )
                continue
            ros_msg = _proto_to_ros(proto, meta.ros_cls)
            self._rx_publishers[frame.msg_id].publish(ros_msg)
            self._publish_for_node(frame.msg_id, frame.src, ros_msg)

    def _handle_firmware_update(
        self,
        request: ros_srvs.FirmwareUpdate.Request,
        response: ros_srvs.FirmwareUpdate.Response,
    ) -> ros_srvs.FirmwareUpdate.Response:
        node_id = request.node_id

        self.get_logger().info(
            f"Firmware update request: node=0x{node_id:02X}, path={request.firmware_path}"
        )

        with self._update_lock:
            if self._update_in_progress:
                response.success = False
                response.message = "Another firmware update is already in progress"
                response.bytes_written = 0
                return response
            self._update_in_progress = True

        try:
            time.sleep(0.2)

            # Resolve firmware path: explicit path or lookup from CSV
            firmware_path = self._resolve_firmware_path(node_id, request.firmware_path)
            if firmware_path is None:
                response.success = False
                response.message = "Firmware path not specified and no entry found in deployment metadata"
                response.bytes_written = 0
                return response

            def log_callback(msg: str) -> None:
                self.get_logger().info(f"[FW Update 0x{node_id:02X}] {msg}")

            def progress_callback(update: ProgressUpdate) -> None:
                msg = ros_msgs.FirmwareProgress()
                msg.node_id = node_id
                msg.step = update.step
                msg.bytes_processed = update.bytes_processed
                msg.bytes_total = update.bytes_total
                msg.message = update.message
                if update.bytes_total > 0:
                    msg.percent = int((update.bytes_processed / update.bytes_total) * 100)
                else:
                    msg.percent = 0
                self._firmware_progress_publisher.publish(msg)
                self.get_logger().debug(f"[FW Progress 0x{node_id:02X}] {update.step}: {update.message}")

            updater = FirmwareUpdater(
                ser=self._bus._serial,
                host_id=self._host_id,
                chunk_size=512,
                timeout=1.5,
                command_retries=2,
                ping_retries=80,
                ping_delay=0.25,
                log_callback=log_callback,
                progress_callback=progress_callback,
            )

            result = updater.update(
                node_id=node_id,
                firmware_path=firmware_path,
                skip_reset=request.skip_reset,
                skip_boot=request.skip_boot,
                r2bus_pb2=r2bus_pb2,
            )

            response.success = result.success
            response.message = result.message
            response.bytes_written = result.bytes_written

            if result.success:
                self.get_logger().info(
                    f"Firmware update complete for 0x{node_id:02X}: {result.bytes_written} bytes"
                )
            else:
                self.get_logger().error(
                    f"Firmware update failed for 0x{node_id:02X}: {result.message}"
                )

        except Exception as exc:
            response.success = False
            response.message = f"Unexpected error: {exc}"
            response.bytes_written = 0
            self.get_logger().error(f"Firmware update exception: {exc}")

        finally:
            self._bus._serial.reset_input_buffer()
            self._bus._serial.reset_output_buffer()
            with self._update_lock:
                self._update_in_progress = False

        return response

    def _handle_get_available_firmware(
        self,
        request: ros_srvs.GetAvailableFirmware.Request,
        response: ros_srvs.GetAvailableFirmware.Response,
    ) -> ros_srvs.GetAvailableFirmware.Response:
        """Return list of available firmware from deployment package."""
        if self._firmware_package:
            for entry in self._firmware_package.entries:
                fw_info = ros_msgs.FirmwareInfo()
                fw_info.node_id = entry.node_id
                fw_info.node_name = entry.node_name or ""
                fw_info.pcb_rev = entry.pcb_rev or ""
                fw_info.artifact = str(entry.artifact)
                fw_info.sha256 = entry.sha256 or ""
                response.firmware.append(fw_info)
        return response

    def _resolve_firmware_path(self, node_id: int, explicit_path: str) -> Optional[Path]:
        """Resolve firmware path from explicit path or CSV lookup.
        
        Returns the Path to firmware file, or None if not found.
        """
        # If explicit path provided, use it
        if explicit_path.strip():
            path = Path(explicit_path)
            if path.exists():
                return path
            self.get_logger().warning(f"Explicit firmware path not found: {explicit_path}")
            return None

        # Try to resolve from firmware package CSV
        if self._firmware_package is None:
            return None

        entry = self._firmware_package.get_for_node(node_id)
        if entry is None:
            self.get_logger().warning(
                f"No firmware entry found in CSV for node 0x{node_id:02X}"
            )
            return None

        if not entry.artifact.exists():
            self.get_logger().warning(f"Firmware artifact not found: {entry.artifact}")
            return None

        self.get_logger().info(
            f"Resolved firmware for 0x{node_id:02X} from CSV: {entry.artifact.name}"
        )
        return entry.artifact


def main(argv: Optional[list[str]] = None) -> int:
    rclpy.init(args=argv)
    try:
        node = R2BusGateway()
    except Exception as exc:
        print(f"Failed to start r2bus_gateway: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

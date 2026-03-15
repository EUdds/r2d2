#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
except ImportError as exc:  # pragma: no cover - environment setup
    FastAPI = None
    WebSocket = None
    WebSocketDisconnect = None
    CORSMiddleware = None
    uvicorn = None
    _WEB_IMPORT_ERROR = exc
else:
    _WEB_IMPORT_ERROR = None

import interfaces.msg as ros_msgs
import interfaces.srv as ros_srvs


BROADCAST_ID = 0xFF

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


@dataclass(frozen=True)
class MessageMeta:
    name: str
    ros_cls: object


@dataclass
class OutboundMessage:
    payload: dict
    target: Optional[WebSocket]

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


def _build_message_map() -> Dict[str, MessageMeta]:
    if r2bus_pb2 is None:
        raise RuntimeError(f"Unable to import r2bus_pb2: {_PROTO_IMPORT_ERROR}")
    message_map: Dict[str, MessageMeta] = {}
    for desc in r2bus_pb2.DESCRIPTOR.message_types_by_name.values():
        name = desc.name
        ros_cls = getattr(ros_msgs, name, None)
        if ros_cls is None:
            continue
        message_map[name] = MessageMeta(name=name, ros_cls=ros_cls)
    return message_map


def _ros_msg_to_dict(msg: object) -> dict:
    data: Dict[str, object] = {}
    for field in msg.__slots__:
        value = getattr(msg, field)
        if isinstance(value, (list, tuple)):
            data[field] = list(value)
        else:
            data[field] = value
    return data


def _apply_payload(msg: object, payload: dict) -> None:
    for key, value in payload.items():
        if not hasattr(msg, key):
            raise ValueError(f"Unknown field '{key}'")
        setattr(msg, key, value)


class WebBridgeNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__("r2bus_web_bridge")
        self._loop = loop

        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8090)
        self.declare_parameter("cors_origins", ["*"])
        default_nodes_path = _find_nodes_path()
        self.declare_parameter(
            "nodes_path",
            str(default_nodes_path) if default_nodes_path else "",
        )
        self.declare_parameter("subscribe_per_node", True)

        if FastAPI is None or uvicorn is None:
            raise RuntimeError(
                "fastapi/uvicorn is required. Install with: "
                "sudo apt install python3-fastapi python3-uvicorn"
            )
        if r2bus_pb2 is None:
            raise RuntimeError(f"protobuf is required: {_PROTO_IMPORT_ERROR}")

        self._message_map = _build_message_map()
        nodes_path_value = self.get_parameter("nodes_path").get_parameter_value().string_value
        nodes_path = Path(nodes_path_value) if nodes_path_value else None
        self._node_names = _load_nodes(nodes_path)
        self._subscribe_per_node = bool(
            self.get_parameter("subscribe_per_node").get_parameter_value().bool_value
        )

        self._tx_publishers: Dict[str, object] = {}
        self._node_tx_publishers: Dict[Tuple[int, str], object] = {}
        self._queue: Optional[asyncio.Queue[OutboundMessage]] = None
        self._ws_clients: Set[WebSocket] = set()

        # Create publishers for arm control (direct ROS 2 topics, not R2BUS)
        self._top_arm_publisher = self.create_publisher(ros_msgs.ArmRequest, 'top_arm_request', 10)
        self._bottom_arm_publisher = self.create_publisher(ros_msgs.ArmRequest, 'bottom_arm_request', 10)

        # Debug: verify publishers are different objects
        self.get_logger().info(f"Top arm publisher ID: {id(self._top_arm_publisher)}, topic: {self._top_arm_publisher.topic_name}")
        self.get_logger().info(f"Bottom arm publisher ID: {id(self._bottom_arm_publisher)}, topic: {self._bottom_arm_publisher.topic_name}")

        # Create client to query gateway for firmware info
        self._firmware_client = self.create_client(
            ros_srvs.GetAvailableFirmware,
            "r2bus/get_available_firmware"
        )

        for name, meta in self._message_map.items():
            topic = f"r2bus/{name}/tx"
            self._tx_publishers[name] = self.create_publisher(meta.ros_cls, topic, 10)

        for name, meta in self._message_map.items():
            topic = f"r2bus/{name}"
            self.create_subscription(
                meta.ros_cls,
                topic,
                lambda msg, name=name, topic=topic: self._handle_message(name, topic, msg),
                10,
            )
            tx_topic = f"r2bus/{name}/tx"
            self.create_subscription(
                meta.ros_cls,
                tx_topic,
                lambda msg, name=name, topic=tx_topic: self._handle_message(name, topic, msg),
                10,
            )
        
        # Subscribe to firmware progress updates
        self.create_subscription(
            ros_msgs.FirmwareProgress,
            "r2bus/firmware_progress",
            self._handle_firmware_progress,
            10,
        )

        if self._subscribe_per_node and self._node_names:
            for node_id, node_name in self._node_names.items():
                for name, meta in self._message_map.items():
                    topic = f"r2bus/{node_name}/{name}"
                    self.create_subscription(
                        meta.ros_cls,
                        topic,
                        lambda msg, name=name, topic=topic, node_id=node_id: self._handle_node_message(
                            name, topic, node_id, msg
                        ),
                        10,
                    )
                    tx_topic = f"r2bus/{node_name}/{name}/tx"
                    self.create_subscription(
                        meta.ros_cls,
                        tx_topic,
                        lambda msg, name=name, topic=tx_topic, node_id=node_id: self._handle_node_message(
                            name, topic, node_id, msg
                        ),
                        10,
                    )

        self.app = self._build_app()

    def _handle_firmware_progress(self, msg: object) -> None:
        """Handle firmware progress updates and broadcast to all connected clients."""
        payload = {
            "type": "firmware_progress",
            "node_id": int(msg.node_id),
            "step": msg.step,
            "bytes_processed": int(msg.bytes_processed),
            "bytes_total": int(msg.bytes_total),
            "percent": int(msg.percent),
            "message": msg.message,
        }
        self._enqueue(payload, target=None)

    def _handle_message(self, name: str, topic: str, msg: object) -> None:
        payload = {
            "type": "telemetry",
            "topic": topic,
            "message": name,
            "timestamp": time.time(),
            "data": _ros_msg_to_dict(msg),
        }
        self._enqueue(payload, target=None)

    def _handle_node_message(self, name: str, topic: str, node_id: int, msg: object) -> None:
        payload = {
            "type": "telemetry",
            "topic": topic,
            "message": name,
            "node_id": node_id,
            "timestamp": time.time(),
            "data": _ros_msg_to_dict(msg),
        }
        self._enqueue(payload, target=None)

    def _enqueue(self, payload: dict, target: Optional[WebSocket]) -> None:
        if self._queue is None:
            return
        def _put() -> None:
            self._queue.put_nowait(OutboundMessage(payload=payload, target=target))

        self._loop.call_soon_threadsafe(_put)

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="R2BUS Web Bridge", version="0.1.0")

        origins = self.get_parameter("cors_origins").get_parameter_value().string_array_value
        app.add_middleware(
            CORSMiddleware,
            allow_origins=list(origins) if origins else ["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/health")
        async def health() -> dict:
            return {"ok": True, "time": time.time()}

        @app.get("/messages")
        async def messages() -> dict:
            return {
                "messages": sorted(self._message_map.keys()),
                "topics": [f"r2bus/{name}" for name in sorted(self._message_map.keys())],
                "nodes": self._node_names,
                "services": sorted(self._message_map.keys()),
                "tx_topics": [f"r2bus/{name}/tx" for name in sorted(self._message_map.keys())],
            }

        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket) -> None:
            await websocket.accept()
            self._ws_clients.add(websocket)
            
            # Query gateway for available firmware
            available_firmware = await self._get_available_firmware()
            
            await websocket.send_json(
                {
                    "type": "hello",
                    "messages": sorted(self._message_map.keys()),
                    "services": sorted(self._message_map.keys()),
                    "nodes": self._node_names,
                    "tx_topics": [f"r2bus/{name}/tx" for name in sorted(self._message_map.keys())],
                    "available_firmware": available_firmware,
                }
            )
            try:
                while True:
                    data = await websocket.receive_json()
                    await self._handle_ws_message(websocket, data)
            except WebSocketDisconnect:
                pass
            finally:
                self._ws_clients.discard(websocket)

        return app

    async def _handle_ws_message(self, websocket: WebSocket, data: dict) -> None:
        msg_type = data.get("type")
        if msg_type == "ping":
            await websocket.send_json({"type": "pong", "time": time.time()})
            return
        if msg_type == "service_call":
            await self._handle_service_call(websocket, data)
            return
        if msg_type == "arm_control":
            await self._handle_arm_control(websocket, data)
            return
        if msg_type != "send":
            await websocket.send_json({"type": "error", "error": "Unknown message type"})
            return

        request_id = data.get("request_id")
        name = data.get("message")
        dest_id = data.get("dest_id")
        payload = data.get("payload", {})

        if not isinstance(name, str) or name not in self._message_map:
            await websocket.send_json({"type": "error", "request_id": request_id, "error": "Unknown message"})
            return
        if not isinstance(dest_id, int) or not 0 <= dest_id <= 0xFF:
            await websocket.send_json({"type": "error", "request_id": request_id, "error": "Invalid dest_id"})
            return
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            await websocket.send_json({"type": "error", "request_id": request_id, "error": "Payload must be object"})
            return

        meta = self._message_map[name]
        ros_payload = meta.ros_cls()
        try:
            _apply_payload(ros_payload, payload)
        except ValueError as exc:
            await websocket.send_json({"type": "error", "request_id": request_id, "error": str(exc)})
            return

        try:
            topic, publisher = self._resolve_tx_publisher(name, dest_id)
        except ValueError as exc:
            await websocket.send_json({"type": "error", "request_id": request_id, "error": str(exc)})
            return
        publisher.publish(ros_payload)

        await websocket.send_json(
            {
                "type": "accepted",
                "request_id": request_id,
                "message": name,
                "dest_id": dest_id,
                "topic": topic,
            }
        )

    async def _handle_arm_control(self, websocket: WebSocket, data: dict) -> None:
        """Handle arm control requests from web UI."""
        request_id = data.get("request_id")
        arm = data.get("arm")  # "top_arm" or "bottom_arm"
        payload = data.get("payload", {})

        if arm not in ["top_arm", "bottom_arm"]:
            await websocket.send_json({
                "type": "error",
                "request_id": request_id,
                "error": "Invalid arm (must be 'top_arm' or 'bottom_arm')"
            })
            return

        if not isinstance(payload, dict):
            await websocket.send_json({
                "type": "error",
                "request_id": request_id,
                "error": "Payload must be object"
            })
            return

        # Create ArmRequest message
        msg = ros_msgs.ArmRequest()
        msg.open = bool(payload.get("open", False))
        msg.position_override = bool(payload.get("position_override", False))
        msg.angle = int(payload.get("angle", 0))

        # Publish to appropriate topic
        if arm == "top_arm":
            self._top_arm_publisher.publish(msg)
            topic = "top_arm_request"
            self.get_logger().info(f"Publishing to top_arm_request: open={msg.open}, pos_override={msg.position_override}, angle={msg.angle}")
        else:
            self._bottom_arm_publisher.publish(msg)
            topic = "bottom_arm_request"
            self.get_logger().info(f"Publishing to bottom_arm_request: open={msg.open}, pos_override={msg.position_override}, angle={msg.angle}")

        await websocket.send_json({
            "type": "accepted",
            "request_id": request_id,
            "arm": arm,
            "topic": topic,
        })
    async def _handle_service_call(self, websocket: WebSocket, data: dict) -> None:
        """Handle service call requests from web UI."""
        request_id = data.get("request_id")
        service = data.get("service")
        
        if service == "r2bus/firmware_update":
            await self._handle_firmware_update_call(websocket, request_id, data)
        else:
            await websocket.send_json({
                "type": "service_response",
                "request_id": request_id,
                "result": {
                    "success": False,
                    "message": f"Unknown service: {service}",
                    "bytes_written": 0,
                }
            })

    async def _handle_firmware_update_call(self, websocket: WebSocket, request_id: str, data: dict) -> None:
        """Call firmware update service on gateway."""
        try:
            # Create a client for the firmware update service
            client = self.create_client(ros_srvs.FirmwareUpdate, "r2bus/firmware_update")
            
            if not client.wait_for_service(timeout_sec=2.0):
                await websocket.send_json({
                    "type": "service_response",
                    "request_id": request_id,
                    "result": {
                        "success": False,
                        "message": "Firmware update service not available",
                        "bytes_written": 0,
                    }
                })
                return
            
            # Build request
            request = ros_srvs.FirmwareUpdate.Request()
            request.node_id = int(data.get("node_id", 0))
            request.firmware_path = str(data.get("firmware_path", ""))
            request.skip_reset = bool(data.get("skip_reset", False))
            request.skip_boot = bool(data.get("skip_boot", False))
            
            # Call service
            future = client.call_async(request)
            
            # Wait for response
            while not future.done():
                await asyncio.sleep(0.1)
            
            result = future.result()
            if result is None:
                await websocket.send_json({
                    "type": "service_response",
                    "request_id": request_id,
                    "result": {
                        "success": False,
                        "message": "Service returned None",
                        "bytes_written": 0,
                    }
                })
                return
            
            await websocket.send_json({
                "type": "service_response",
                "request_id": request_id,
                "result": {
                    "success": result.success,
                    "message": result.message,
                    "bytes_written": result.bytes_written,
                }
            })
        except Exception as exc:
            self.get_logger().error(f"Firmware update call error: {exc}")
            await websocket.send_json({
                "type": "service_response",
                "request_id": request_id,
                "result": {
                    "success": False,
                    "message": f"Error: {exc}",
                    "bytes_written": 0,
                }
            })

    async def _get_available_firmware(self) -> list:
        """Query gateway for available firmware via service call."""
        try:
            if not self._firmware_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning("Firmware service not available")
                return []
            
            request = ros_srvs.GetAvailableFirmware.Request()
            future = self._firmware_client.call_async(request)
            
            # Wait for response with timeout
            while not future.done():
                await asyncio.sleep(0.1)
            
            if future.result() is None:
                self.get_logger().warning("Firmware service returned None")
                return []
            
            return [
                {
                    "node_id": int(fw.node_id),
                    "node_name": fw.node_name,
                    "pcb_rev": fw.pcb_rev,
                    "artifact": fw.artifact,
                    "sha256": fw.sha256,
                }
                for fw in future.result().firmware
            ]
        except Exception as exc:
            self.get_logger().warning(f"Failed to get available firmware: {exc}")
            return []

    def _resolve_tx_publisher(self, name: str, dest_id: int) -> Tuple[str, object]:
        if dest_id == BROADCAST_ID:
            return f"r2bus/{name}/tx", self._tx_publishers[name]
        node_name = self._node_names.get(dest_id)
        if node_name is None:
            raise ValueError("Unknown dest_id (add to nodes.json or use 0xFF)")
        key = (dest_id, name)
        publisher = self._node_tx_publishers.get(key)
        if publisher is None:
            topic = f"r2bus/{node_name}/{name}/tx"
            publisher = self.create_publisher(self._message_map[name].ros_cls, topic, 10)
            self._node_tx_publishers[key] = publisher
        else:
            topic = f"r2bus/{node_name}/{name}/tx"
        return topic, publisher

    async def broadcast_loop(self) -> None:
        if self._queue is None:
            raise RuntimeError("Outbound queue not initialized")
        while True:
            outbound = await self._queue.get()
            if outbound.target is not None:
                await self._send_to_client(outbound.target, outbound.payload)
            else:
                await self._send_to_all(outbound.payload)

    async def _send_to_client(self, websocket: WebSocket, payload: dict) -> None:
        try:
            await websocket.send_json(payload)
        except Exception:
            self._ws_clients.discard(websocket)

    async def _send_to_all(self, payload: dict) -> None:
        if not self._ws_clients:
            return
        stale = []
        for ws in self._ws_clients:
            try:
                await ws.send_json(payload)
            except Exception:
                stale.append(ws)
        for ws in stale:
            self._ws_clients.discard(ws)

    async def run_http(self) -> None:
        host = self.get_parameter("web_host").get_parameter_value().string_value
        port = self.get_parameter("web_port").get_parameter_value().integer_value
        config = uvicorn.Config(self.app, host=host, port=port, log_level="info")
        server = uvicorn.Server(config)
        await server.serve()

    def set_queue(self, queue: asyncio.Queue[OutboundMessage]) -> None:
        self._queue = queue


def main(argv: Optional[list[str]] = None) -> int:
    rclpy.init(args=argv)
    loop = asyncio.get_event_loop()

    try:
        node = WebBridgeNode(loop)
    except Exception as exc:
        print(f"Failed to start web bridge: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    async def main_async() -> None:
        node.set_queue(asyncio.Queue())
        http_task = asyncio.create_task(node.run_http())
        broadcast_task = asyncio.create_task(node.broadcast_loop())
        try:
            while not http_task.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0.01)
        finally:
            http_task.cancel()
            broadcast_task.cancel()

    try:
        loop.run_until_complete(main_async())
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

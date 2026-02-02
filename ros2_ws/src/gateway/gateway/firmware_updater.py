#!/usr/bin/env python3
"""Firmware updater integration for the R2 bus gateway.

This module provides firmware update capability using the bootloader protocol
from flash_r2bus.py, allowing updates without disabling the gateway node.
"""

from __future__ import annotations

import csv
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional

try:
    import serial  # type: ignore
except ImportError:
    serial = None

try:
    from google.protobuf.message import DecodeError  # type: ignore
except ImportError:
    DecodeError = None


SYNC0 = 0x55
SYNC1 = 0xAA
PROTOCOL_VERSION = 1

CMD_PING = 0x01
CMD_INFO = 0x02
CMD_START = 0x10
CMD_DATA = 0x11
CMD_DONE = 0x12
CMD_BOOT = 0x13
CMD_ABORT = 0x14
CMD_ACK = 0x80

BOOT_STATUS_OK = 0
BOOT_STATUS_BAD_STATE = 1
BOOT_STATUS_INVALID_ARG = 2
BOOT_STATUS_TOO_LARGE = 3
BOOT_STATUS_ALIGNMENT = 4
BOOT_STATUS_CRC_MISMATCH = 5
BOOT_STATUS_INTERNAL_ERROR = 6
BOOT_STATUS_NO_APP = 7
BOOT_STATUS_INVALID_CMD = 8
BOOT_STATUS_RETRY = 9
BOOT_STATUS_COLLISION = 10

MAX_PAYLOAD = 1024
DEFAULT_HOST_ID = 0x00

STATUS_TEXT = {
    BOOT_STATUS_OK: "ok",
    BOOT_STATUS_BAD_STATE: "bad-state",
    BOOT_STATUS_INVALID_ARG: "invalid-arg",
    BOOT_STATUS_TOO_LARGE: "too-large",
    BOOT_STATUS_ALIGNMENT: "alignment",
    BOOT_STATUS_CRC_MISMATCH: "crc-mismatch",
    BOOT_STATUS_INTERNAL_ERROR: "internal-error",
    BOOT_STATUS_NO_APP: "no-app",
    BOOT_STATUS_INVALID_CMD: "invalid-cmd",
    BOOT_STATUS_RETRY: "retry",
    BOOT_STATUS_COLLISION: "collision",
}

HEADER_STRUCT = struct.Struct("<BBBBII")
INFO_STRUCT = struct.Struct("<IIIIII8s")

ECU_RESET_ATTEMPTS = 3

R2BUS_CRC_SEED = 0xFFFF
R2BUS_CRC_POLY = 0x1021


class BootloaderError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        status: int | None = None,
        acked_cmd: int | None = None,
        ack_payload: bytes | None = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.acked_cmd = acked_cmd
        self.ack_payload = ack_payload or b""


class ProtocolError(RuntimeError):
    pass


def _r2bus_crc16(data: bytes) -> int:
    crc = R2BUS_CRC_SEED
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ R2BUS_CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def _r2bus_build_frame(dest: int, src: int, msg_id: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("R2 bus payload exceeds 255 bytes")
    header = bytes([(dest & 0xFF), (src & 0xFF), (msg_id & 0xFF), len(payload) & 0xFF])
    crc = _r2bus_crc16(header + payload)
    return bytes([SYNC0, SYNC1]) + header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _r2bus_try_parse(buffer: bytearray):
    while True:
        if len(buffer) < 2:
            return None
        if buffer[0] != SYNC0:
            del buffer[0]
            continue
        if buffer[1] != SYNC1:
            del buffer[0]
            continue
        if len(buffer) < 6:
            return None
        length = buffer[5]
        total = 2 + 4 + length + 2
        if len(buffer) < total:
            return None
        segment = buffer[2 : 6 + length]
        payload = bytes(buffer[6 : 6 + length])
        crc_rx = buffer[6 + length] | (buffer[7 + length] << 8)
        crc_calc = _r2bus_crc16(segment)
        dest = buffer[2]
        src = buffer[3]
        msg_id = buffer[4]
        del buffer[:total]
        if crc_rx != crc_calc:
            continue
        return dest, src, msg_id, payload


@dataclass
class FirmwareEntry:
    """Metadata for a single firmware image."""
    node_name: str
    node_id: int
    pcb_rev: str | None
    artifact: Path
    sha256: str

    @property
    def label(self) -> str:
        name = self.node_name or f"node_0x{self.node_id:02X}"
        return f"{name} (0x{self.node_id:02X})"


@dataclass
class UpdateResult:
    success: bool
    message: str
    bytes_written: int = 0


@dataclass
class ProgressUpdate:
    """Progress update during firmware update."""
    step: str
    bytes_processed: int = 0
    bytes_total: int = 0
    message: str = ""


class FirmwarePackage:
    """Load and manage firmware images from a deployment directory with metadata.csv."""

    def __init__(self, root: Path, entries: List[FirmwareEntry]) -> None:
        self.root = root
        self.entries = entries

    @classmethod
    def from_dir(cls, bundle_dir: Path) -> "FirmwarePackage":
        """Load firmware package from directory with metadata.csv."""
        bundle_dir = bundle_dir.resolve()
        metadata_path = bundle_dir / "metadata.csv"
        if not metadata_path.exists():
            raise FileNotFoundError(f"metadata.csv not found in {bundle_dir}")

        entries: List[FirmwareEntry] = []
        with metadata_path.open(newline="") as fh:
            reader = csv.DictReader(fh)
            if reader.fieldnames is None:
                raise ValueError("metadata.csv has no header row")
            for idx, row in enumerate(reader, start=1):
                try:
                    raw_id = row.get("node_id") or row.get("node_id_hex") or row.get("id") or row.get("node")
                    node_id = int(raw_id.strip().rstrip("u"), 0) if raw_id else None
                    if node_id is None:
                        raise ValueError("missing node_id")
                except Exception as exc:
                    raise ValueError(f"Row {idx}: unable to parse node id ({exc})") from exc

                artifact_name = (row.get("artifact") or row.get("filename") or "").strip()
                if not artifact_name:
                    raise ValueError(f"Row {idx}: missing artifact path")
                artifact = (bundle_dir / artifact_name).resolve()
                node_name = (row.get("node_name") or "").strip()
                pcb_rev = (row.get("pcb_rev") or "").strip() or None
                sha256 = (row.get("sha256") or "").strip()
                entries.append(
                    FirmwareEntry(
                        node_name=node_name,
                        node_id=node_id & 0xFF,
                        pcb_rev=pcb_rev,
                        artifact=artifact,
                        sha256=sha256.lower() if sha256 else "",
                    )
                )

        if not entries:
            raise ValueError("No firmware entries found in metadata.csv")
        return cls(bundle_dir, entries)

    def find_by_name(self, names: Iterable[str]) -> List[FirmwareEntry]:
        """Find firmware entries by node name."""
        wanted = {name.strip().lower() for name in names}
        return [entry for entry in self.entries if entry.node_name.lower() in wanted]

    def find_by_id(self, ids: Iterable[int]) -> List[FirmwareEntry]:
        """Find firmware entries by node ID."""
        wanted = {node_id & 0xFF for node_id in ids}
        return [entry for entry in self.entries if (entry.node_id & 0xFF) in wanted]

    def get_for_node(self, node_id: int) -> FirmwareEntry | None:
        """Get firmware entry for a specific node ID."""
        matches = self.find_by_id([node_id])
        return matches[0] if matches else None


class FirmwareUpdater:
    """Perform firmware updates over an existing serial connection."""

    def __init__(
        self,
        ser: "serial.Serial",
        host_id: int = DEFAULT_HOST_ID,
        chunk_size: int = 512,
        timeout: float = 1.5,
        command_retries: int = 2,
        ping_retries: int = 80,
        ping_delay: float = 0.25,
        log_callback: Optional[Callable[[str], None]] = None,
        progress_callback: Optional[Callable[[ProgressUpdate], None]] = None,
    ) -> None:
        self._ser = ser
        self._host_id = host_id & 0xFF
        self._chunk_size = min(chunk_size, MAX_PAYLOAD)
        self._timeout = timeout
        self._command_retries = max(0, command_retries)
        self._ping_retries = ping_retries
        self._ping_delay = ping_delay
        self._log = log_callback or (lambda msg: None)
        self._progress = progress_callback or (lambda _: None)
        self._node_id = 0
        self._last_sender_id: int | None = None

    def _frame_packet(self, cmd: int, payload: bytes) -> bytes:
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        header = HEADER_STRUCT.pack(
            self._node_id,
            self._host_id,
            cmd,
            PROTOCOL_VERSION,
            len(payload),
            crc,
        )
        return bytes((SYNC0, SYNC1)) + header + payload

    def _send_packet(self, cmd: int, payload: bytes = b"") -> None:
        frame = self._frame_packet(cmd, payload)
        self._ser.write(frame)
        self._ser.flush()

    def _read_exact(self, length: int, deadline: float) -> bytes | None:
        data = bytearray()
        while len(data) < length:
            if time.monotonic() >= deadline:
                return None
            chunk = self._ser.read(length - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def _read_packet(self, timeout: float):
        deadline = time.monotonic() + timeout
        got_sync0 = False
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self._ser.in_waiting == 0:
                time.sleep(min(0.001, remaining))
                continue
            byte = self._ser.read(1)
            if not byte:
                continue
            val = byte[0]
            if not got_sync0:
                got_sync0 = val == SYNC0
                continue
            if val != SYNC1:
                got_sync0 = val == SYNC0
                continue
            header = self._read_exact(HEADER_STRUCT.size, deadline)
            if header is None:
                return None
            dest, src, cmd, version, length, crc = HEADER_STRUCT.unpack(header)
            if length > MAX_PAYLOAD:
                raise ProtocolError(f"Payload too large: {length}")
            payload = self._read_exact(length, deadline)
            if payload is None:
                return None
            calc_crc = zlib.crc32(payload) & 0xFFFFFFFF
            if calc_crc != crc:
                raise ProtocolError(f"CRC mismatch for cmd 0x{cmd:02X}")
            return dest, src, cmd, version, payload
        return None

    def _expect_ack(self, expected_cmd: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProtocolError("Timed out waiting for ACK")
            packet = self._read_packet(remaining)
            if packet is None:
                continue
            dest, src, cmd, version, payload = packet
            if dest not in (self._host_id, 0xFF):
                continue
            if self._node_id != 0xFF and src != self._node_id:
                continue
            if version != PROTOCOL_VERSION:
                raise ProtocolError(f"Protocol version mismatch: {version}")
            if cmd != CMD_ACK:
                raise ProtocolError(f"Unexpected packet 0x{cmd:02X}")
            if len(payload) < 2:
                raise ProtocolError("ACK payload too short")
            acked = payload[0]
            status = payload[1]
            ack_payload = payload[2:]
            if acked != expected_cmd:
                raise ProtocolError(f"ACK for unexpected command 0x{acked:02X}")
            if status != BOOT_STATUS_OK:
                text = STATUS_TEXT.get(status, f"err-{status}")
                raise BootloaderError(
                    f"{text}",
                    status=status,
                    acked_cmd=acked,
                    ack_payload=ack_payload,
                )
            self._last_sender_id = src
            return ack_payload

    def _command_with_retry(
        self,
        cmd: int,
        payload: bytes,
        timeout: float,
        retries: int | None = None,
    ) -> bytes:
        attempts = (self._command_retries if retries is None else max(0, retries)) + 1
        last_error: ProtocolError | BootloaderError | None = None
        for attempt in range(attempts):
            if attempt:
                time.sleep(0.05)
            self._send_packet(cmd, payload)
            try:
                return self._expect_ack(cmd, timeout)
            except (BootloaderError, ProtocolError) as exc:
                last_error = exc
                continue
        assert last_error is not None
        raise last_error

    def _wait_for_bootloader(self) -> bool:
        original_node = self._node_id
        for attempt in range(self._ping_retries):
            try:
                self._command_with_retry(CMD_PING, b"", timeout=max(self._timeout, 1.0), retries=0)
                return True
            except (ProtocolError, BootloaderError):
                time.sleep(self._ping_delay)

        self._node_id = 0xFF
        for attempt in range(self._ping_retries):
            try:
                self._command_with_retry(CMD_PING, b"", timeout=max(self._timeout, 1.0), retries=0)
                if self._last_sender_id is not None:
                    self._node_id = self._last_sender_id
                else:
                    self._node_id = original_node
                return True
            except (ProtocolError, BootloaderError):
                time.sleep(self._ping_delay)

        self._node_id = original_node
        return False

    def _send_ecu_reset(self, r2bus_pb2) -> bool:
        msg_id = r2bus_pb2.MessageId.MESSAGE_ID_ECU_RESET
        frame = _r2bus_build_frame(self._node_id, self._host_id, msg_id, b"")
        self._ser.write(frame)
        self._ser.flush()

        buffer = bytearray()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            parsed = _r2bus_try_parse(buffer)
            if parsed:
                dest, src, recv_msg_id, payload = parsed
                if recv_msg_id == r2bus_pb2.MessageId.MESSAGE_ID_ACK and src == self._node_id:
                    try:
                        ack = r2bus_pb2.Ack()
                        ack.ParseFromString(payload)
                        if ack.original_msg == msg_id and ack.status == r2bus_pb2.STATUS_OK:
                            return True
                    except Exception:
                        pass
            chunk = self._ser.read(max(1, self._ser.in_waiting))
            if chunk:
                buffer.extend(chunk)
            else:
                time.sleep(0.01)
        return False

    def update(
        self,
        node_id: int,
        firmware_path: Path,
        skip_reset: bool = False,
        skip_boot: bool = False,
        r2bus_pb2=None,
    ) -> UpdateResult:
        self._node_id = node_id & 0xFF

        if not firmware_path.exists():
            return UpdateResult(False, f"Firmware file not found: {firmware_path}")

        try:
            image = firmware_path.read_bytes()
        except OSError as exc:
            return UpdateResult(False, f"Failed to read firmware: {exc}")

        if len(image) == 0:
            return UpdateResult(False, "Firmware file is empty")

        crc = zlib.crc32(image) & 0xFFFFFFFF
        self._log(f"Loaded {len(image)} bytes, CRC32=0x{crc:08X}")
        self._progress(ProgressUpdate(
            step="load",
            bytes_processed=len(image),
            bytes_total=len(image),
            message=f"Loaded {len(image)} bytes"
        ))

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        if not skip_reset and r2bus_pb2 is not None:
            self._log(f"Sending ECU reset to 0x{self._node_id:02X}...")
            self._progress(ProgressUpdate(step="reset", message="Sending ECU reset"))
            for attempt in range(ECU_RESET_ATTEMPTS):
                if self._send_ecu_reset(r2bus_pb2):
                    self._log("ECU reset acknowledged")
                    self._progress(ProgressUpdate(step="reset", message="ECU reset acknowledged"))
                    break
                if attempt < ECU_RESET_ATTEMPTS - 1:
                    self._log("No reset ACK, retrying...")
            time.sleep(1.0)

        self._log("Waiting for bootloader...")
        self._progress(ProgressUpdate(step="bootloader", message="Waiting for bootloader"))
        if not self._wait_for_bootloader():
            return UpdateResult(False, "Bootloader did not respond")

        self._log("Bootloader connected, getting info...")
        self._progress(ProgressUpdate(step="info", message="Getting bootloader info"))
        try:
            info_payload = self._command_with_retry(CMD_INFO, b"", timeout=2.0)
            if len(info_payload) != INFO_STRUCT.size:
                return UpdateResult(False, "Unexpected info payload size")
            info = INFO_STRUCT.unpack(info_payload)
            app_max_size = info[2]
            self._log(f"App max size: {app_max_size}, image size: {len(image)}")
            self._progress(ProgressUpdate(step="info", message=f"App max size: {app_max_size}"))

            if len(image) > app_max_size:
                return UpdateResult(False, f"Image too large ({len(image)} > {app_max_size})")

        except (ProtocolError, BootloaderError) as exc:
            return UpdateResult(False, f"Failed to get bootloader info: {exc}")

        sectors = max(1, (len(image) + 4095) // 4096)
        erase_timeout = max(3.0, sectors * 0.12 + 2.0)
        self._log(f"Erasing flash (~{sectors} sectors)...")
        self._progress(ProgressUpdate(step="erase", message=f"Erasing flash (~{sectors} sectors)"))
        try:
            self._command_with_retry(
                CMD_START,
                struct.pack("<II", len(image), crc),
                timeout=erase_timeout,
            )
            self._progress(ProgressUpdate(step="erase", message="Erase complete"))
        except (ProtocolError, BootloaderError) as exc:
            return UpdateResult(False, f"Failed to start session: {exc}")

        self._log("Programming...")
        self._progress(ProgressUpdate(step="program", message="Starting programming"))
        offset = 0
        try:
            while offset < len(image):
                chunk = image[offset : offset + self._chunk_size]
                payload = struct.pack("<I", offset) + chunk
                ack_payload = self._command_with_retry(CMD_DATA, payload, timeout=self._timeout + 5)
                if len(ack_payload) < 4:
                    return UpdateResult(False, "ACK missing progress counter")
                (total_written,) = struct.unpack("<I", ack_payload[:4])
                offset = total_written
                percent = int((offset / len(image)) * 100.0)
                self._log(f"Progress: {percent}% ({offset}/{len(image)})")
                self._progress(ProgressUpdate(
                    step="program",
                    bytes_processed=offset,
                    bytes_total=len(image),
                    message=f"Programming {percent}%"
                ))
        except (ProtocolError, BootloaderError) as exc:
            try:
                self._send_packet(CMD_ABORT)
            except Exception:
                pass
            return UpdateResult(False, f"Programming failed: {exc}")

        self._log("Finishing...")
        self._progress(ProgressUpdate(step="finish", message="Finishing firmware update"))
        try:
            done_payload = self._command_with_retry(CMD_DONE, b"", timeout=3.0)
            if len(done_payload) == 8:
                done_size, done_crc = struct.unpack("<II", done_payload)
                self._log(f"Done: size={done_size}, crc=0x{done_crc:08X}")
                self._progress(ProgressUpdate(step="finish", message=f"Done: {done_size} bytes verified"))
        except (ProtocolError, BootloaderError) as exc:
            return UpdateResult(False, f"Finish failed: {exc}")

        if not skip_boot:
            self._log("Booting new firmware...")
            self._progress(ProgressUpdate(step="boot", message="Booting new firmware"))
            try:
                self._command_with_retry(CMD_BOOT, b"", timeout=2.0)
                self._progress(ProgressUpdate(step="boot", message="Boot complete"))
            except (ProtocolError, BootloaderError) as exc:
                return UpdateResult(False, f"Boot command failed: {exc}")

        self._progress(ProgressUpdate(step="complete", message="Firmware update successful"))
        return UpdateResult(True, "Firmware update successful", len(image))

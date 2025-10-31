#!/usr/bin/env python3
"""
Simple host-side helper for interacting with the R2 bus over a USB/RS485
adapter. Supports continuous logging and sending addressed packets (including
the ECU reset command) to any node on the bus.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

try:
    import serial  # type: ignore
except ImportError:
    sys.exit("PySerial is required: pip install pyserial")

SYNC0 = 0x55
SYNC1 = 0xAA
CRC_SEED = 0xFFFF
CRC_POLY = 0x1021

HOST_ID = 0x00
BROADCAST_ID = 0xFF

MSG_ECU_RESET = 0x01
MSG_PING = 0x02
MSG_PONG = 0x03
MSG_HEARTBEAT = 0x04
MSG_ACK = 0x7F

MSG_NAMES = {
    MSG_ECU_RESET: "ECU_RESET",
    MSG_PING: "PING",
    MSG_PONG: "PONG",
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_ACK: "ACK",
}


def msg_name(msg_id: int) -> str:
    return MSG_NAMES.get(msg_id, f"0x{msg_id:02X}")


def crc16_ccitt(data: Iterable[int], seed: int = CRC_SEED) -> int:
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


@dataclass
class Frame:
    dest: int
    src: int
    msg_id: int
    length: int
    payload: bytes
    crc: int


class R2BusSerial:
    def __init__(self, port: str, baud: int, timeout: float) -> None:
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=timeout,
            write_timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._buffer = bytearray()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

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
            crc_calc = crc16_ccitt(buf[2 : 6 + length])
            del buf[:frame_len]
            if crc_rx != crc_calc:
                continue
            return Frame(dest, src, msg_id, length, payload, crc_rx)

    def read_frame(self, timeout: Optional[float]) -> Optional[Frame]:
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            frame = self._extract_frame()
            if frame:
                return frame
            chunk = self.ser.read(128)
            if chunk:
                self._buffer.extend(chunk)
                continue
            if deadline is not None and time.monotonic() >= deadline:
                return None

    def send_frame(
        self,
        dest: int,
        msg_id: int,
        payload: bytes = b"",
        *,
        src: int = HOST_ID,
    ) -> None:
        if len(payload) > 255:
            raise ValueError("Payload too large (max 255 bytes)")
        header = bytes([dest, src, msg_id, len(payload)])
        crc = crc16_ccitt(header + payload)
        frame = bytes([SYNC0, SYNC1]) + header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self.ser.write(frame)
        self.ser.flush()


def format_payload(data: bytes) -> str:
    if not data:
        return "-"
    hex_part = " ".join(f"{b:02X}" for b in data)
    printable = "".join(chr(b) if 0x20 <= b <= 0x7E else "." for b in data)
    return f"{hex_part} | {printable}"


def format_frame(frame: Frame) -> str:
    timestamp = time.strftime("%H:%M:%S", time.localtime()) + f".{int((time.time() % 1)*1000):03d}"
    name = msg_name(frame.msg_id)
    return (
        f"{timestamp} "
        f"{frame.src:02X} -> {frame.dest:02X} "
        f"{name} len={frame.length} payload={format_payload(frame.payload)}"
    )


def log_command(args: argparse.Namespace) -> int:
    bus = R2BusSerial(args.port, args.baud, args.timeout)
    start = time.monotonic()
    try:
        print(f"Listening on {args.port} @ {args.baud} baud. Press Ctrl-C to stop.")
        while True:
            frame = bus.read_frame(timeout=args.poll_interval)
            if frame:
                if args.filter_dest is not None and frame.dest != args.filter_dest:
                    continue
                if args.filter_src is not None and frame.src != args.filter_src:
                    continue
                print(format_frame(frame))
            if args.duration is not None and (time.monotonic() - start) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
    return 0


def parse_payload(args: argparse.Namespace) -> bytes:
    if args.payload_hex:
        cleaned = args.payload_hex.replace(" ", "").replace("-", "")
        if len(cleaned) % 2:
            raise ValueError("Hex payload must have an even number of digits")
        return bytes.fromhex(cleaned)
    if args.text is not None:
        return args.text.encode("utf-8")
    return bytes(args.data or [])


def wait_for_ack(bus: R2BusSerial, dest: int, msg_id: int, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        frame = bus.read_frame(timeout=remaining)
        if not frame:
            break
        print(format_frame(frame))
        if frame.msg_id == MSG_ACK and frame.src == dest and frame.length >= 2:
            status = frame.payload[0]
            original = frame.payload[1]
            if original == msg_id:
                return status == 0
    return False


def wait_for_message(
    bus: R2BusSerial,
    expected_src: int,
    expected_msg: int,
    timeout: float,
    *,
    expected_dest: Optional[int] = None,
) -> Optional[Frame]:
    end = time.monotonic() + timeout
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        frame = bus.read_frame(timeout=remaining)
        if not frame:
            break
        print(format_frame(frame))
        if frame.msg_id == expected_msg and frame.src == expected_src:
            if expected_dest is None or frame.dest == expected_dest:
                return frame
    return None


def send_command(args: argparse.Namespace) -> int:
    try:
        payload = parse_payload(args)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    bus = R2BusSerial(args.port, args.baud, args.timeout)
    try:
        bus.send_frame(args.dest, args.msg, payload, src=args.src)
        print(
            f"Sent {msg_name(args.msg)} to {args.dest:02X} "
            f"(len={len(payload)} payload={format_payload(payload)})"
        )
        if args.wait_ack and args.dest != BROADCAST_ID:
            ok = wait_for_ack(bus, args.dest, args.msg, args.ack_timeout)
            if ok:
                print("ACK received")
                return 0
            print("No ACK received", file=sys.stderr)
            return 1
    finally:
        bus.close()
    return 0


def reset_command(args: argparse.Namespace) -> int:
    args.msg = MSG_ECU_RESET
    args.payload_hex = None
    args.text = None
    args.data = []
    wait_ack = not args.no_wait_ack

    bus = R2BusSerial(args.port, args.baud, args.timeout)
    try:
        bus.send_frame(args.dest, args.msg, b"", src=args.src)
        print(f"Sent ECU_RESET to {args.dest:02X}")
        if wait_ack and args.dest != BROADCAST_ID:
            ok = wait_for_ack(bus, args.dest, args.msg, args.ack_timeout)
            if ok:
                print("ACK received")
                return 0
            print("No ACK received", file=sys.stderr)
            return 1
    finally:
        bus.close()
    return 0


def ping_command(args: argparse.Namespace) -> int:
    args.msg = MSG_PING
    if args.payload_hex is None and args.text is None and not args.data:
        args.text = "ping"
    try:
        payload = parse_payload(args)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    bus = R2BusSerial(args.port, args.baud, args.timeout)
    try:
        bus.send_frame(args.dest, args.msg, payload, src=args.src)
        print(
            f"Sent PING to {args.dest:02X} "
            f"(len={len(payload)} payload={format_payload(payload)})"
        )
        if args.wait_ack and args.dest != BROADCAST_ID:
            ok = wait_for_ack(bus, args.dest, args.msg, args.ack_timeout)
            if not ok:
                print("No ACK received", file=sys.stderr)
        response = wait_for_message(
            bus,
            expected_src=args.dest,
            expected_msg=MSG_PONG,
            timeout=args.pong_timeout,
            expected_dest=args.src,
        )
        if response:
            print("PONG received")
            return 0
        print("No PONG response", file=sys.stderr)
        return 1
    finally:
        bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="R2 bus serial helper")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: %(default)s)")
    parser.add_argument("--baud", default=115200, type=int, help="Baud rate (default: %(default)s)")
    parser.add_argument("--timeout", default=0.1, type=float, help="Serial IO timeout in seconds")
    parser.add_argument("--src", default=HOST_ID, type=lambda x: int(x, 0), help="Host ID to use")

    sub = parser.add_subparsers(dest="command", required=True)

    log_p = sub.add_parser("log", help="Continuously log frames on the bus")
    log_p.add_argument("--duration", type=float, help="Stop after this many seconds")
    log_p.add_argument("--poll-interval", default=0.1, type=float, help="Read timeout while polling")
    log_p.add_argument("--filter-dest", type=lambda x: int(x, 0), help="Only show frames sent to this ID")
    log_p.add_argument("--filter-src", type=lambda x: int(x, 0), help="Only show frames from this ID")
    log_p.set_defaults(func=log_command)

    send_p = sub.add_parser("send", help="Send an arbitrary frame")
    send_p.add_argument("dest", type=lambda x: int(x, 0), help="Destination node ID (e.g. 0x10)")
    send_p.add_argument("msg", type=lambda x: int(x, 0), help="Message ID (number or 0xXX)")
    send_p.add_argument("--payload-hex", help="Payload as hex bytes (e.g. '01 02 0A')")
    send_p.add_argument("--text", help="Payload as UTF-8 text")
    send_p.add_argument(
        "--data",
        type=lambda x: int(x, 0),
        nargs="*",
        help="Payload bytes as integers (e.g. --data 1 2 10)",
    )
    send_p.add_argument("--wait-ack", action="store_true", help="Wait for ACK from destination")
    send_p.add_argument(
        "--ack-timeout",
        default=1.0,
        type=float,
        help="Seconds to wait for ACK when --wait-ack is set",
    )
    send_p.set_defaults(func=send_command)

    reset_p = sub.add_parser("reset", help="Send the ECU reset command and optionally wait for ACK")
    reset_p.add_argument("dest", type=lambda x: int(x, 0), help="Destination node ID (e.g. 0x10)")
    reset_p.add_argument(
        "--no-wait-ack",
        action="store_true",
        help="Send reset without waiting for an ACK",
    )
    reset_p.add_argument(
        "--ack-timeout",
        default=1.5,
        type=float,
        help="Seconds to wait for ACK response",
    )
    reset_p.set_defaults(func=reset_command)

    ping_p = sub.add_parser("ping", help="Send a ping to a node and wait for response")
    ping_p.add_argument("dest", type=lambda x: int(x, 0), help="Destination node ID")
    ping_p.add_argument("--wait-ack", action="store_true", help="Also wait for ACK from destination")
    ping_p.add_argument(
        "--ack-timeout",
        default=1.0,
        type=float,
        help="Seconds to wait for ACK when --wait-ack is set",
    )
    ping_p.add_argument(
        "--pong-timeout",
        default=1.0,
        type=float,
        help="Seconds to wait for a PONG response",
    )
    ping_p.add_argument("--payload-hex", help="Optional payload as hex")
    ping_p.add_argument("--text", help="Optional payload text (defaults to 'ping')")
    ping_p.add_argument(
        "--data",
        type=lambda x: int(x, 0),
        nargs="*",
        help="Optional payload as integer bytes",
    )
    ping_p.set_defaults(func=ping_command)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

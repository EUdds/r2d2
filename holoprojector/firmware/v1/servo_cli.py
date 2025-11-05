#!/usr/bin/env python3
"""
Convenience CLI for issuing servo move commands over the R2 bus.

This wraps the lower-level helpers in ``r2bus_tool.py`` to provide a focused
interface for debugging servo behaviour from the host.
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

import r2bus_tool


def _parse_node_id(value: str) -> int:
    node_id = int(value, 0)
    if not 0 <= node_id <= 0xFF:
        raise argparse.ArgumentTypeError("Node ID must be between 0 and 0xFF")
    return node_id


def _parse_servo(value: str) -> int:
    servo = int(value, 0)
    if servo < 0:
        raise argparse.ArgumentTypeError("Servo index must be non-negative")
    if servo > 0xFF:
        raise argparse.ArgumentTypeError("Servo index must fit in one byte")
    return servo


def _parse_u32(value: str) -> int:
    num = int(value, 0)
    if num < 0:
        raise argparse.ArgumentTypeError("Value must be non-negative")
    if num > 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("Value exceeds uint32 range")
    return num


def _build_move_command(args: argparse.Namespace) -> r2bus_tool.r2bus_pb2.ServoMoveCommand:
    command = r2bus_tool.r2bus_pb2.ServoMoveCommand()
    command.servo = args.servo
    command.position_deg = args.position
    command.duration_ms = args.duration_ms
    return command


def move_command(args: argparse.Namespace) -> int:
    command = _build_move_command(args)

    bus = r2bus_tool.R2BusSerial(args.port, args.baud, args.timeout)
    try:
        payload = command.SerializeToString()
        bus.send_frame(
            args.dest,
            r2bus_tool.MSG_SERVO_MOVE_CMD,
            payload,
            src=args.src,
        )
        print(
            "Sent servo_move to 0x{dest:02X}: servo={servo} pos={pos:.2f} pulse_override_us={pulse}".format(
                dest=args.dest,
                servo=command.servo,
                pos=command.position_deg,
                pulse=command.duration_ms if command.duration_ms else "none",
            )
        )
        if args.wait_ack and args.dest != r2bus_tool.BROADCAST_ID:
            ok = r2bus_tool.wait_for_ack(
                bus,
                dest=args.dest,
                msg_id=r2bus_tool.MSG_SERVO_MOVE_CMD,
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


def pulse_command(args: argparse.Namespace) -> int:
    if args.pulse_us == 0:
        print("Error: pulse width must be greater than zero", file=sys.stderr)
        return 2

    command = r2bus_tool.r2bus_pb2.ServoMoveCommand()
    command.servo = args.servo
    command.position_deg = args.fallback_angle
    command.duration_ms = args.pulse_us

    bus = r2bus_tool.R2BusSerial(args.port, args.baud, args.timeout)
    try:
        payload = command.SerializeToString()
        bus.send_frame(
            args.dest,
            r2bus_tool.MSG_SERVO_MOVE_CMD,
            payload,
            src=args.src,
        )
        print(
            "Sent servo_pulse to 0x{dest:02X}: servo={servo} pulse_us={pulse} fallback_deg={deg:.2f}".format(
                dest=args.dest,
                servo=command.servo,
                pulse=command.duration_ms,
                deg=command.position_deg,
            )
        )
        if args.wait_ack and args.dest != r2bus_tool.BROADCAST_ID:
            ok = r2bus_tool.wait_for_ack(
                bus,
                dest=args.dest,
                msg_id=r2bus_tool.MSG_SERVO_MOVE_CMD,
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


def home_command(args: argparse.Namespace) -> int:
    if args.all and args.servo is not None:
        print("Error: specify either --servo or --all, not both", file=sys.stderr)
        return 2
    if not args.all and args.servo is None:
        print("Error: provide --servo when --all is not set", file=sys.stderr)
        return 2

    command = r2bus_tool.r2bus_pb2.ServoHomeCommand()
    if args.all:
        command.all = True
        command.servo = 0
    else:
        command.all = False
        command.servo = args.servo

    bus = r2bus_tool.R2BusSerial(args.port, args.baud, args.timeout)
    try:
        payload = command.SerializeToString()
        bus.send_frame(
            args.dest,
            r2bus_tool.MSG_SERVO_HOME_CMD,
            payload,
            src=args.src,
        )
        target = "all servos" if command.all else f"servo {command.servo}"
        print(f"Sent servo_home to 0x{args.dest:02X}: target={target}")
        if args.wait_ack and args.dest != r2bus_tool.BROADCAST_ID:
            ok = r2bus_tool.wait_for_ack(
                bus,
                dest=args.dest,
                msg_id=r2bus_tool.MSG_SERVO_HOME_CMD,
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
    parser = argparse.ArgumentParser(description="CLI for debugging servo moves over the R2 bus")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port path (default: %(default)s)")
    parser.add_argument("--baud", default=115200, type=int, help="Baud rate (default: %(default)s)")
    parser.add_argument("--timeout", default=0.1, type=float, help="Serial IO timeout in seconds")
    parser.add_argument(
        "--src",
        default=r2bus_tool.HOST_ID,
        type=_parse_node_id,
        help="Host ID to use (default: %(default)#04x)",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    move_p = sub.add_parser("move", help="Send a single servo move command")
    move_p.add_argument("dest", type=_parse_node_id, help="Destination node ID (e.g. 0x10)")
    move_p.add_argument("--servo", required=True, type=_parse_servo, help="Servo channel index")
    move_p.add_argument(
        "--position",
        required=True,
        type=float,
        help="Target servo position in degrees",
    )
    move_p.add_argument(
        "--duration-ms",
        default=0,
        type=_parse_u32,
        help="Optional raw pulse override in microseconds (leave 0 to use angle mapping)",
    )
    move_p.add_argument(
        "--wait-ack",
        action="store_true",
        help="Wait for an ACK response from the destination",
    )
    move_p.add_argument(
        "--ack-timeout",
        default=1.0,
        type=float,
        help="Seconds to wait for ACK when --wait-ack is used",
    )
    move_p.set_defaults(func=move_command)

    pulse_p = sub.add_parser("pulse", help="Send a raw pulse width (microseconds)")
    pulse_p.add_argument("dest", type=_parse_node_id, help="Destination node ID (e.g. 0x10)")
    pulse_p.add_argument("--servo", required=True, type=_parse_servo, help="Servo channel index")
    pulse_p.add_argument(
        "--pulse-us",
        required=True,
        type=_parse_u32,
        help="Target pulse width in microseconds (e.g. 1500)",
    )
    pulse_p.add_argument(
        "--fallback-angle",
        default=0.0,
        type=float,
        help="Angle to report in the command (ignored by firmware when --pulse-us is used)",
    )
    pulse_p.add_argument(
        "--wait-ack",
        action="store_true",
        help="Wait for an ACK response from the destination",
    )
    pulse_p.add_argument(
        "--ack-timeout",
        default=1.0,
        type=float,
        help="Seconds to wait for ACK when --wait-ack is used",
    )
    pulse_p.set_defaults(func=pulse_command)

    home_p = sub.add_parser("home", help="Return one or all servos to their home angles")
    home_p.add_argument("dest", type=_parse_node_id, help="Destination node ID (e.g. 0x10)")
    home_p.add_argument(
        "--servo",
        type=_parse_servo,
        help="Servo channel index to home",
    )
    home_p.add_argument(
        "--all",
        action="store_true",
        help="Home all servos on the destination",
    )
    home_p.add_argument(
        "--wait-ack",
        action="store_true",
        help="Wait for an ACK response from the destination",
    )
    home_p.add_argument(
        "--ack-timeout",
        default=1.0,
        type=float,
        help="Seconds to wait for ACK when --wait-ack is used",
    )
    home_p.set_defaults(func=home_command)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

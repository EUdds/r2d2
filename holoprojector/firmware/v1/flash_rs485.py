#!/usr/bin/env python3
import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

import tqdm

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - dependency hint
    sys.exit("PySerial is required: pip install pyserial")  # pragma: no cover

try:
    from google.protobuf.message import DecodeError
except ImportError as exc:  # pragma: no cover - dependency hint
    sys.exit("Google protobuf is required: pip install protobuf")  # pragma: no cover

PROTO_DIR = Path(__file__).resolve().with_name("proto")
if PROTO_DIR.exists():
    sys.path.insert(0, str(PROTO_DIR))

try:
    import r2bus_pb2
except ImportError as exc:  # pragma: no cover - dependency hint
    sys.exit(f"Unable to import R2 bus protobuf module: {exc}")  # pragma: no cover

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

MAX_PAYLOAD = 1024

STATUS_TEXT = {
    0: "ok",
    1: "bad-state",
    2: "invalid-arg",
    3: "too-large",
    4: "alignment",
    5: "crc-mismatch",
    6: "internal-error",
    7: "no-app",
    8: "invalid-cmd",
}

HEADER_STRUCT = struct.Struct("<BBBBII")  # dest, src, cmd, version, length, crc
INFO_STRUCT = struct.Struct("<IIIIII8s")

MessageId = r2bus_pb2.MessageId

R2BUS_MSG_ECU_RESET = MessageId.MESSAGE_ID_ECU_RESET
R2BUS_MSG_ACK = MessageId.MESSAGE_ID_ACK
R2BUS_HOST_ID = 0x00
R2BUS_BROADCAST_ID = 0xFF
R2BUS_CRC_SEED = 0xFFFF
R2BUS_CRC_POLY = 0x1021


class BootloaderError(RuntimeError):
    """Raised when the bootloader reports an error."""


class ProtocolError(RuntimeError):
    """Raised on framing or CRC errors."""


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


def request_application_reset(
    port: str,
    baud: int,
    node_id: int,
    host_id: int,
    ack_timeout: float,
    wait_for_ack: bool = True,
) -> bool:
    """Send ECU reset command over the R2 bus to reboot the application into the bootloader.

    Returns True if an ACK was received, False otherwise.
    """

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        write_timeout=max(1.0, ack_timeout),
    )
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        frame = _r2bus_build_frame(node_id, host_id, R2BUS_MSG_ECU_RESET, b"")
        ser.write(frame)
        ser.flush()
        if not wait_for_ack:
            return False
        buffer = bytearray()
        deadline = time.monotonic() + max(0.0, ack_timeout)
        while True:
            parsed = _r2bus_try_parse(buffer)
            if parsed:
                dest, src, msg_id, payload = parsed
                if (
                    msg_id == R2BUS_MSG_ACK
                    and src == node_id
                    and dest in (host_id & 0xFF, R2BUS_HOST_ID)
                ):
                    try:
                        ack = r2bus_pb2.Ack()
                        ack.ParseFromString(payload)
                    except DecodeError:
                        continue
                    if ack.original_msg == R2BUS_MSG_ECU_RESET:
                        if ack.status == r2bus_pb2.STATUS_OK:
                            return True
                        status_name = r2bus_pb2.Status.Name(ack.status)
                        raise BootloaderError(f"ECU reset rejected with status {status_name}")
                # Ignore unrelated frames and continue reading
                continue
            if time.monotonic() >= deadline:
                break
            chunk = ser.read(128)
            if chunk:
                buffer.extend(chunk)
            else:
                time.sleep(0.01)
        return False
    finally:
        ser.close()


class BootloaderClient:
    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float,
        node_id: int,
        host_id: int,
        command_retries: int,
    ) -> None:
        if not (0 <= node_id <= 0xFF):
            raise ValueError("node_id must be in range 0x00-0xFF")
        if not (0 <= host_id <= 0xFF):
            raise ValueError("host_id must be in range 0x00-0xFF")
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=timeout,
            write_timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.base_timeout = timeout
        self.node_id = node_id & 0xFF
        self.host_id = host_id & 0xFF
        self.command_retries = max(0, command_retries)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _frame_packet(self, cmd: int, payload: bytes) -> bytes:
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        header = HEADER_STRUCT.pack(
            self.node_id,
            self.host_id,
            cmd,
            PROTOCOL_VERSION,
            len(payload),
            crc,
        )
        return bytes((SYNC0, SYNC1)) + header + payload

    def send_packet(self, cmd: int, payload: bytes = b"") -> None:
        frame = self._frame_packet(cmd, payload)
        self.ser.write(frame)
        self.ser.flush()

    def _read_exact(self, length: int, deadline: float) -> bytes | None:
        data = bytearray()
        while len(data) < length:
            if time.monotonic() >= deadline:
                return None
            chunk = self.ser.read(length - len(data))
            if not chunk:
                continue
            data.extend(chunk)
        return bytes(data)

    def read_packet(self, timeout: float) -> tuple[int, int, int, int, bytes] | None:
        deadline = time.monotonic() + timeout
        got_sync0 = False
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self.ser.in_waiting == 0:
                time.sleep(min(0.001, remaining))
                continue
            byte = self.ser.read(1)
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

    def expect_ack(self, expected_cmd: int, timeout: float) -> bytes:
        return self._expect_ack_once(expected_cmd, timeout)

    def _expect_ack_once(self, expected_cmd: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProtocolError("Timed out waiting for ACK")
            packet = self.read_packet(remaining)
            if packet is None:
                continue
            dest, src, cmd, version, payload = packet
            if dest not in (self.host_id, R2BUS_BROADCAST_ID):
                continue
            if src != self.node_id:
                continue
            if version != PROTOCOL_VERSION:
                raise ProtocolError(f"Protocol version mismatch: {version}")
            if cmd != CMD_ACK:
                raise ProtocolError(f"Unexpected packet 0x{cmd:02X}")
            if len(payload) < 2:
                raise ProtocolError("ACK payload too short")
            acked = payload[0]
            status = payload[1]
            if acked != expected_cmd:
                raise ProtocolError(f"ACK for unexpected command 0x{acked:02X}")
            if status != 0:
                text = STATUS_TEXT.get(status, f"err-{status}")
                extra = payload[2:].decode(errors="ignore")
                raise BootloaderError(f"{text}: {extra}")
            return payload[2:]

    def _command_with_retry(
        self,
        cmd: int,
        payload: bytes,
        timeout: float,
        retries: int | None = None,
    ) -> bytes:
        attempts = (self.command_retries if retries is None else max(0, retries)) + 1
        last_protocol_error: ProtocolError | None = None
        for attempt in range(attempts):
            if attempt:
                time.sleep(0.05)
            self.send_packet(cmd, payload)
            try:
                return self._expect_ack_once(cmd, timeout)
            except BootloaderError:
                raise
            except ProtocolError as exc:
                last_protocol_error = exc
                continue
        assert last_protocol_error is not None
        raise last_protocol_error

    def wait_for_bootloader(self, retries: int, delay: float, handshake_timeout: float) -> None:
        for attempt in range(retries):
            try:
                self._command_with_retry(
                    CMD_PING,
                    b"",
                    timeout=handshake_timeout,
                    retries=self.command_retries,
                )
                return
            except (ProtocolError, BootloaderError):
                time.sleep(delay)
        raise ProtocolError("Bootloader did not respond to ping")

    def get_info(self) -> dict[str, int | bytes]:
        payload = self._command_with_retry(
            CMD_INFO,
            b"",
            timeout=max(2.0, self.base_timeout),
            retries=self.command_retries,
        )
        if len(payload) != INFO_STRUCT.size:
            raise ProtocolError("Unexpected info payload size")
        info = INFO_STRUCT.unpack(payload)
        return {
            "protocol": info[0],
            "app_offset": info[1],
            "app_max_size": info[2],
            "app_present": info[3],
            "app_size": info[4],
            "app_crc": info[5],
            "board_id": info[6],
        }

    def start_session(self, size: int, crc: int, erase_timeout: float) -> None:
        self._command_with_retry(
            CMD_START,
            struct.pack("<II", size, crc),
            timeout=erase_timeout,
            retries=self.command_retries,
        )

    def send_chunk(self, offset: int, data: bytes, timeout: float) -> int:
        payload = struct.pack("<I", offset) + data
        self.send_packet(CMD_DATA, payload)
        # print("Sent chunk offset", offset)
        ack_payload = self._expect_ack_once(CMD_DATA, timeout=(timeout + 5))
        if len(ack_payload) < 4:
            raise ProtocolError("ACK missing progress counter")
        (total_written,) = struct.unpack("<I", ack_payload[:4])
        return total_written

    def finish(self) -> tuple[int, int]:
        payload = self._command_with_retry(
            CMD_DONE,
            b"",
            timeout=max(3.0, self.base_timeout),
            retries=self.command_retries,
        )
        if len(payload) != 8:
            raise ProtocolError("Unexpected DONE payload size")
        return struct.unpack("<II", payload)

    def boot(self) -> None:
        try:
            self._command_with_retry(
                CMD_BOOT,
                b"",
                timeout=max(2.0, self.base_timeout),
                retries=self.command_retries,
            )
        except BootloaderError as exc:
            raise BootloaderError(f"Boot failed: {exc}") from exc


def format_board_id(raw: bytes) -> str:
    return raw.hex()


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash RP2040 applications over RS485")
    parser.add_argument("binary", help="Path to .bin image built for the application slot")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Serial port (default: %(default)s)")
    parser.add_argument("-b", "--baud", default=115200, type=int, help="RS485 UART baud (default: %(default)s)")
    parser.add_argument("-c", "--chunk", default=512, type=int, help="Chunk size in bytes (<=1024, default: %(default)s)")
    parser.add_argument("-t", "--timeout", default=1.5, type=float, help="Per-chunk response timeout in seconds")
    parser.add_argument(
        "--erase-timeout",
        default=None,
        type=float,
        help="Timeout in seconds to wait for the flash erase phase (defaults to estimate based on image size)",
    )
    parser.add_argument("--no-boot", action="store_true", help="Do not ask the bootloader to start the new image")
    parser.add_argument("--retries", default=80, type=int, help="Ping retries while waiting for bootloader")
    parser.add_argument("--delay", default=0.25, type=float, help="Delay between ping retries (seconds)")
    parser.add_argument(
        "--node",
        type=lambda x: int(x, 0),
        default=0x10,
        help="R2 bus node ID to reset before flashing (default: 0x10)",
    )
    parser.add_argument(
        "--host-id",
        type=lambda x: int(x, 0),
        default=R2BUS_HOST_ID,
        help="Host/source ID to use when issuing the reset command (default: 0x00)",
    )
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Skip sending the ECU reset command before contacting the bootloader",
    )
    parser.add_argument(
        "--reset-timeout",
        default=1.5,
        type=float,
        help="Seconds to wait for an ECU reset ACK (default: %(default)s)",
    )
    parser.add_argument(
        "--bootloader-wait",
        default=1.0,
        type=float,
        help="Seconds to wait after reset before talking to the bootloader (default: %(default)s)",
    )
    parser.add_argument(
        "--command-retries",
        default=2,
        type=int,
        help="Extra resend attempts for bootloader commands before failing (default: %(default)s)",
    )
    args = parser.parse_args()

    if not (1 <= args.chunk <= 1024):
        parser.error("Chunk size must be between 1 and 1024 bytes")
    if not (0 <= args.node <= 0xFF):
        parser.error("Node ID must be between 0x00 and 0xFF")
    if not (0 <= args.host_id <= 0xFF):
        parser.error("Host ID must be between 0x00 and 0xFF")
    if args.command_retries < 0:
        parser.error("command-retries must be >= 0")
    node_id = args.node & 0xFF
    host_id = args.host_id & 0xFF
    try:
        with open(args.binary, "rb") as fh:
            image = fh.read()
    except OSError as exc:  # pragma: no cover - file system errors
        sys.exit(f"Failed to read binary: {exc}")

    print(f"Loaded {len(image)} bytes from {args.binary}")
    crc = zlib.crc32(image) & 0xFFFFFFFF
    print(f"Image CRC32: 0x{crc:08X}")

    client = None
    try:
        if not args.no_reset:
            print(f"Issuing ECU reset to node 0x{node_id:02X}...", flush=True)
            ack_received = request_application_reset(
                port=args.port,
                baud=args.baud,
                node_id=node_id,
                host_id=host_id,
                ack_timeout=args.reset_timeout,
                wait_for_ack=True,
            )
            if ack_received:
                print("Reset ACK received.")
            else:
                print("No reset ACK received; continuing.")
            if args.bootloader_wait > 0:
                print(f"Waiting {args.bootloader_wait:.2f}s for bootloader...", flush=True)
                time.sleep(args.bootloader_wait)

        client = BootloaderClient(
            args.port,
            args.baud,
            timeout=args.timeout,
            node_id=node_id,
            host_id=host_id,
            command_retries=args.command_retries,
        )
        print("Waiting for bootloader...", end="", flush=True)
        client.wait_for_bootloader(args.retries, args.delay, handshake_timeout=max(args.timeout, 1.0))
        print(" connected.")
        info = client.get_info()
        print(
            f"Board protocol={info['protocol']} app_present={info['app_present']} "
            f"size={info['app_size']} crc=0x{info['app_crc']:08X} id={format_board_id(info['board_id'])}"
        )
        if len(image) > info["app_max_size"]:
            raise BootloaderError("Binary does not fit in application slot")
        sectors = max(1, (len(image) + 4095) // 4096)
        if args.erase_timeout is not None:
            erase_timeout = args.erase_timeout
        else:
            # Allow generous time for sector erase (~120ms per 4k sector + additional slack)
            erase_timeout = max(3.0, sectors * 0.12 + 2.0)
        print(f"Erasing flash region (~{sectors} sectors)...", flush=True)
        client.start_session(len(image), crc, erase_timeout=erase_timeout)
        print("Programming image:")
        offset = 0
        start = time.monotonic()
        pb = tqdm.tqdm(total=len(image), unit="B", unit_scale=True)
        while offset < len(image):
            chunk = image[offset : offset + args.chunk]
            total = client.send_chunk(offset, chunk, timeout=args.timeout)
            offset += len(chunk)
            pb.update(len(chunk))
        pb.close()
        print()
        done_size, done_crc = client.finish()
        duration = time.monotonic() - start
        print(
            f"Programming complete in {duration:.2f}s "
            f"(size={done_size} crc=0x{done_crc:08X})"
        )
        if not args.no_boot:
            print("Requesting boot...")
            client.boot()
            print("Boot command acknowledged.")
        else:
            print("Boot skipped at user request.")
    except (BootloaderError, ProtocolError) as exc:
        print(f"\nError: {exc}")
        if client is not None:
            try:
                client.send_packet(CMD_ABORT)
            except Exception:  # pragma: no cover - best effort
                pass
        return 1
    finally:
        if client is not None:
            client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

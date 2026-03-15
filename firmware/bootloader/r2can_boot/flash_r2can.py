#!/usr/bin/env python3
"""Flash firmware to r2can bootloader over CAN.

Accepts Intel HEX (.hex) or raw binary (.bin) files.

Usage:
    flash_r2can.py firmware.hex --interface socketcan --channel can0
    flash_r2can.py bazel-bin/front_logic_2.hex --channel can0
    flash_r2can.py firmware.bin --channel vcan0 --node 1
"""
import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

try:
    import can  # type: ignore
except ImportError:
    sys.exit("python-can is required: pip install python-can")

try:
    import tqdm  # type: ignore
except ImportError:
    tqdm = None  # type: ignore

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
PROTOCOL_VERSION = 1

CMD_PING  = 0x01
CMD_INFO  = 0x02
CMD_START = 0x10
CMD_DATA  = 0x11
CMD_DONE  = 0x12
CMD_BOOT  = 0x13
CMD_ABORT = 0x14
CMD_ACK   = 0x80

STATUS_OK            = 0
STATUS_BAD_STATE     = 1
STATUS_INVALID_ARG   = 2
STATUS_TOO_LARGE     = 3
STATUS_ALIGNMENT     = 4
STATUS_CRC_MISMATCH  = 5
STATUS_INTERNAL_ERR  = 6
STATUS_NO_APP        = 7
STATUS_INVALID_CMD   = 8

STATUS_TEXT = {
    STATUS_OK:           "ok",
    STATUS_BAD_STATE:    "bad-state",
    STATUS_INVALID_ARG:  "invalid-arg",
    STATUS_TOO_LARGE:    "too-large",
    STATUS_ALIGNMENT:    "alignment",
    STATUS_CRC_MISMATCH: "crc-mismatch",
    STATUS_INTERNAL_ERR: "internal-error",
    STATUS_NO_APP:       "no-app",
    STATUS_INVALID_CMD:  "invalid-cmd",
}

CAN_ID_HOST_TO_DEV = 0x7E0
CAN_ID_DEV_TO_HOST = 0x7E8
MAX_FRAG_DATA      = 7
MAX_PKT_SIZE       = 1024


class BootloaderError(RuntimeError):
    pass


class ProtocolError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# Intel HEX parser
# ---------------------------------------------------------------------------
def parse_intel_hex(path: Path) -> bytes:
    """Parse an Intel HEX file and return a contiguous binary image.

    Supports I8HEX (8-bit) and I32HEX (32-bit extended linear address records).
    Fills gaps between records with 0xFF.
    """
    segments: dict[int, bytearray] = {}  # base_address -> data
    ext_addr = 0  # upper 16 bits of address for 32-bit addressing

    with open(path, "r") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line or line[0] != ":":
                continue
            try:
                raw = bytes.fromhex(line[1:])
            except ValueError:
                raise ProtocolError(f"Line {lineno}: invalid hex data")

            if len(raw) < 5:
                raise ProtocolError(f"Line {lineno}: record too short")

            byte_count = raw[0]
            address    = (raw[1] << 8) | raw[2]
            rec_type   = raw[3]
            data       = raw[4 : 4 + byte_count]
            checksum   = raw[4 + byte_count]

            # Verify checksum
            calc = (256 - (sum(raw[: 4 + byte_count]) & 0xFF)) & 0xFF
            if calc != checksum:
                raise ProtocolError(f"Line {lineno}: checksum mismatch")

            if rec_type == 0x00:  # Data record
                abs_addr = (ext_addr << 16) | address
                if abs_addr not in segments:
                    segments[abs_addr] = bytearray()
                segments[abs_addr].extend(data)

            elif rec_type == 0x01:  # End Of File
                break

            elif rec_type == 0x04:  # Extended Linear Address
                if len(data) != 2:
                    raise ProtocolError(f"Line {lineno}: bad extended address record")
                ext_addr = (data[0] << 8) | data[1]

            elif rec_type == 0x05:  # Start Linear Address (entry point)
                pass  # Ignored for flashing purposes

            elif rec_type == 0x02:  # Extended Segment Address
                if len(data) != 2:
                    raise ProtocolError(f"Line {lineno}: bad segment address record")
                ext_addr = ((data[0] << 8) | data[1]) >> 12  # upper 16 bits of 20-bit address

            # Other record types ignored

    if not segments:
        raise ProtocolError("No data records found in HEX file")

    # Find address range
    min_addr = min(segments)
    max_addr = max(
        base + len(data) for base, data in segments.items()
    )

    # Build contiguous binary, filling gaps with 0xFF
    image = bytearray(b"\xff" * (max_addr - min_addr))
    for base, data in segments.items():
        off = base - min_addr
        image[off : off + len(data)] = data

    return bytes(image)


# ---------------------------------------------------------------------------
# CAN transport
# ---------------------------------------------------------------------------
class CanBootloaderClient:
    def __init__(self, bus: can.BusABC, timeout: float = 2.0):
        self.bus = bus
        self.timeout = timeout
        self._rx_frags: list[bytes] = []
        self._rx_seq: int = -1

    def _send_packet(self, pkt: bytes) -> None:
        """Fragment and send a logical packet over CAN."""
        offset = 0
        seq = 0
        while offset < len(pkt):
            remaining = len(pkt) - offset
            frag_len = min(remaining, MAX_FRAG_DATA)
            is_last = (offset + frag_len >= len(pkt))
            frame_data = bytes([((0x80 if is_last else 0x00) | (seq & 0x7F))]) + pkt[offset:offset + frag_len]
            msg = can.Message(
                arbitration_id=CAN_ID_HOST_TO_DEV,
                data=frame_data,
                is_extended_id=False,
            )
            self.bus.send(msg, timeout=self.timeout)
            offset += frag_len
            seq = (seq + 1) & 0x7F

    def _recv_packet(self, timeout: float) -> bytes:
        """Receive and reassemble a logical packet from CAN fragments."""
        frags: list[bytes] = []
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProtocolError("Timed out waiting for CAN response")
            msg = self.bus.recv(timeout=remaining)
            if msg is None:
                continue
            if msg.arbitration_id != CAN_ID_DEV_TO_HOST:
                continue
            if len(msg.data) < 1:
                continue

            hdr = msg.data[0]
            seq = hdr & 0x7F
            is_last = bool(hdr & 0x80)
            frag_data = bytes(msg.data[1:])

            if seq == 0:
                frags = [frag_data]
            else:
                frags.append(frag_data)

            if is_last:
                return b"".join(frags)

    def _build_packet(self, cmd: int, payload: bytes = b"") -> bytes:
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        header = struct.pack("<BBHIH", cmd, PROTOCOL_VERSION, len(payload), crc, 0)
        # Actually: cmd(1) version(1) length(2) crc(4) = 8 bytes header
        header = bytes([cmd, PROTOCOL_VERSION]) + struct.pack("<H", len(payload)) + struct.pack("<I", crc)
        return header + payload

    def _parse_ack(self, pkt: bytes, expected_cmd: int) -> bytes:
        if len(pkt) < 8:
            raise ProtocolError(f"ACK packet too short: {len(pkt)}")
        cmd = pkt[0]
        version = pkt[1]
        payload_len = struct.unpack_from("<H", pkt, 2)[0]
        pkt_crc = struct.unpack_from("<I", pkt, 4)[0]

        if version != PROTOCOL_VERSION:
            raise ProtocolError(f"Version mismatch: {version}")
        if cmd != CMD_ACK:
            raise ProtocolError(f"Expected ACK (0x{CMD_ACK:02X}), got 0x{cmd:02X}")
        if len(pkt) < 8 + payload_len:
            raise ProtocolError("ACK packet truncated")

        payload = pkt[8:8 + payload_len]
        calc_crc = zlib.crc32(payload) & 0xFFFFFFFF
        if calc_crc != pkt_crc:
            raise ProtocolError("ACK CRC mismatch")

        if len(payload) < 2:
            raise ProtocolError("ACK payload too short")

        acked_cmd = payload[0]
        status = payload[1]
        if acked_cmd != expected_cmd:
            raise ProtocolError(f"ACK for wrong command: 0x{acked_cmd:02X} expected 0x{expected_cmd:02X}")
        if status != STATUS_OK:
            text = STATUS_TEXT.get(status, f"err-{status}")
            raise BootloaderError(f"Command 0x{expected_cmd:02X} failed: {text}")

        return payload[2:]

    def _command(self, cmd: int, payload: bytes = b"", timeout: float | None = None) -> bytes:
        pkt = self._build_packet(cmd, payload)
        self._send_packet(pkt)
        resp = self._recv_packet(timeout or self.timeout)
        return self._parse_ack(resp, cmd)

    def ping(self) -> None:
        self._command(CMD_PING)

    def get_info(self) -> dict:
        extra = self._command(CMD_INFO, timeout=3.0)
        if len(extra) < 24:
            raise ProtocolError(f"INFO response too short: {len(extra)}")
        proto, app_off, app_max, present, app_sz, app_crc = struct.unpack_from("<IIIIII", extra)
        return {
            "protocol": proto,
            "app_offset": app_off,
            "app_max_size": app_max,
            "app_present": present,
            "app_size": app_sz,
            "app_crc": app_crc,
        }

    def start_session(self, size: int, crc: int, erase_timeout: float = 10.0) -> None:
        self._command(CMD_START, struct.pack("<II", size, crc), timeout=erase_timeout)

    def send_chunk(self, offset: int, data: bytes) -> int:
        extra = self._command(CMD_DATA, struct.pack("<I", offset) + data, timeout=10.0)
        if len(extra) < 4:
            raise ProtocolError("DATA ACK missing progress counter")
        (total_written,) = struct.unpack_from("<I", extra)
        return total_written

    def finish(self) -> tuple[int, int]:
        extra = self._command(CMD_DONE, timeout=5.0)
        if len(extra) < 8:
            raise ProtocolError("DONE ACK payload too short")
        return struct.unpack_from("<II", extra)

    def boot(self) -> None:
        self._command(CMD_BOOT)

    def abort(self) -> None:
        try:
            self._command(CMD_ABORT)
        except Exception:
            pass

    def wait_for_bootloader(self, retries: int = 20, delay: float = 0.5) -> None:
        for attempt in range(retries):
            try:
                self.ping()
                return
            except (ProtocolError, BootloaderError, can.CanError):
                time.sleep(delay)
        raise ProtocolError("Bootloader did not respond to ping")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Flash r2can bootloader over CAN. Accepts .hex or .bin files.",
    )
    parser.add_argument(
        "firmware",
        help="Path to firmware file (.hex Intel HEX or .bin raw binary)",
    )
    parser.add_argument(
        "--interface", "-i",
        default="socketcan",
        help="python-can interface (default: %(default)s)",
    )
    parser.add_argument(
        "--channel", "-c",
        default="can0",
        help="CAN channel (default: %(default)s)",
    )
    parser.add_argument(
        "--bitrate",
        default=500000,
        type=int,
        help="CAN bitrate in bps (default: %(default)s)",
    )
    parser.add_argument(
        "--chunk",
        default=240,
        type=int,
        help="Firmware chunk size in bytes per DATA command (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout",
        default=3.0,
        type=float,
        help="Per-command timeout in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--ping-retries",
        default=20,
        type=int,
        help="Ping retries while waiting for bootloader (default: %(default)s)",
    )
    parser.add_argument(
        "--ping-delay",
        default=0.5,
        type=float,
        help="Delay between ping retries in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--erase-timeout",
        default=None,
        type=float,
        help="Flash erase timeout (default: estimated from image size)",
    )
    parser.add_argument(
        "--no-boot",
        action="store_true",
        help="Do not request boot after programming",
    )
    args = parser.parse_args()

    fw_path = Path(args.firmware)
    if not fw_path.exists():
        sys.exit(f"Firmware file not found: {fw_path}")

    # Load firmware
    suffix = fw_path.suffix.lower()
    if suffix == ".hex":
        print(f"Parsing Intel HEX file: {fw_path}")
        try:
            image = parse_intel_hex(fw_path)
        except (ProtocolError, OSError) as exc:
            sys.exit(f"Failed to parse HEX file: {exc}")
    elif suffix == ".bin":
        print(f"Loading binary file: {fw_path}")
        try:
            image = fw_path.read_bytes()
        except OSError as exc:
            sys.exit(f"Failed to read binary: {exc}")
    else:
        sys.exit(f"Unsupported file type '{suffix}'. Use .hex or .bin")

    print(f"Firmware size: {len(image)} bytes")
    image_crc = zlib.crc32(image) & 0xFFFFFFFF
    print(f"Firmware CRC32: 0x{image_crc:08X}")

    # Open CAN bus
    try:
        bus = can.interface.Bus(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
        )
    except can.CanError as exc:
        sys.exit(f"Failed to open CAN bus: {exc}")

    client = CanBootloaderClient(bus, timeout=args.timeout)
    try:
        print("Waiting for bootloader...", end="", flush=True)
        client.wait_for_bootloader(retries=args.ping_retries, delay=args.ping_delay)
        print(" connected.")

        info = client.get_info()
        print(
            f"Device: protocol={info['protocol']} "
            f"app_offset=0x{info['app_offset']:08X} "
            f"app_max={info['app_max_size']} bytes "
            f"app_present={info['app_present']}"
        )

        if len(image) > info["app_max_size"]:
            raise BootloaderError(
                f"Image too large: {len(image)} > {info['app_max_size']} bytes"
            )

        # Estimate erase timeout
        sectors = max(1, (len(image) + 4095) // 4096)
        erase_timeout = args.erase_timeout or max(5.0, sectors * 0.15 + 3.0)

        print(f"Erasing ~{sectors} flash sectors...", flush=True)
        client.start_session(len(image), image_crc, erase_timeout=erase_timeout)
        print("Flash erased.")

        print("Programming:")
        offset = 0
        start = time.monotonic()

        if tqdm is not None:
            pb = tqdm.tqdm(total=len(image), unit="B", unit_scale=True)
        else:
            pb = None

        while offset < len(image):
            chunk = image[offset:offset + args.chunk]
            client.send_chunk(offset, chunk)
            offset += len(chunk)
            if pb:
                pb.update(len(chunk))

        if pb:
            pb.close()

        written, done_crc = client.finish()
        duration = time.monotonic() - start
        print(f"\nProgramming complete in {duration:.2f}s  size={written}  CRC=0x{done_crc:08X}")

        if not args.no_boot:
            print("Requesting boot...")
            client.boot()
            print("Boot acknowledged.")
        else:
            print("Boot skipped.")

    except (BootloaderError, ProtocolError) as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        client.abort()
        return 1
    except can.CanError as exc:
        print(f"\nCAN error: {exc}", file=sys.stderr)
        return 1
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())

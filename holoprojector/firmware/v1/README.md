# Holoprojector Pico Firmware v1

This folder contains a standalone Raspberry Pi Pico application that blinks the
on-board LED and exposes a lightweight RS485-based “R2 bus” so multiple nodes
can communicate with a host over a shared two-wire differential link. The
project is structured as its own CMake target that pulls in the Official Pico
SDK so it can be built without relying on the rest of the ROS workspace.

## Requirements

- CMake ≥ 3.13
- ARM GCC toolchain (`gcc-arm-none-eabi`) and build tools (`ninja` or `make`)
- Pico SDK (either provide a local checkout via `PICO_SDK_PATH` or allow this
  project to fetch it automatically)

## Configure & Build

```bash
cd holoprojector/firmware/v1
cmake -B build -S . \
  -DPICO_BOARD=pico \
  -DPICO_SDK_FETCH_FROM_GIT=ON
cmake --build build
```

Notes:

- If you already have the Pico SDK, set `PICO_SDK_PATH=/path/to/pico-sdk` and
  omit `PICO_SDK_FETCH_FROM_GIT`.
- The default target name is `holoprojector_pico_v1`. Building will generate a
  `.uf2`, `.elf`, and `.bin` inside `build/`.

## Flashing

1. Hold the `BOOTSEL` button on the Pico, connect it over USB, and release the
   button to mount the device as a USB mass-storage drive.
2. Copy `build/holoprojector_pico_v1.uf2` onto the mounted drive.
3. The board will reboot automatically and start blinking the on-board LED
   every 500 ms.

## RS485 / R2 Bus

- Hardware: `uart1` with `TX=GPIO4`, `RX=GPIO5`, `DIR=GPIO6` (DIR high = TX).
- Protocol: framed packets beginning with `0x55 0xAA`, addressed source/dest
  bytes, message id, length (≤255), payload, and a CRC-16/CCITT checksum.
- Library: `include/rs485.h` drives the transceiver and `include/r2bus.h`
  handles framing, CRC validation, ACKs, ping/pong keep-alives, and the ECU
  reset command that instructs a node to reboot for OTA updates.
- Multi-node: every Pico sets its own `NODE_ID` (default `0x10`). Messages may
  target another node id, the host (`0x00`), or broadcast (`0xFF`).
- ECU reset command: when the host sends `R2BUS_MSG_ECU_RESET` to a node, the
  firmware replies with an ACK then schedules a watchdog reboot so the bootloader
  can take over for OTA flashing.

### Host Utilities

- `r2bus_tool.py` offers quick logging and transmit capabilities from a PC:
  ```bash
  # Continuously log traffic
  python3 r2bus_tool.py --port /dev/ttyUSB0 log

  # Send a ping to node 0x10 and wait for a PONG
  python3 r2bus_tool.py --port /dev/ttyUSB0 ping 0x10

  # Issue an ECU reset (wait for ACK by default)
  python3 r2bus_tool.py --port /dev/ttyUSB0 reset 0x10
  ```
- The tool depends on `pyserial` (`pip install pyserial`) and prints frames in
  the same format understood by the firmware library, including ACK details and
  payload hex dumps.

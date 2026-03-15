# R2CAN Boot - RGB LED Demo Application

Demo application for custom RP2354 board using the R2CAN BSP library.

## Features

- RGB LED cycling pattern (Red → Green → Blue → Off)
- Uses shared R2CAN BSP library from `firmware/libs/r2can_bsp`
- 500ms interval between colors
- USB serial output enabled for debugging

## Hardware

This application uses the R2CAN BSP which defines:
- Red LED: GPIO 12
- Green LED: GPIO 13
- Blue LED: GPIO 14

## Customization

### Change Blink Pattern

Edit [main.c](main.c) to change the LED pattern. See [examples.c](../../libs/r2can_bsp/examples.c) for more pattern ideas:

```c
// Example: All LEDs blinking together
while (true) {
    board_led_all_on();
    sleep_ms(500);
    board_led_all_off();
    sleep_ms(500);
}
```

### Change Blink Speed

Modify the `BLINK_INTERVAL_MS` value in [main.c:4](main.c#L4):

```c
#define BLINK_INTERVAL_MS 500  // Time in milliseconds
```

### Add More Peripherals

Extend the R2CAN BSP at [firmware/libs/r2can_bsp](../../libs/r2can_bsp) to add support for UART, SPI, I2C, or other peripherals. See the [BSP README](../../libs/r2can_bsp/README.md) for details.

## Building

### Prerequisites

- Pico SDK installed
- CMake 3.13 or higher
- ARM GCC compiler (arm-none-eabi-gcc)

### Build Steps

#### Option 1: Using Make (Easiest)

```bash
cd /home/eric/r2d2/firmware/bootloader/r2can_boot

# Build for RP2354 (default)
make

# Build and flash
make flash

# Clean build
make clean

# Build for RP2040
make rp2040

# See all targets
make help
```

#### Option 2: Using the build script

```bash
cd /home/eric/r2d2/firmware/bootloader/r2can_boot

# Build for RP2354 (default)
./build.sh

# Clean build
./build.sh --clean

# Build and flash
./build.sh --flash

# Build for RP2040
./build.sh --board pico

# See all options
./build.sh --help
```

#### Option 3: Manual CMake build

```bash
cd /home/eric/r2d2/firmware/bootloader/r2can_boot

# Create build directory
mkdir -p build
cd build

# Configure for RP2350/RP2354 (Pico 2)
cmake .. -DPICO_BOARD=pico2

# Or configure for RP2040 (Pico 1)
# cmake .. -DPICO_BOARD=pico

# Build
make -j4
```

#### Option 4: VSCode Tasks

If using VSCode, press `Ctrl+Shift+B` to access build tasks:
- **Build r2can_boot** - Standard build
- **Clean Build r2can_boot** - Clean and rebuild
- **Build and Flash r2can_boot** - Build and flash to device
- **Build r2can_boot (RP2040)** - Build for RP2040 instead of RP2354

The build will generate several files in the `build` directory:
- `r2can_boot.uf2` - Drag-and-drop firmware file (use this!)
- `r2can_boot.elf` - Executable with debug symbols
- `r2can_boot.bin` - Raw binary
- `r2can_boot.hex` - Intel HEX format

## Flashing

### Method 1: Drag and Drop (Recommended)

1. Hold the BOOTSEL button on your board while plugging it into USB
2. The board will appear as a USB mass storage device (RPI-RP2)
3. Drag and drop `build/r2can_boot.uf2` onto the drive
4. The board will automatically reboot and start the RGB cycling pattern

### Method 2: Using picotool

```bash
# Load firmware
picotool load -x build/r2can_boot.uf2

# Or force load if board is running
picotool load -f build/r2can_boot.uf2
picotool reboot
```

### Method 3: Using OpenOCD/Debug Probe

If you have a debug probe (e.g., Raspberry Pi Debug Probe) connected:

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
    -c "program build/r2can_boot.elf verify reset exit"
```

## Project Structure

```
r2can_boot/
├── main.c              - Application entry point
├── CMakeLists.txt      - Build configuration
├── pico_sdk_import.cmake
└── README.md           - This file

../../libs/r2can_bsp/   - Shared board support library
├── include/
│   └── board.h         - BSP API and pin definitions
├── src/
│   └── board.c         - BSP implementation
├── examples.c          - Usage examples
├── CMakeLists.txt      - Library build configuration
└── README.md           - BSP documentation
```

## Troubleshooting

- **LEDs not working**: Verify your board uses GPIO 12/13/14 for RGB LEDs. If not, update the pin definitions in [board.h](../../libs/r2can_bsp/include/board.h)
- **Build fails - BSP not found**: Check that the R2CAN BSP exists at `firmware/libs/r2can_bsp`
- **Build fails - SDK not found**: Ensure `PICO_SDK_PATH` environment variable is set or pass it to cmake: `cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk`
- **Wrong board type**: Make sure to specify the correct PICO_BOARD (pico2 for RP2354, pico for RP2040)
- **USB serial not working**: Check that USB cable supports data (not just power)

## See Also

- [R2CAN BSP Documentation](../../libs/r2can_bsp/README.md)
- [R2CAN BSP Examples](../../libs/r2can_bsp/examples.c)
- [Pico SDK Documentation](https://www.raspberrypi.com/documentation/pico-sdk/)

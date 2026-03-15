# Makefile wrapper for r2can_boot build system
# Provides convenient make targets for building the firmware

.PHONY: all clean flash help rp2040 rp2350

# Default target
all: rp2350

# Build for RP2350/RP2354 (Pico 2)
rp2350:
	@./build.sh --board pico2

# Build for RP2040 (Pico 1)
rp2040:
	@./build.sh --board pico

# Clean build directory
clean:
	@rm -rf build
	@echo "Build directory cleaned"

# Build and flash firmware
flash:
	@./build.sh --flash

# Rebuild from scratch
rebuild: clean all

# Show help
help:
	@echo "R2CAN Boot Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make              - Build for RP2350/RP2354 (default)"
	@echo "  make rp2350       - Build for RP2350/RP2354"
	@echo "  make rp2040       - Build for RP2040"
	@echo "  make clean        - Remove build directory"
	@echo "  make flash        - Build and flash to device"
	@echo "  make rebuild      - Clean and rebuild"
	@echo "  make help         - Show this help"
	@echo ""
	@echo "For more options, run: ./build.sh --help"

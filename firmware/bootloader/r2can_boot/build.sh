#!/bin/bash
# Build script for r2can_boot firmware

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
BUILD_DIR="build"
JOBS=$(nproc 2>/dev/null || echo 4)
CLEAN=false
FLASH=false

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Print usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Build r2can_boot firmware for RP2354 board"
    echo ""
    echo "Options:"
    echo "  -b, --board BOARD    Target board (default: pico2)"
    echo "                       Options: pico2 (RP2350), pico (RP2040)"
    echo "  -c, --clean          Clean build directory before building"
    echo "  -f, --flash          Flash firmware after building (requires picotool)"
    echo "  -j, --jobs N         Number of parallel build jobs (default: $JOBS)"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                   # Build for pico2"
    echo "  $0 --clean           # Clean build"
    echo "  $0 --flash           # Build and flash"
    echo "  $0 -b pico           # Build for RP2040"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--board)
            BOARD="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN=true
            shift
            ;;
        -f|--flash)
            FLASH=true
            shift
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}Error: Unknown option $1${NC}"
            usage
            exit 1
            ;;
    esac
done

# Change to script directory
cd "$SCRIPT_DIR"

echo -e "${GREEN}=== R2CAN Boot Build Script ===${NC}"
echo "Build directory: $BUILD_DIR"
echo "Jobs: $JOBS"
echo ""

# Clean if requested
if [ "$CLEAN" = true ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf "$BUILD_DIR"
fi

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo -e "${GREEN}Configuring CMake...${NC}"
cmake .. -DPICO_PLATFORM="rp2350" || {
    echo -e "${RED}CMake configuration failed!${NC}"
    exit 1
}

# Build
echo -e "${GREEN}Building...${NC}"
make -j"$JOBS" || {
    echo -e "${RED}Build failed!${NC}"
    exit 1
}

# Success
echo ""
echo -e "${GREEN}✓ Build successful!${NC}"
echo ""
echo "Output files:"
ls -lh r2can_boot.uf2 r2can_boot.elf 2>/dev/null || true

# Flash if requested
if [ "$FLASH" = true ]; then
    echo ""
    echo -e "${YELLOW}Flashing firmware...${NC}"

    if ! command -v picotool &> /dev/null; then
        echo -e "${RED}Error: picotool not found!${NC}"
        echo "Please install picotool or flash manually:"
        echo "  1. Hold BOOTSEL button while connecting board"
        echo "  2. Drag and drop: $SCRIPT_DIR/$BUILD_DIR/r2can_boot.uf2"
        exit 1
    fi

    echo "Attempting to flash with picotool..."
    if picotool load -f r2can_boot.uf2; then
        echo -e "${GREEN}✓ Flash successful!${NC}"
        echo "Rebooting device..."
        picotool reboot
    else
        echo -e "${RED}Flash failed!${NC}"
        echo "Try manually:"
        echo "  1. Hold BOOTSEL button while connecting board"
        echo "  2. Drag and drop: $SCRIPT_DIR/$BUILD_DIR/r2can_boot.uf2"
        exit 1
    fi
fi

echo ""
echo -e "${GREEN}Done!${NC}"

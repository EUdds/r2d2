cmake_minimum_required(VERSION 3.13)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# IMPORTANT: Set platform BEFORE including Pico SDK (matches official examples)
# Only set PICO_PLATFORM, not PICO_BOARD (let SDK auto-detect board)
set(PICO_PLATFORM rp2350 CACHE STRING "Platform type" FORCE)

# Include the Pico SDK
include(pico_sdk_import.cmake)

# Set project name and supported languages
project(r2can_boot C CXX ASM)

# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add the V2 BSP library
set(V2_BSP_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libs/v2_bsp")
if(NOT EXISTS "${V2_BSP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "V2 BSP library not found at ${V2_BSP_DIR}")
endif()
add_subdirectory(${V2_BSP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/v2_bsp)

# Add the MCP2515 CAN library
set(MCP2515_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libs/mcp2515")
if(NOT EXISTS "${MCP2515_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "MCP2515 library not found at ${MCP2515_DIR}")
endif()
add_subdirectory(${MCP2515_DIR} ${CMAKE_CURRENT_BINARY_DIR}/mcp2515)

# Create the executable
add_executable(r2can_boot
    main.c
)

# Link against libraries
target_link_libraries(r2can_boot
    pico_stdlib
    hardware_spi
    v2_bsp
    mcp2515
)

# Enable USB stdio for debugging
pico_enable_stdio_usb(r2can_boot 1)
pico_enable_stdio_uart(r2can_boot 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(r2can_boot)

# Copy compile_commands.json to project root and repo root for clangd
add_custom_command(TARGET r2can_boot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/../../../compile_commands.json
    COMMENT "Copying compile_commands.json to project and repo root"
)

cmake_minimum_required(VERSION 3.13)

# MCP2515 CAN Controller Driver Library
# Simple driver for MCP2515 standalone CAN controller

add_library(mcp2515 STATIC
    src/mcp2515.c
)

target_include_directories(mcp2515
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(mcp2515
    PUBLIC
        pico_stdlib
        hardware_spi
)

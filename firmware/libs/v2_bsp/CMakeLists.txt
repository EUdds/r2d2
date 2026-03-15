cmake_minimum_required(VERSION 3.13)

# V2 Board Support Package Library
# Provides hardware abstraction for custom RP2354 boards

add_library(v2_bsp STATIC
    src/board.c
)

target_include_directories(v2_bsp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(v2_bsp
    PUBLIC
        pico_stdlib
        hardware_spi
)

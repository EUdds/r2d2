cmake_minimum_required(VERSION 3.13)

add_library(st7789 STATIC
    src/st7789.c
)

target_include_directories(st7789
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(st7789
    PUBLIC
        pico_stdlib
        hardware_spi
)

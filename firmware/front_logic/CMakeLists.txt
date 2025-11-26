cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(front_logic_v1 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs)

set(R2BUS_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../libs/r2bus")
if(NOT EXISTS "${R2BUS_LIB_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Expected shared r2bus library at ${R2BUS_LIB_DIR}")
endif()

set(NANOPB_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../holoprojector/firmware/v1/libs/nanopb")
if(NOT EXISTS "${NANOPB_LIB_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Expected nanopb library at ${NANOPB_LIB_DIR}")
endif()

add_subdirectory(${NANOPB_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nanopb)

set(R2BUS_NANOPB_HINTS
    ${NANOPB_LIB_DIR}/generator
    ${NANOPB_LIB_DIR}
)
add_subdirectory(${R2BUS_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/r2bus)

add_executable(front_logic_v1
    src/main.c
    src/r2bus_receiver.c
    src/tasks.c
    src/status_led.c
    src/screen.c
    src/logic_lights.c
)


target_link_libraries(front_logic_v1
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_spi
    r2bus
)

target_include_directories(front_logic_v1
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/proto
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/pico-libfixmath/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../holoprojector
)

pico_set_linker_script(front_logic_v1 ${CMAKE_CURRENT_LIST_DIR}/linker.ld)

pico_enable_stdio_usb(front_logic_v1 1)
pico_enable_stdio_uart(front_logic_v1 0)

pico_add_extra_outputs(front_logic_v1)

# Copy the compile_commands.json to the root directory
add_custom_command(TARGET front_logic_v1 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_SOURCE_DIR}/compile_commands.json
)

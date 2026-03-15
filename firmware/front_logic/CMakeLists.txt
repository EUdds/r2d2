cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(FRONT_LOGIC_PCB_REV "1" CACHE STRING "PCB revision (1 or 2)")

# Default to shared pico-sdk in libs/ if not already specified
if(NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(_shared_sdk "${CMAKE_CURRENT_LIST_DIR}/../libs/pico-sdk")
    if(EXISTS "${_shared_sdk}/CMakeLists.txt")
        set(PICO_SDK_PATH "${_shared_sdk}")
    endif()
endif()

# Must be set before pico_sdk_import
if(FRONT_LOGIC_PCB_REV GREATER_EQUAL 2)
    set(PICO_PLATFORM "rp2350" CACHE STRING "Pico platform" FORCE)
endif()

include(pico_sdk_import.cmake)

project(front_logic_v${FRONT_LOGIC_PCB_REV} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(COMMON_LIBS_DIR "${CMAKE_CURRENT_LIST_DIR}/../libs")
set(TARGET_NAME "front_logic_v${FRONT_LOGIC_PCB_REV}")
set(COMMS_SRCS "")
set(COMMS_LIBS "")

if(FRONT_LOGIC_PCB_REV GREATER_EQUAL 2)
    set(V2_BSP_DIR "${COMMON_LIBS_DIR}/v2_bsp")
    if(NOT EXISTS "${V2_BSP_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "v2_bsp library not found at ${V2_BSP_DIR}")
    endif()
    add_subdirectory(${V2_BSP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/v2_bsp)

    set(MCP2515_DIR "${COMMON_LIBS_DIR}/mcp2515")
    if(NOT EXISTS "${MCP2515_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "mcp2515 library not found at ${MCP2515_DIR}")
    endif()
    add_subdirectory(${MCP2515_DIR} ${CMAKE_CURRENT_BINARY_DIR}/mcp2515)

    list(APPEND COMMS_SRCS src/can_receiver.c)
    list(APPEND COMMS_LIBS v2_bsp mcp2515)
else()
    set(R2BUS_LIB_DIR "${COMMON_LIBS_DIR}/r2bus")
    if(NOT EXISTS "${R2BUS_LIB_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Expected shared r2bus library at ${R2BUS_LIB_DIR}")
    endif()

    set(NANOPB_LIB_DIR "${COMMON_LIBS_DIR}/nanopb")
    if(NOT EXISTS "${NANOPB_LIB_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Expected shared nanopb library at ${NANOPB_LIB_DIR}")
    endif()

    add_subdirectory(${NANOPB_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nanopb)

    set(R2BUS_NANOPB_HINTS
        ${NANOPB_LIB_DIR}/generator
        ${NANOPB_LIB_DIR}
    )
    add_subdirectory(${R2BUS_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/r2bus)

    list(APPEND COMMS_SRCS src/r2bus_receiver.c)
    list(APPEND COMMS_LIBS r2bus)
endif()

set(ST7789_DIR "${COMMON_LIBS_DIR}/st7789")
if(NOT EXISTS "${ST7789_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "st7789 library not found at ${ST7789_DIR}")
endif()
add_subdirectory(${ST7789_DIR} ${CMAKE_CURRENT_BINARY_DIR}/st7789)

set(FIXMATH_INCLUDE_DIR "${COMMON_LIBS_DIR}/pico-libfixmath/include")
if(NOT EXISTS "${FIXMATH_INCLUDE_DIR}")
    set(FIXMATH_INCLUDE_DIR "")
    message(WARNING "pico-libfixmath include dir not found at ${COMMON_LIBS_DIR}/pico-libfixmath/include; continuing without it")
endif()

add_executable(${TARGET_NAME}
    src/main.c
    src/tasks.c
    src/status_led.c
    src/screen.c
    src/logic_lights.c
    ${COMMS_SRCS}
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    PCB_REV=${FRONT_LOGIC_PCB_REV}
)

target_link_libraries(${TARGET_NAME}
    pico_stdlib
    pico_rand
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_spi
    st7789
    ${COMMS_LIBS}
)

target_include_directories(${TARGET_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/proto
        ${CMAKE_CURRENT_SOURCE_DIR}/../../holoprojector
)

if(FIXMATH_INCLUDE_DIR)
    target_include_directories(${TARGET_NAME} PRIVATE ${FIXMATH_INCLUDE_DIR})
endif()

if(FRONT_LOGIC_PCB_REV LESS 2)
    pico_set_linker_script(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/linker.ld)
endif()

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})

# Copy the compile_commands.json to the root directory
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_LIST_DIR}/../../compile_commands.json
)

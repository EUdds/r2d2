cmake_minimum_required(VERSION 3.15)

project(r2d2_firmware_superbuild LANGUAGES C CXX)

include(ExternalProject)

option(BUILD_FRONT_LOGIC    "Build front_logic v1 firmware" ON)
option(BUILD_FRONT_LOGIC_V2 "Build front_logic v2 firmware" ON)
option(BUILD_HOLOPROJECTOR  "Build holoprojector firmware" ON)
option(BUILD_BOOTLOADER     "Build rs485 bootloader firmware" ON)
option(BUILD_CAN_EXAMPLE    "Build CAN example application" ON)

set(PICO_BOARD "pico" CACHE STRING "Pico SDK board type (v1 builds)")
set(PICO_SDK_PATH "" CACHE PATH "Path to the Pico SDK (optional)")

# Default to the shared SDK in libs/ if present and not already specified
if(NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(_shared_sdk "${CMAKE_CURRENT_LIST_DIR}/libs/pico-sdk")
    if(EXISTS "${_shared_sdk}/CMakeLists.txt")
        set(PICO_SDK_PATH "${_shared_sdk}" CACHE PATH "Path to the Pico SDK (optional)" FORCE)
        message(STATUS "Using shared Pico SDK at ${PICO_SDK_PATH}")
    endif()
elseif(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH "Path to the Pico SDK (optional)")
endif()

set(BOOTLOADER_BLINK_NODE_CONFIG "" CACHE FILEPATH "Path to bootloader node_config.json (optional)")
set(BOOTLOADER_BLINK_VARIANTS "" CACHE STRING "Bootloader variants NAME:ID[:PCB_REV] list")
set(BOOTLOADER_BLINK_DEFAULT_NODE_ID "0x10u" CACHE STRING "Bootloader default node ID")
set(BOOTLOADER_BLINK_DEFAULT_PCB_REV "0" CACHE STRING "Bootloader default PCB revision")

# Common args for RP2040 (v1) builds
set(_common_cmake_args "")
if(CMAKE_BUILD_TYPE)
    list(APPEND _common_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
endif()
list(APPEND _common_cmake_args -DPICO_BOARD=${PICO_BOARD})
if(PICO_SDK_PATH)
    list(APPEND _common_cmake_args -DPICO_SDK_PATH=${PICO_SDK_PATH})
endif()

# Args for RP2350 (v2) builds — no PICO_BOARD, platform is set by the sub-project
set(_v2_cmake_args "")
if(CMAKE_BUILD_TYPE)
    list(APPEND _v2_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
endif()
if(PICO_SDK_PATH)
    list(APPEND _v2_cmake_args -DPICO_SDK_PATH=${PICO_SDK_PATH})
endif()

function(add_firmware_project target_name source_dir)
    ExternalProject_Add(${target_name}
        SOURCE_DIR ${source_dir}
        BINARY_DIR ${CMAKE_BINARY_DIR}/${target_name}
        CMAKE_ARGS ${_common_cmake_args}
        INSTALL_COMMAND ""
    )
endfunction()

set(_firmware_targets "")

if(BUILD_FRONT_LOGIC)
    ExternalProject_Add(front_logic
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/front_logic
        BINARY_DIR ${CMAKE_BINARY_DIR}/front_logic
        CMAKE_ARGS ${_common_cmake_args} -DFRONT_LOGIC_PCB_REV=1
        INSTALL_COMMAND ""
    )
    list(APPEND _firmware_targets front_logic)
endif()

if(BUILD_FRONT_LOGIC_V2)
    ExternalProject_Add(front_logic_v2
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/front_logic
        BINARY_DIR ${CMAKE_BINARY_DIR}/front_logic_v2
        CMAKE_ARGS ${_v2_cmake_args} -DFRONT_LOGIC_PCB_REV=2
        INSTALL_COMMAND ""
    )
    list(APPEND _firmware_targets front_logic_v2)
endif()

if(BUILD_HOLOPROJECTOR)
    add_firmware_project(holoprojector ${CMAKE_CURRENT_LIST_DIR}/holoprojector)
    list(APPEND _firmware_targets holoprojector)
endif()

if(BUILD_BOOTLOADER)
    set(_bootloader_args ${_common_cmake_args})
    if(BOOTLOADER_BLINK_NODE_CONFIG)
        list(APPEND _bootloader_args -DBLINK_NODE_CONFIG=${BOOTLOADER_BLINK_NODE_CONFIG})
    endif()
    if(BOOTLOADER_BLINK_VARIANTS)
        list(APPEND _bootloader_args -DBLINK_VARIANTS=${BOOTLOADER_BLINK_VARIANTS})
    endif()
    list(APPEND _bootloader_args -DBLINK_DEFAULT_NODE_ID=${BOOTLOADER_BLINK_DEFAULT_NODE_ID})
    list(APPEND _bootloader_args -DBLINK_DEFAULT_PCB_REV=${BOOTLOADER_BLINK_DEFAULT_PCB_REV})

    ExternalProject_Add(bootloader
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/bootloader/rp2040-rs485-bootloader
        BINARY_DIR ${CMAKE_BINARY_DIR}/bootloader
        CMAKE_ARGS ${_bootloader_args}
        INSTALL_COMMAND ""
    )
    list(APPEND _firmware_targets bootloader)
endif()

if(BUILD_CAN_EXAMPLE)
    add_firmware_project(can_example ${CMAKE_CURRENT_LIST_DIR}/can_example)
    list(APPEND _firmware_targets can_example)
endif()

if(_firmware_targets)
    add_custom_target(firmware_all ALL DEPENDS ${_firmware_targets})
endif()

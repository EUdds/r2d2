cmake_minimum_required(VERSION 3.15)

project(r2d2_firmware_superbuild LANGUAGES C CXX)

include(ExternalProject)

option(BUILD_FRONT_LOGIC "Build front_logic firmware" ON)
option(BUILD_HOLOPROJECTOR "Build holoprojector firmware" ON)
option(BUILD_BOOTLOADER "Build rs485 bootloader firmware" ON)

set(PICO_BOARD "pico" CACHE STRING "Pico SDK board type")
set(PICO_SDK_PATH "" CACHE PATH "Path to the Pico SDK (optional)")
option(PICO_SDK_FETCH_FROM_GIT "Fetch Pico SDK if not found locally" OFF)
set(PICO_SDK_FETCH_FROM_GIT_TAG "" CACHE STRING "Pico SDK git tag (optional)")

set(BOOTLOADER_BLINK_NODE_CONFIG "" CACHE FILEPATH "Path to bootloader node_config.json (optional)")
set(BOOTLOADER_BLINK_VARIANTS "" CACHE STRING "Bootloader variants NAME:ID[:PCB_REV] list")
set(BOOTLOADER_BLINK_DEFAULT_NODE_ID "0x10u" CACHE STRING "Bootloader default node ID")
set(BOOTLOADER_BLINK_DEFAULT_PCB_REV "0" CACHE STRING "Bootloader default PCB revision")

if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH "Path to the Pico SDK (optional)")
endif()

if(NOT PICO_SDK_FETCH_FROM_GIT AND DEFINED ENV{PICO_SDK_FETCH_FROM_GIT})
    string(TOUPPER "$ENV{PICO_SDK_FETCH_FROM_GIT}" _pico_fetch_env)
    if(_pico_fetch_env AND NOT _pico_fetch_env STREQUAL "0" AND NOT _pico_fetch_env STREQUAL "OFF"
       AND NOT _pico_fetch_env STREQUAL "FALSE")
        set(PICO_SDK_FETCH_FROM_GIT ON CACHE BOOL "Fetch Pico SDK if not found locally" FORCE)
    endif()
endif()

if(NOT PICO_SDK_FETCH_FROM_GIT_TAG AND DEFINED ENV{PICO_SDK_FETCH_FROM_GIT_TAG})
    set(PICO_SDK_FETCH_FROM_GIT_TAG "$ENV{PICO_SDK_FETCH_FROM_GIT_TAG}" CACHE STRING "Pico SDK git tag (optional)")
endif()

set(_common_cmake_args "")
if(CMAKE_BUILD_TYPE)
    list(APPEND _common_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
endif()
list(APPEND _common_cmake_args -DPICO_BOARD=${PICO_BOARD})
if(PICO_SDK_PATH)
    list(APPEND _common_cmake_args -DPICO_SDK_PATH=${PICO_SDK_PATH})
endif()
list(APPEND _common_cmake_args -DPICO_SDK_FETCH_FROM_GIT=${PICO_SDK_FETCH_FROM_GIT})
if(PICO_SDK_FETCH_FROM_GIT_TAG)
    list(APPEND _common_cmake_args -DPICO_SDK_FETCH_FROM_GIT_TAG=${PICO_SDK_FETCH_FROM_GIT_TAG})
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
    add_firmware_project(front_logic ${CMAKE_CURRENT_LIST_DIR}/front_logic)
    list(APPEND _firmware_targets front_logic)
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

if(_firmware_targets)
    add_custom_target(firmware_all ALL DEPENDS ${_firmware_targets})
endif()

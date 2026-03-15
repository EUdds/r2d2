# This helper mirrors the import file shipped with Pico SDK and adds an
# optional FetchContent flow so the SDK can be retrieved automatically.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the Raspberry Pi Pico SDK")
set(PICO_SDK_FETCH_FROM_GIT ON CACHE BOOL "Allow pico_sdk_import to download the SDK from Git")
set(PICO_SDK_FETCH_FROM_GIT_PATH "${CMAKE_BINARY_DIR}/sdk" CACHE PATH "Where to store a downloaded Pico SDK")
set(PICO_SDK_GIT_TAG "2.0.0" CACHE STRING "Git tag or branch for the Pico SDK")

if (NOT PICO_SDK_PATH)
    if (PICO_SDK_FETCH_FROM_GIT)
        include(FetchContent)
        FetchContent_Declare(pico_sdk
            GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
            GIT_TAG        ${PICO_SDK_GIT_TAG}
            SOURCE_DIR     ${PICO_SDK_FETCH_FROM_GIT_PATH}/pico-sdk
        )
        FetchContent_GetProperties(pico_sdk)
        if (NOT pico_sdk_POPULATED)
            message(STATUS "Fetching pico-sdk (${PICO_SDK_GIT_TAG})")
            FetchContent_Populate(pico_sdk)
        endif()
        set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
    else()
        message(FATAL_ERROR
            "PICO_SDK_PATH is not set. Provide it or enable PICO_SDK_FETCH_FROM_GIT.")
    endif()
endif()

if (NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_PATH}")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

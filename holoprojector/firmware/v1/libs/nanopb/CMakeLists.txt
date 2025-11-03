cmake_minimum_required(VERSION 3.10)

set(NANOPB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(NANOPB_SOURCE_DIR ${NANOPB_ROOT}/holoprojector)

add_library(nanopb STATIC
    ${NANOPB_SOURCE_DIR}/pb_common.c
    ${NANOPB_SOURCE_DIR}/pb_decode.c
    ${NANOPB_SOURCE_DIR}/pb_encode.c
)

target_include_directories(nanopb
    PUBLIC
        ${NANOPB_SOURCE_DIR}
)

cmake_minimum_required(VERSION 3.10)

project(nanopb C)

add_library(nanopb STATIC
    pb_common.c
    pb_decode.c
    pb_encode.c
)

# Publish include directory so dependents can include pb_*.h
# We deliberately avoid pulling in the upstream nanopb CMake logic, which
# requires the protoc toolchain at configure time. The checked-in generated
# sources (r2bus.pb.c/h) are built from the Python tooling already, so only the
# runtime support library is needed here.
target_include_directories(nanopb
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

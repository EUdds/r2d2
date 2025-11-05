cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(holoprojector_pico_v1 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs)

add_executable(holoprojector_pico_v1
    src/main.c
    src/rs485.c
    src/r2bus.c
    src/r2bus_receiver.c
    src/r2bus_transmitter.c
    src/tasks.c
    src/status_led.c
    src/ws2812.c
    src/pico_servo.c
    src/holoprojector_lights.c
    src/psi_light.c
    # proto/r2bus.pb.c
)

find_program(PROTOC_EXECUTABLE protoc)
find_program(NANOPB_PLUGIN nanopb_generator.py PATHS ${CMAKE_CURRENT_SOURCE_DIR}/libs/nanopb/generator)

if (PROTOC_EXECUTABLE AND NANOPB_PLUGIN)
  set(PROTO_SRC ${CMAKE_CURRENT_SOURCE_DIR}/proto/r2bus.proto)
  set(PROTO_OPTS ${CMAKE_CURRENT_SOURCE_DIR}/proto/r2bus.options)
  set(GENERATED_C ${CMAKE_CURRENT_BINARY_DIR}/r2bus.pb.c)
  set(GENERATED_H ${CMAKE_CURRENT_BINARY_DIR}/r2bus.pb.h)

  add_custom_command(
    OUTPUT ${GENERATED_C} ${GENERATED_H}
    COMMAND ${PROTOC_EXECUTABLE}
            -I=${CMAKE_CURRENT_SOURCE_DIR}/proto
            --nanopb_opt=-I${CMAKE_CURRENT_SOURCE_DIR}/proto
            --nanopb_out=${CMAKE_CURRENT_BINARY_DIR}
            ${PROTO_SRC}
    DEPENDS ${PROTO_SRC} ${PROTO_OPTS}
    COMMENT "Generating nanopb sources from r2bus.proto"
  )

  add_custom_target(generate_r2bus_proto DEPENDS ${GENERATED_C} ${GENERATED_H})
  add_dependencies(holoprojector_pico_v1 generate_r2bus_proto)

  target_include_directories(holoprojector_pico_v1 PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_sources(holoprojector_pico_v1 PRIVATE ${GENERATED_C})
else()
  message(WARNING "protoc/nanopb generator not found; using pre-generated r2bus.pb.c/.h")
endif()



pico_generate_pio_header(holoprojector_pico_v1 ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)
pico_generate_pio_header(holoprojector_pico_v1 ${CMAKE_CURRENT_LIST_DIR}/src/pico_servo.pio)

target_link_libraries(holoprojector_pico_v1
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_pwm
    fixmath
    nanopb
)

target_include_directories(holoprojector_pico_v1
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/proto
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/pico-libfixmath/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../holoprojector
)

pico_set_linker_script(holoprojector_pico_v1 ${CMAKE_CURRENT_LIST_DIR}/linker.ld)

pico_enable_stdio_usb(holoprojector_pico_v1 1)
pico_enable_stdio_uart(holoprojector_pico_v1 0)

pico_add_extra_outputs(holoprojector_pico_v1)

# Copy the compile_commands.json to the root directory
add_custom_command(TARGET holoprojector_pico_v1 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_SOURCE_DIR}/compile_commands.json
)

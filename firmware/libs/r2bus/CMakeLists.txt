add_library(r2bus STATIC
    src/r2bus.c
    src/r2bus_transmitter.c
    src/rs485.c
)

target_include_directories(r2bus
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}  # For generated protobuf headers
)

target_link_libraries(r2bus
    PUBLIC
        pico_stdlib
        nanopb
)

set(_r2bus_nanopb_search_paths
    ${R2BUS_NANOPB_HINTS}
    ${CMAKE_SOURCE_DIR}/libs/nanopb
    ${CMAKE_SOURCE_DIR}/../libs/nanopb
    ${CMAKE_SOURCE_DIR}/../../libs/nanopb
)
list(REMOVE_ITEM _r2bus_nanopb_search_paths "")
list(REMOVE_DUPLICATES _r2bus_nanopb_search_paths)

set(_r2bus_nanopb_generator_paths "")
foreach(_np_root IN LISTS _r2bus_nanopb_search_paths)
    if(NOT _np_root)
        continue()
    endif()
    if(IS_DIRECTORY "${_np_root}/generator")
        list(APPEND _r2bus_nanopb_generator_paths "${_np_root}/generator")
    elseif(IS_DIRECTORY "${_np_root}")
        list(APPEND _r2bus_nanopb_generator_paths "${_np_root}")
    endif()
endforeach()
list(REMOVE_DUPLICATES _r2bus_nanopb_generator_paths)

if(NOT _r2bus_nanopb_generator_paths)
    message(FATAL_ERROR "Unable to locate nanopb generator directory. Set R2BUS_NANOPB_HINTS to your nanopb checkout.")
endif()

if(NOT PROTOC_EXECUTABLE)
    find_program(PROTOC_EXECUTABLE
        NAMES protoc protoc.exe protoc.bat
        PATHS ${_r2bus_nanopb_generator_paths}
        NO_DEFAULT_PATH
    )
endif()

if(NOT NANOPB_PLUGIN)
    find_program(NANOPB_PLUGIN
        NAMES nanopb_generator.py nanopb_generator nanopb_generator.bat
        PATHS ${_r2bus_nanopb_generator_paths}
        NO_DEFAULT_PATH
    )
endif()

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

  target_sources(r2bus PRIVATE ${GENERATED_C})
else()
  message(FATAL_ERROR "protoc/nanopb generator not found; cannot build r2bus library")
endif()

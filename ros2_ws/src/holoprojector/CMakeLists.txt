cmake_minimum_required(VERSION 3.8)
project(holoprojector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(interfaces REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/holoprojector.proto)
add_library(holoprojector_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(holoprojector_proto ${Protobuf_LIBRARIES})


add_executable(holoprojector_node src/Holoprojector.cpp src/SerialInterface.cpp)
target_include_directories(holoprojector_node PUBLIC 
  include/holoprojector
  ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(holoprojector_node holoprojector_proto)
ament_target_dependencies(holoprojector_node rclcpp std_msgs interfaces)

install(TARGETS
  holoprojector_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB ROS_MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "msg/*.msg")
file(GLOB ROS_SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ROS_MSG_FILES}
  ${ROS_SRV_FILES}
  DEPENDENCIES std_msgs
)

ament_package()

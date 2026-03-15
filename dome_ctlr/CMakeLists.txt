cmake_minimum_required(VERSION 3.8)
project(dome_ctlr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_file(DOME_MOTOR_STATIC libdome_motor.a PATHS /usr/local/lib)
find_library(DOME_MOTOR_SHARED NAMES dome_motor PATHS /usr/local/lib)
if(DOME_MOTOR_STATIC)
  set(DOME_MOTOR_LIB ${DOME_MOTOR_STATIC})
else()
  set(DOME_MOTOR_LIB ${DOME_MOTOR_SHARED})
endif()
find_path(DOME_MOTOR_INCLUDE dome_motor/DomeMotor.hpp PATHS /usr/local/include)
find_library(TMC2209_LIB NAMES TMC2209 PATHS /usr/local/lib)
find_library(GPIOD_LIB NAMES gpiod)

include_directories(${DOME_MOTOR_INCLUDE})

add_executable(dome_ctlr_node src/dome_ctlr_node.cpp)
ament_target_dependencies(dome_ctlr_node rclcpp std_msgs)

# Provide a fallback TMC2209 implementation if the shared lib is not installed.
if(NOT TMC2209_LIB)
  message(WARNING "libTMC2209 not found in /usr/local/lib; building from source as fallback.")
  add_library(tmc2209_local STATIC ../dome_motor/TMC2209.cpp)
  target_include_directories(tmc2209_local PUBLIC ../dome_motor)
  set(TMC2209_LIB tmc2209_local)
endif()

target_link_libraries(dome_ctlr_node ${DOME_MOTOR_LIB} ${TMC2209_LIB} ${GPIOD_LIB})
set_target_properties(dome_ctlr_node PROPERTIES
  BUILD_RPATH "/usr/local/lib"
  INSTALL_RPATH "/usr/local/lib"
  SKIP_BUILD_RPATH FALSE
)

install(TARGETS
  dome_ctlr_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

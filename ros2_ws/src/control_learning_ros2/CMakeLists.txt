cmake_minimum_required(VERSION 3.20)
project(control_learning_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(fmt REQUIRED)
find_package(casadi REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/cpp/robot/dynamics/include
)

# Build the robot dynamics library
add_library(dynamics
  ../../../src/cpp/robot/dynamics/model.cpp
)
target_link_libraries(dynamics
  casadi
)

# Build the ROS2 simulation node
add_executable(robot_simulator_node
  src/robot_simulator_node.cpp
)

ament_target_dependencies(robot_simulator_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

target_link_libraries(robot_simulator_node
  dynamics
  fmt::fmt
  casadi
)

# Build the example simple controller (optional)
add_executable(simple_controller
  examples/simple_controller.cpp
)

ament_target_dependencies(simple_controller
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS
  robot_simulator_node
  simple_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install example files
install(PROGRAMS
  examples/square_pattern.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

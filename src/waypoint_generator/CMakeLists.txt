cmake_minimum_required(VERSION 3.5)
project(waypoint_generator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE
  ament_cmake
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros)
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executables
add_executable(waypoint_generator src/waypoint_generator.cpp)
add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)  # 추가됨

ament_target_dependencies(waypoint_generator
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

ament_target_dependencies(waypoint_visualizer  # 추가됨
  rclcpp geometry_msgs nav_msgs visualization_msgs
)

# Install Cpp executables
install(TARGETS
  waypoint_generator
  waypoint_visualizer  # 추가됨
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


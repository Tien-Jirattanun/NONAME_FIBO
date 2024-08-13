cmake_minimum_required(VERSION 3.8)
project(robot_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Include directories
include_directories(
  include
  include/robot_control_cpp/kinematics_calculation
)

# Add executable
add_executable(
  kinematics_publisher 
  src/kinematics_publisher.cpp 
  src/kinematics_calculation/kinematics_calculation.cpp
)

# Link dependencies
ament_target_dependencies(
  kinematics_publisher
  rclcpp
  std_msgs
  geometry_msgs
  tf2_msgs
)

# Install targets
install(TARGETS
  kinematics_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
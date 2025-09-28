cmake_minimum_required(VERSION 3.8)
project(calm_man_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_msg REQUIRED)  # Make sure your custom_msg package is built

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Executable
add_executable(kalman_node src/calm_man_node.cpp)

# Link libraries
ament_target_dependencies(kalman_node
  rclcpp
  std_msgs
  custom_msg
  Eigen3
)

# Install
install(TARGETS kalman_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


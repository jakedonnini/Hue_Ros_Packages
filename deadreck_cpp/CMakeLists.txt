cmake_minimum_required(VERSION 3.8)
project(deadreck_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(deadreck_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)

add_executable(dead_reckoning_node src/deadreck_node.cpp)
ament_target_dependencies(dead_reckoning_node rclcpp std_msgs custom_msg)

install(TARGETS
  dead_reckoning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

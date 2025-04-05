cmake_minimum_required(VERSION 3.8)
project(navigator_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# Create the executable
add_executable(gps_navigation_node src/gps_navigation_node.cpp)
target_include_directories(gps_navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gps_navigation_node PUBLIC c_std_99 cxx_std_17)  # Require C++17
ament_target_dependencies(gps_navigation_node
  rclcpp
  custom_msg
  std_msgs)

target_link_libraries(gps_navigation_node
  ${CMAKE_THREAD_LIBS_INIT})

# Install the executable
install(TARGETS gps_navigation_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files if you have any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*.py"
)

# Export information to downstream packages
ament_package()

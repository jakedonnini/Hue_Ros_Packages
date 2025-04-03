cmake_minimum_required(VERSION 3.8)
project(gps_publisher_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

add_executable(gps_publisher src/gps1_node.cpp)
ament_target_dependencies(gps_publisher
  rclcpp
  std_msgs
  custom_msg
)

target_include_directories(gps_publisher PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(gps_publisher ${LIBSERIALPORT_LIBRARIES})

install(TARGETS gps_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()


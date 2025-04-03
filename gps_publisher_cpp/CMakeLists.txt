cmake_minimum_required(VERSION 3.8)
project(gps_publisher_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

add_executable(gps_pub1 src/gps1_node.cpp)
ament_target_dependencies(gps_pub1
  rclcpp
  std_msgs
  custom_msg
)

target_include_directories(gps_pub1 PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(gps_pub1 ${LIBSERIALPORT_LIBRARIES})

add_executable(gps_pub2 src/gps2_node.cpp)
ament_target_dependencies(gps_pub2
  rclcpp
  std_msgs
  custom_msg
)

target_include_directories(gps_pub2 PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(gps_pub2 ${LIBSERIALPORT_LIBRARIES})

add_executable(gps_fus src/gps_fusion_node.cpp)
ament_target_dependencies(gps_fus
  rclcpp
  std_msgs
  custom_msg
  Eigen3
  message_filters
)

target_include_directories(gps_fus PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(gps_fus ${LIBSERIALPORT_LIBRARIES})

install(TARGETS gps_pub1 gps_pub2 gps_pub3
  DESTINATION lib/${PROJECT_NAME})

ament_package()

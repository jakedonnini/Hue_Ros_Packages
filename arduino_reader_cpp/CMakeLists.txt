cmake_minimum_required(VERSION 3.8)
project(arduino_reader_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#include cpp "include" dir
include_directories(include)

#create cpp exec
add_executable(ard_read_exe src/arduino_serial_node.cpp)
ament_target_dependencies(ard_read_exe rclcpp)
ament_target_dependencies(ard_read_exe custom_msg)

target_include_directories(ard_read_exe PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(ard_read_exe ${LIBSERIALPORT_LIBRARIES})

# install cpp execs
install(TARGETS
  ard_read_exe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(my_cpp_py_pkg)

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
add_executable(cpp_exe src/cpp_node.cpp)
ament_target_dependencies(cpp_exe rclcpp)
ament_target_dependencies(cpp_exe custom_msg)

target_include_directories(cpp_exe PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(cpp_exe ${LIBSERIALPORT_LIBRARIES})

# install cpp execs
install(TARGETS
  cpp_exe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

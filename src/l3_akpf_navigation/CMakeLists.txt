cmake_minimum_required(VERSION 3.8)
project(l3_akpf_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(l3_akpf_node src/l3_akpf_node.cpp)
target_compile_features(l3_akpf_node PUBLIC cxx_std_17)
ament_target_dependencies(
  l3_akpf_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
)

install(TARGETS l3_akpf_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

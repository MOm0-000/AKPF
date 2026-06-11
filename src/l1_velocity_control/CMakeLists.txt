cmake_minimum_required(VERSION 3.8)
project(l1_velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(l1_velocity_node src/l1_velocity_node.cpp)
target_compile_features(l1_velocity_node PUBLIC cxx_std_17)
ament_target_dependencies(
  l1_velocity_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
)

install(TARGETS l1_velocity_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(l5_safety_shield)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(l5_safety_shield_node src/l5_safety_shield_node.cpp)
target_compile_features(l5_safety_shield_node PUBLIC cxx_std_17)
ament_target_dependencies(
  l5_safety_shield_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  std_msgs
)

install(TARGETS l5_safety_shield_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

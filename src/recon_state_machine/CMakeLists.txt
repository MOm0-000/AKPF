cmake_minimum_required(VERSION 3.8)
project(recon_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(recon_state_machine_cpp src/recon_state_machine_cpp.cpp)
target_compile_features(recon_state_machine_cpp PUBLIC cxx_std_17)
ament_target_dependencies(
  recon_state_machine_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
)

install(
  TARGETS recon_state_machine_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

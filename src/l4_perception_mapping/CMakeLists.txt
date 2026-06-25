cmake_minimum_required(VERSION 3.8)
project(l4_perception_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)

add_executable(l4_pointcloud_mapper_node src/l4_pointcloud_mapper_node.cpp)
target_compile_features(l4_pointcloud_mapper_node PUBLIC cxx_std_17)
ament_target_dependencies(
  l4_pointcloud_mapper_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(l4_synthetic_cloud_node src/l4_synthetic_cloud_node.cpp)
target_compile_features(l4_synthetic_cloud_node PUBLIC cxx_std_17)
ament_target_dependencies(
  l4_synthetic_cloud_node
  rclcpp
  sensor_msgs
)

add_executable(l4_gz_pointcloud_bridge_node src/l4_gz_pointcloud_bridge_node.cpp)
target_compile_features(l4_gz_pointcloud_bridge_node PUBLIC cxx_std_17)
target_include_directories(l4_gz_pointcloud_bridge_node PRIVATE
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)
target_link_directories(l4_gz_pointcloud_bridge_node PRIVATE
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)
target_link_libraries(l4_gz_pointcloud_bridge_node
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)
target_compile_options(l4_gz_pointcloud_bridge_node PRIVATE
  ${GZ_TRANSPORT_CFLAGS_OTHER}
  ${GZ_MSGS_CFLAGS_OTHER}
)
ament_target_dependencies(
  l4_gz_pointcloud_bridge_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  l4_pointcloud_mapper_node
  l4_synthetic_cloud_node
  l4_gz_pointcloud_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

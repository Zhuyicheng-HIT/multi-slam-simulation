cmake_minimum_required(VERSION 3.8)
project(mid360_reliable_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(fastlio_cloud_mapper_node src/fastlio_cloud_mapper_node.cpp)
ament_target_dependencies(
  fastlio_cloud_mapper_node
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(pointcloud_occupancy_grid_node src/pointcloud_occupancy_grid_node.cpp)
ament_target_dependencies(
  pointcloud_occupancy_grid_node
  nav_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(TARGETS
  fastlio_cloud_mapper_node
  pointcloud_occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

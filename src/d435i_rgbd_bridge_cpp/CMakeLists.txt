
cmake_minimum_required(VERSION 3.8)
project(d435i_rgbd_bridge_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(d435i_rgbd_bridge src/d435i_rgbd_bridge.cpp)
target_compile_features(d435i_rgbd_bridge PUBLIC cxx_std_17)
ament_target_dependencies(d435i_rgbd_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(d435i_rgbd_bridge
  gz-msgs10::core
  gz-transport13::core
)

add_executable(gz_rgbd_rate_probe src/gz_rgbd_rate_probe.cpp)
target_compile_features(gz_rgbd_rate_probe PUBLIC cxx_std_17)
target_link_libraries(gz_rgbd_rate_probe
  gz-msgs10::core
  gz-transport13::core
)

add_executable(ros_rgbd_rate_probe src/ros_rgbd_rate_probe.cpp)
target_compile_features(ros_rgbd_rate_probe PUBLIC cxx_std_17)
ament_target_dependencies(ros_rgbd_rate_probe
  rclcpp
  sensor_msgs
)

install(TARGETS
  d435i_rgbd_bridge
  gz_rgbd_rate_probe
  ros_rgbd_rate_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


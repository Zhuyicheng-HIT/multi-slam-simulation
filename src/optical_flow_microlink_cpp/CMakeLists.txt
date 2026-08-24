cmake_minimum_required(VERSION 3.8)
project(optical_flow_microlink_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(gz_microlink_flow_bridge src/gz_microlink_flow_bridge.cpp)
target_compile_features(gz_microlink_flow_bridge PUBLIC cxx_std_17)
ament_target_dependencies(gz_microlink_flow_bridge
  mavros_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(gz_microlink_flow_bridge PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gz_microlink_flow_bridge
  ${OpenCV_LIBRARIES}
  gz-msgs10::core
  gz-transport13::core
)

install(TARGETS gz_microlink_flow_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(mid360_sim_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gz_livox_bridge_node src/gz_livox_bridge_node.cpp)
target_compile_features(gz_livox_bridge_node PUBLIC cxx_std_17)
target_include_directories(gz_livox_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  gz_livox_bridge_node
  livox_ros_driver2
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(
  gz_livox_bridge_node
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(TARGETS gz_livox_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversion test/test_conversion.cpp)
  if(TARGET test_conversion)
    target_compile_features(test_conversion PUBLIC cxx_std_17)
    target_include_directories(test_conversion PUBLIC include)
  endif()
endif()

ament_export_include_directories(include)
ament_package()

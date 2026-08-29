cmake_minimum_required(VERSION 3.8)
project(uf_pointcloud_body_filter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(pointcloud_filter_core src/pointcloud_filter.cpp)
target_compile_features(pointcloud_filter_core PUBLIC cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(pointcloud_filter_core PRIVATE -O3)
endif()
target_include_directories(pointcloud_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointcloud_filter_core sensor_msgs)

add_executable(pointcloud_body_filter_cpp src/pointcloud_body_filter_node.cpp)
target_compile_features(pointcloud_body_filter_cpp PUBLIC cxx_std_17)
target_include_directories(pointcloud_body_filter_cpp PRIVATE include)
ament_target_dependencies(pointcloud_body_filter_cpp rclcpp sensor_msgs std_msgs)
target_link_libraries(pointcloud_body_filter_cpp pointcloud_filter_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_pointcloud_filter test/test_pointcloud_filter.cpp)
  if(TARGET test_pointcloud_filter)
    target_compile_features(test_pointcloud_filter PUBLIC cxx_std_17)
    target_include_directories(test_pointcloud_filter PRIVATE include)
    ament_target_dependencies(test_pointcloud_filter sensor_msgs)
    target_link_libraries(test_pointcloud_filter pointcloud_filter_core)
  endif()
  ament_add_pytest_test(test_body_filter_node test/test_body_filter_node.py TIMEOUT 30)
endif()

install(TARGETS pointcloud_filter_core pointcloud_body_filter_cpp
  EXPORT export_pointcloud_filter_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_pointcloud_filter_core HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs)

ament_package()

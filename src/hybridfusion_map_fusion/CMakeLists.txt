cmake_minimum_required(VERSION 3.8)
project(hybridfusion_map_fusion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree registration search)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(hybridfusion_core src/hybridfusion_core.cpp)
target_compile_features(hybridfusion_core PUBLIC cxx_std_17)
target_include_directories(hybridfusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(hybridfusion_core PUBLIC ${PCL_LIBRARIES} yaml-cpp)
target_compile_definitions(hybridfusion_core PUBLIC ${PCL_DEFINITIONS})

add_executable(hybridfusion_offline src/hybridfusion_offline.cpp)
target_compile_features(hybridfusion_offline PUBLIC cxx_std_17)
target_link_libraries(hybridfusion_offline hybridfusion_core)

add_executable(rgbd_map_exporter src/rgbd_map_exporter.cpp)
target_compile_features(rgbd_map_exporter PUBLIC cxx_std_17)
target_include_directories(rgbd_map_exporter PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(rgbd_map_exporter
  geometry_msgs
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_eigen
  tf2_ros
)
target_link_libraries(rgbd_map_exporter ${PCL_LIBRARIES})

add_executable(lidar_map_exporter src/lidar_map_exporter.cpp)
target_compile_features(lidar_map_exporter PUBLIC cxx_std_17)
target_include_directories(lidar_map_exporter PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(lidar_map_exporter
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(lidar_map_exporter ${PCL_LIBRARIES})

install(TARGETS
  hybridfusion_core
  hybridfusion_offline
  rgbd_map_exporter
  lidar_map_exporter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/data DESTINATION share/${PROJECT_NAME}/test)
install(PROGRAMS
  scripts/collect_hybridfusion_simulation.sh
  scripts/generate_hybridfusion_dataset.py
  scripts/evaluate_hybridfusion_runs.py
  scripts/run_hybridfusion_benchmark.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_hybridfusion_core test/test_hybridfusion_core.cpp)
  target_link_libraries(test_hybridfusion_core hybridfusion_core)
  ament_add_pytest_test(
    test_dataset_generator test/test_dataset_generator.py
    TIMEOUT 90
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(
    test_evaluator test/test_evaluator.py
    TIMEOUT 30
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(hybridfusion_core)
ament_package()

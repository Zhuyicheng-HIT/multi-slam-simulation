cmake_minimum_required(VERSION 3.16)
project(uf_safety_supervisor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uf_interfaces REQUIRED)

add_library(safety_supervisor_core
  src/obstacle_safety_core.cpp
  src/command_arbiter_core.cpp
  src/local_avoidance_core.cpp)
target_compile_features(safety_supervisor_core PUBLIC cxx_std_17)
target_include_directories(safety_supervisor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(safety_supervisor_core PUBLIC Eigen3::Eigen)

add_executable(raw_obstacle_safety_monitor src/raw_obstacle_safety_monitor.cpp)
target_compile_features(raw_obstacle_safety_monitor PUBLIC cxx_std_17)
target_link_libraries(raw_obstacle_safety_monitor safety_supervisor_core Eigen3::Eigen)
ament_target_dependencies(raw_obstacle_safety_monitor
  diagnostic_msgs geometry_msgs livox_ros_driver2 nav_msgs rclcpp std_msgs uf_interfaces)

add_executable(flight_command_arbiter src/flight_command_arbiter.cpp)
target_compile_features(flight_command_arbiter PUBLIC cxx_std_17)
target_link_libraries(flight_command_arbiter safety_supervisor_core Eigen3::Eigen)
ament_target_dependencies(flight_command_arbiter
  diagnostic_msgs geometry_msgs mavros_msgs rclcpp std_msgs uf_interfaces)

add_executable(local_avoidance_planner src/local_avoidance_planner.cpp)
target_compile_features(local_avoidance_planner PUBLIC cxx_std_17)
target_link_libraries(local_avoidance_planner safety_supervisor_core Eigen3::Eigen)
ament_target_dependencies(local_avoidance_planner
  geometry_msgs livox_ros_driver2 nav_msgs rclcpp std_msgs uf_interfaces)

add_executable(safety_runtime_benchmark tools/safety_runtime_benchmark.cpp)
target_compile_features(safety_runtime_benchmark PUBLIC cxx_std_17)
target_link_libraries(safety_runtime_benchmark safety_supervisor_core Eigen3::Eigen)

add_executable(local_avoidance_benchmark tools/local_avoidance_benchmark.cpp)
target_compile_features(local_avoidance_benchmark PUBLIC cxx_std_17)
target_link_libraries(local_avoidance_benchmark safety_supervisor_core Eigen3::Eigen)

add_executable(nav_avoidance_scenario_matrix tools/nav_avoidance_scenario_matrix.cpp)
target_compile_features(nav_avoidance_scenario_matrix PUBLIC cxx_std_17)
target_link_libraries(nav_avoidance_scenario_matrix safety_supervisor_core Eigen3::Eigen)

install(TARGETS safety_supervisor_core EXPORT export_safety_supervisor_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS raw_obstacle_safety_monitor flight_command_arbiter local_avoidance_planner
  safety_runtime_benchmark local_avoidance_benchmark
  nav_avoidance_scenario_matrix
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  tools/safety_slice_ros_smoke.py
  tools/local_avoidance_ros_smoke.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_obstacle_safety_core test/test_obstacle_safety_core.cpp)
  target_link_libraries(test_obstacle_safety_core safety_supervisor_core Eigen3::Eigen)
  ament_add_gtest(test_command_arbiter_core test/test_command_arbiter_core.cpp)
  target_link_libraries(test_command_arbiter_core safety_supervisor_core Eigen3::Eigen)
  ament_add_gtest(test_local_avoidance_core test/test_local_avoidance_core.cpp)
  target_link_libraries(test_local_avoidance_core safety_supervisor_core Eigen3::Eigen)
  ament_add_pytest_test(test_command_ownership test/test_command_ownership.py)
endif()

ament_export_targets(export_safety_supervisor_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

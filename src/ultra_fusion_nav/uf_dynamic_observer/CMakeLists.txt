cmake_minimum_required(VERSION 3.16)
project(uf_dynamic_observer)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uf_dynamic_interfaces REQUIRED)

add_library(dynamic_observer_core
  src/causal_imu_deskew.cpp
  src/clean_scan_admission.cpp
  src/conservative_free_space.cpp)
target_compile_features(dynamic_observer_core PUBLIC cxx_std_17)
target_include_directories(dynamic_observer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dynamic_observer_core PUBLIC Eigen3::Eigen)

add_executable(dynamic_observer_node src/dynamic_observer_node.cpp)
target_compile_features(dynamic_observer_node PUBLIC cxx_std_17)
target_include_directories(dynamic_observer_node PRIVATE include)
target_link_libraries(dynamic_observer_node dynamic_observer_core Eigen3::Eigen)
ament_target_dependencies(dynamic_observer_node
  diagnostic_msgs geometry_msgs livox_ros_driver2 nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(clean_scan_gateway_node src/clean_scan_gateway_node.cpp)
target_compile_features(clean_scan_gateway_node PUBLIC cxx_std_17)
target_include_directories(clean_scan_gateway_node PRIVATE include)
target_link_libraries(clean_scan_gateway_node dynamic_observer_core Eigen3::Eigen)
ament_target_dependencies(clean_scan_gateway_node
  diagnostic_msgs livox_ros_driver2 rclcpp sensor_msgs std_msgs uf_dynamic_interfaces)

add_executable(dynamic_observer_benchmark src/dynamic_observer_benchmark.cpp)
target_compile_features(dynamic_observer_benchmark PUBLIC cxx_std_17)
target_include_directories(dynamic_observer_benchmark PRIVATE include)
target_link_libraries(dynamic_observer_benchmark dynamic_observer_core)

install(TARGETS dynamic_observer_core
  EXPORT export_dynamic_observer_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS dynamic_observer_node dynamic_observer_benchmark clean_scan_gateway_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  tools/analyze_clean_gateway_ab_replay.py
  tools/clean_gateway_ros_smoke.py
  tools/dynamic_observer_ros_smoke.py
  tools/evaluate_dynamic_observer_csv.py
  tools/fast_lio_latency_probe.py
  tools/generate_clean_gateway_frozen_replay.py
  tools/run_clean_gateway_ab_replay.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_conservative_free_space test/test_conservative_free_space.cpp)
  if(TARGET test_conservative_free_space)
    target_compile_features(test_conservative_free_space PUBLIC cxx_std_17)
    target_include_directories(test_conservative_free_space PRIVATE include)
    target_link_libraries(test_conservative_free_space dynamic_observer_core)
  endif()
  ament_add_pytest_test(test_evaluator_contract test/test_evaluator_contract.py)
endif()

ament_export_targets(export_dynamic_observer_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(uf_relocalization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uf_dynamic_observer REQUIRED)
find_package(uf_interfaces REQUIRED)

add_library(active_relocalization_policy src/active_relocalization_policy.cpp)
target_compile_features(active_relocalization_policy PUBLIC cxx_std_17)
target_include_directories(active_relocalization_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(automatic_loop_closure_policy src/automatic_loop_closure_policy.cpp)
target_compile_features(automatic_loop_closure_policy PUBLIC cxx_std_17)
target_include_directories(automatic_loop_closure_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(registration_core src/registration_core.cpp)
target_compile_features(registration_core PUBLIC cxx_std_17)
target_include_directories(registration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(registration_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(registration_core PUBLIC ${PCL_DEFINITIONS})

add_library(multi_frame_consistency_gate src/multi_frame_consistency_gate.cpp)
target_compile_features(multi_frame_consistency_gate PUBLIC cxx_std_17)
target_include_directories(multi_frame_consistency_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(multi_frame_consistency_gate PUBLIC Eigen3::Eigen)

add_library(keyframe_synchronization src/keyframe_synchronization.cpp)
target_compile_features(keyframe_synchronization PUBLIC cxx_std_17)
target_include_directories(keyframe_synchronization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(keyframe_database src/keyframe_database.cpp)
target_compile_features(keyframe_database PUBLIC cxx_std_17)
target_include_directories(keyframe_database PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(keyframe_database PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(keyframe_database PUBLIC ${PCL_DEFINITIONS})

add_library(descriptor_core src/descriptor_core.cpp)
target_compile_features(descriptor_core PUBLIC cxx_std_17)
target_include_directories(descriptor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(descriptor_core PUBLIC ${PCL_LIBRARIES})
target_compile_definitions(descriptor_core PUBLIC ${PCL_DEFINITIONS})

add_executable(relocalization_node src/relocalization_node.cpp)
target_compile_features(relocalization_node PUBLIC cxx_std_17)
target_include_directories(relocalization_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(relocalization_node
  registration_core keyframe_database descriptor_core multi_frame_consistency_gate
  keyframe_synchronization automatic_loop_closure_policy
  Eigen3::Eigen ${PCL_LIBRARIES}
)
ament_target_dependencies(relocalization_node
  ament_index_cpp geometry_msgs nav_msgs pcl_conversions rclcpp
  sensor_msgs std_msgs uf_interfaces
)
target_compile_definitions(relocalization_node PRIVATE ${PCL_DEFINITIONS})

add_executable(lidar_calibration_motion_node src/lidar_calibration_motion_node.cpp)
target_compile_features(lidar_calibration_motion_node PUBLIC cxx_std_17)
target_include_directories(lidar_calibration_motion_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(lidar_calibration_motion_node
  registration_core Eigen3::Eigen ${PCL_LIBRARIES}
)
ament_target_dependencies(lidar_calibration_motion_node
  livox_ros_driver2 rclcpp uf_interfaces
)
target_compile_definitions(lidar_calibration_motion_node PRIVATE ${PCL_DEFINITIONS})

add_executable(dynamic_localization_benchmark src/dynamic_localization_benchmark.cpp)
target_compile_features(dynamic_localization_benchmark PUBLIC cxx_std_17)
target_include_directories(dynamic_localization_benchmark PRIVATE
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(dynamic_localization_benchmark
  registration_core keyframe_database descriptor_core
  uf_dynamic_observer::dynamic_observer_core Eigen3::Eigen ${PCL_LIBRARIES}
)
target_compile_definitions(dynamic_localization_benchmark PRIVATE ${PCL_DEFINITIONS})

install(TARGETS active_relocalization_policy automatic_loop_closure_policy
  registration_core keyframe_database descriptor_core
  multi_frame_consistency_gate keyframe_synchronization
  EXPORT export_registration_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS relocalization_node lidar_calibration_motion_node
  dynamic_localization_benchmark
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_active_relocalization_policy test/test_active_relocalization_policy.cpp)
  target_link_libraries(
    test_active_relocalization_policy active_relocalization_policy)
  ament_add_gtest(
    test_automatic_loop_closure_policy test/test_automatic_loop_closure_policy.cpp)
  target_link_libraries(
    test_automatic_loop_closure_policy automatic_loop_closure_policy)
  ament_add_gtest(test_registration_core test/test_registration_core.cpp)
  target_link_libraries(test_registration_core registration_core)
  ament_add_gtest(
    test_multi_frame_consistency_gate test/test_multi_frame_consistency_gate.cpp)
  target_link_libraries(
    test_multi_frame_consistency_gate multi_frame_consistency_gate)
  ament_add_gtest(
    test_keyframe_synchronization test/test_keyframe_synchronization.cpp)
  target_link_libraries(test_keyframe_synchronization keyframe_synchronization)
  ament_add_gtest(test_keyframe_database test/test_keyframe_database.cpp)
  target_link_libraries(test_keyframe_database keyframe_database)
  ament_add_gtest(test_relocalization_pipeline test/test_relocalization_pipeline.cpp)
  target_link_libraries(
    test_relocalization_pipeline registration_core keyframe_database)
  ament_add_gtest(test_descriptor_core test/test_descriptor_core.cpp)
  target_link_libraries(test_descriptor_core descriptor_core keyframe_database)
endif()

ament_export_targets(export_registration_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()

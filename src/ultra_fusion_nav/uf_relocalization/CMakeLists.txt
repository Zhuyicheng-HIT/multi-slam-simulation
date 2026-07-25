cmake_minimum_required(VERSION 3.8)
project(uf_relocalization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features registration)

add_library(registration_core src/registration_core.cpp)
target_compile_features(registration_core PUBLIC cxx_std_17)
target_include_directories(registration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(registration_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(registration_core PUBLIC ${PCL_DEFINITIONS})

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

install(TARGETS registration_core keyframe_database descriptor_core
  EXPORT export_registration_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration_core test/test_registration_core.cpp)
  target_link_libraries(test_registration_core registration_core)
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

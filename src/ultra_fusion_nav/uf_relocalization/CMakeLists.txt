cmake_minimum_required(VERSION 3.8)
project(uf_relocalization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration)

add_library(registration_core src/registration_core.cpp)
target_compile_features(registration_core PUBLIC cxx_std_17)
target_include_directories(registration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(registration_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(registration_core PUBLIC ${PCL_DEFINITIONS})

install(TARGETS registration_core
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
endif()

ament_export_targets(export_registration_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()

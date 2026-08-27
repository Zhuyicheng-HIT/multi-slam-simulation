cmake_minimum_required(VERSION 3.8)
project(uf_map_maintenance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tools/archive_mid360_session
  tools/materialize_mid360_archive
  tools/offline_map_builder
  tools/validate_map_archive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_maintenance test)
endif()

ament_package()

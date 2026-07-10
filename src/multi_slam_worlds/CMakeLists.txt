cmake_minimum_required(VERSION 3.8)
project(multi_slam_worlds)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds models config scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
ament_package()

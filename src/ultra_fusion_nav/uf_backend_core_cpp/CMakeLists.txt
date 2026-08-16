cmake_minimum_required(VERSION 3.16)
project(uf_backend_core_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_core MODULE src/bindings.cpp)
target_link_libraries(_core PRIVATE Eigen3::Eigen)
target_compile_features(_core PRIVATE cxx_std_17)
target_compile_options(_core PRIVATE -O3)

install(
  TARGETS _core
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()

cmake_minimum_required(VERSION 3.16)
project(motoman_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# ---------------- YMConnect (vendored binary SDK) ----------------

set(YMCONNECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YMConnect)

add_library(YMConnect SHARED IMPORTED)
set_target_properties(YMConnect PROPERTIES
  IMPORTED_LOCATION ${YMCONNECT_ROOT}/libYMConnect.so
  INTERFACE_INCLUDE_DIRECTORIES ${YMCONNECT_ROOT}
)

# ---------------- pybind module ----------------

pybind11_add_module(motoman_cpp
  src/MotomanRobotBindings.cpp
  src/MotomanRobot.cpp
)

target_include_directories(motoman_cpp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(motoman_cpp PRIVATE YMConnect)

# libYMConnect.so is installed next to the module; $ORIGIN resolves it
# in-place (same trick as the legacy build).
set_target_properties(motoman_cpp PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN"
)

# ---------------- install into site-packages ----------------

# Sets PYTHON_INSTALL_DIR (e.g. lib/python3.10/site-packages) so the module
# is importable once the workspace is sourced.
_ament_cmake_python_get_python_install_dir()

install(TARGETS motoman_cpp
  DESTINATION ${PYTHON_INSTALL_DIR}
)

install(FILES ${YMCONNECT_ROOT}/libYMConnect.so
  DESTINATION ${PYTHON_INSTALL_DIR}
)

ament_package()

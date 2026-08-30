cmake_minimum_required(VERSION 3.16)
project(cobot_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cobot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------- YMConnect (vendored binary SDK) ----------------
# Moved here from motoman_cpp/third_party (git history preserved).

set(YMCONNECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YMConnect)

add_library(YMConnect SHARED IMPORTED)
set_target_properties(YMConnect PROPERTIES
  IMPORTED_LOCATION ${YMCONNECT_ROOT}/libYMConnect.so
  INTERFACE_INCLUDE_DIRECTORIES ${YMCONNECT_ROOT}
)

# ---------------- SDK wrapper: no ROS dependencies ----------------

add_library(motoman_robot STATIC src/motoman_robot.cpp)
target_include_directories(motoman_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(motoman_robot PUBLIC YMConnect)

add_library(setup_config STATIC src/setup_config.cpp)
target_link_libraries(setup_config PUBLIC motoman_robot yaml-cpp)

# ---------------- node ----------------

add_executable(cobot_control_node src/cobot_control_node.cpp src/main.cpp)
target_link_libraries(cobot_control_node motoman_robot setup_config)
ament_target_dependencies(cobot_control_node
  rclcpp rclcpp_action std_msgs std_srvs cobot_interfaces
)

# libYMConnect.so is installed next to the executable; $ORIGIN resolves it
# in place (same trick the pybind build used). Link-path rpaths keep the
# ROS libraries resolvable too.
set_target_properties(cobot_control_node PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS cobot_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${YMCONNECT_ROOT}/libYMConnect.so
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

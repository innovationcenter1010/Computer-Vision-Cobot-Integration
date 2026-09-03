cmake_minimum_required(VERSION 3.16)
project(cobot_detector LANGUAGES CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cobot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ---------------- ONNX Runtime (official release tarball) ----------------
# Not in apt / rosdep. docker/Dockerfile unpacks
# onnxruntime-linux-x64-${ORT_VERSION}.tgz to /opt/onnxruntime; for a local
# build pass -DONNXRUNTIME_ROOT=<dir> (colcon build --cmake-args ...).

set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "Unpacked onnxruntime release directory")

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include
  NO_DEFAULT_PATH)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib
  NO_DEFAULT_PATH)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found under ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT} "
    "(need include/onnxruntime_cxx_api.h and lib/libonnxruntime.so). "
    "Download https://github.com/microsoft/onnxruntime/releases and pass "
    "-DONNXRUNTIME_ROOT=<dir>.")
endif()

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION ${ONNXRUNTIME_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES ${ONNXRUNTIME_INCLUDE_DIR}
)

# ---------------- core: no ROS dependencies ----------------

add_library(detector_core STATIC
  src/model_registry.cpp
  src/letterbox.cpp
  src/postprocess.cpp
  src/onnx_detector.cpp
)
target_include_directories(detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(detector_core PUBLIC
  onnxruntime
  yaml-cpp
  opencv_core
  opencv_imgproc
)

# ---------------- node ----------------

add_executable(detector_node src/detector_node.cpp src/main.cpp)
target_link_libraries(detector_node detector_core opencv_imgcodecs)  # imgcodecs: image_source goals (cv::imread)
ament_target_dependencies(detector_node
  rclcpp rclcpp_action sensor_msgs std_msgs cobot_interfaces
)

# libonnxruntime.so* is installed next to the executable; $ORIGIN resolves it
# in place (same trick cobot_control uses for libYMConnect.so).
set_target_properties(detector_node PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS detector_node
  DESTINATION lib/${PROJECT_NAME}
)
# The release tarball ships libonnxruntime.so -> libonnxruntime.so.X.Y.Z;
# copy every variant so the symlink resolves after install.
file(GLOB ONNXRUNTIME_RUNTIME_LIBS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so*")
install(FILES ${ONNXRUNTIME_RUNTIME_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ---------------- tests ----------------

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_letterbox test/test_letterbox.cpp)
  target_link_libraries(test_letterbox detector_core)

  ament_add_gtest(test_postprocess test/test_postprocess.cpp)
  target_link_libraries(test_postprocess detector_core)

  ament_add_gtest(test_model_registry test/test_model_registry.cpp)
  target_link_libraries(test_model_registry detector_core)
  target_compile_definitions(test_model_registry PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

  # Runs the real exported models on real frames and compares against the
  # JSON the old ultralytics-based node produced (tools/dump_golden.py).
  # Skips itself when the .onnx / golden files are absent (e.g. CI without
  # the vision_workspace mount).
  ament_add_gtest(test_golden test/test_golden.cpp)
  target_link_libraries(test_golden detector_core opencv_imgcodecs)
  target_compile_definitions(test_golden PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()

set(YMCONNECT_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YMConnect
)

add_library(YMConnect SHARED IMPORTED)
set_target_properties(YMConnect PROPERTIES
    IMPORTED_LOCATION ${YMCONNECT_ROOT}/libYMConnect.so
    INTERFACE_INCLUDE_DIRECTORIES ${YMCONNECT_ROOT}
)

add_library(robot_core STATIC
    src/MotomanRobot.cpp
)

add_subdirectory(third_party/pybind)

target_include_directories(robot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YMCONNECT_ROOT}
)

target_link_libraries(robot_core PUBLIC YMConnect)

pybind11_add_module(motoman_cpp
    src/bindings.cpp
)

target_include_directories(motoman_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motoman_cpp PRIVATE robot_core)

set_target_properties(motoman_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/cobot_application/cobot
)
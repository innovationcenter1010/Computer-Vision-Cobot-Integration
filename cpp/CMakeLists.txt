set(YMCONNECT_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YMConnect
)

add_library(YMConnect SHARED IMPORTED)
set_target_properties(YMConnect PROPERTIES
    IMPORTED_LOCATION ${YMCONNECT_ROOT}/libYMConnect.so
    INTERFACE_INCLUDE_DIRECTORIES ${YMCONNECT_ROOT}
)


# ------------------- Core libraries ----------------

add_library(robot_core STATIC
    src/MotomanRobot.cpp
)

add_library(astra_core STATIC
    src/AstraCamera.cpp
)

add_subdirectory(third_party/pybind)


# ---------------------- Includes --------------

target_include_directories(robot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YMCONNECT_ROOT}
)

set(ASTRA_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/AstraSDK
)


add_library(AstraSDK SHARED IMPORTED)


set_target_properties(AstraSDK PROPERTIES

    IMPORTED_LOCATION
        ${ASTRA_ROOT}/lib/libastra.so

    INTERFACE_INCLUDE_DIRECTORIES
        ${ASTRA_ROOT}/include
)


# ------------- linking --------


target_include_directories(astra_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


target_link_libraries(astra_core PUBLIC
    AstraSDK
)

#----------

target_link_libraries(robot_core PUBLIC YMConnect)

# ------------------ pybind ----------------------

pybind11_add_module(motoman_cpp
    src/MotomanRobotBindings.cpp
)

target_include_directories(motoman_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motoman_cpp PRIVATE robot_core)

pybind11_add_module(astra_cpp
    src/AstraCameraBindings.cpp
)

target_link_libraries(astra_cpp PRIVATE
    astra_core
)

# ------------------- output pybind modules to proper python appication packages -----

set_target_properties(motoman_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/cobot_application/cobot
)

set_target_properties(astra_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/cobot_application/camera
)


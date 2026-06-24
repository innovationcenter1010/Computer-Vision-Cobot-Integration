set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

set(CMAKE_INSTALL_RPATH
    "$ORIGIN"
)

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

#add_library(astra_core STATIC src/AstraCamera.cpp)

add_subdirectory(third_party/pybind)


# ---------------------- Includes --------------

target_include_directories(robot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YMCONNECT_ROOT}
)

set(ASTRA_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/AstraSDK
)

add_library(astra SHARED IMPORTED)
set_target_properties(astra PROPERTIES
    IMPORTED_LOCATION
        ${ASTRA_ROOT}/lib/libastra.so
    INTERFACE_INCLUDE_DIRECTORIES
        ${ASTRA_ROOT}/include
)


add_library(astra_core_api SHARED IMPORTED)
set_target_properties(astra_core_api PROPERTIES
    IMPORTED_LOCATION
        ${ASTRA_ROOT}/lib/libastra_core_api.so
    INTERFACE_INCLUDE_DIRECTORIES
        ${ASTRA_ROOT}/include
)


add_library(astra_core_sdk SHARED IMPORTED)
set_target_properties(astra_core_sdk PROPERTIES
    IMPORTED_LOCATION
        ${ASTRA_ROOT}/lib/libastra_core.so
    INTERFACE_INCLUDE_DIRECTORIES
        ${ASTRA_ROOT}/include
)

# ------------- linking --------


#target_include_directories(astra_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


#target_link_libraries(astra_core PUBLIC astra astra_core_api astra_core_sdk)
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


#pybind11_add_module(astra_cpp src/AstraCameraBindings.cpp)

#target_link_libraries(astra_cpp PRIVATE astra_core astra astra_core_api astra_core_sdk)
# ------------------- output pybind modules to proper python appication packages -----


add_custom_command(TARGET motoman_cpp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Copying YMConnect"
    COMMAND ${CMAKE_COMMAND} -E copy
        ${YMCONNECT_ROOT}/libYMConnect.so
        $<TARGET_FILE_DIR:motoman_cpp>
)

# ------------------- output pybind modules to proper python appication packages -----

set_target_properties(motoman_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/cobot_application/cobot
)

#set_target_properties(astra_cpp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/cobot_application/camera)


cmake_minimum_required(VERSION 3.10)
project(ydlidar_tmini_plus LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
    set(YDLIDAR_TMINI_PLUS_YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(YDLIDAR_TMINI_PLUS_YAML_TARGET yaml-cpp)
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
    find_package(nlohmann_json REQUIRED)
endif()

option(YDLIDAR_TMINI_PLUS_BUILD_NODE "Build the YDLIDAR T-mini Plus Dora node" ON)

if(YDLIDAR_TMINI_PLUS_BUILD_NODE)
    # Reuse the SDK location configured by the DORA_NAV root project.
    if(NOT EXISTS "${DORA_INCLUDE_DIR}/node_api.h" OR
       NOT EXISTS "${DORA_NODE_API_LIB}")
        message(FATAL_ERROR
            "YDLIDAR T-mini Plus requires DORA_INCLUDE_DIR and DORA_NODE_API_LIB. "
            "Configure the DORA_NAV root project, or set these paths explicitly.")
    endif()

    set(YDLIDAR_SDK_VERSION "1.2.3")
    configure_file(src/ydlidar_sdk/ydlidar_config.h.in sdk_config/ydlidar_config.h @ONLY)

    # Build the bundled SDK sources directly. Its standalone CMake also creates
    # global packaging/uninstall targets and options shared with other drivers.
    add_library(ydlidar_tmini_plus_sdk STATIC
        src/ydlidar_sdk/core/base/timer.cpp
        src/ydlidar_sdk/core/common/ydlidar_def.cpp
        src/ydlidar_sdk/core/network/ActiveSocket.cpp
        src/ydlidar_sdk/core/network/PassiveSocket.cpp
        src/ydlidar_sdk/core/network/SimpleSocket.cpp
        src/ydlidar_sdk/core/serial/serial.cpp
        src/ydlidar_sdk/src/CYdLidar.cpp
        src/ydlidar_sdk/src/DTSLidarDriver.cpp
        src/ydlidar_sdk/src/ETLidarDriver.cpp
        src/ydlidar_sdk/src/GSLidarDriver.cpp
        src/ydlidar_sdk/src/SDMLidarDriver.cpp
        src/ydlidar_sdk/src/YDlidarDriver.cpp
        src/ydlidar_sdk/src/filters/NoiseFilter.cpp
        src/ydlidar_sdk/src/filters/StrongLightFilter.cpp
        src/ydlidar_sdk/src/ydlidar_sdk.cpp)
    if(WIN32)
        target_sources(ydlidar_tmini_plus_sdk PRIVATE
            src/ydlidar_sdk/core/serial/impl/windows/list_ports_win.cpp
            src/ydlidar_sdk/core/serial/impl/windows/win_serial.cpp)
        target_link_libraries(ydlidar_tmini_plus_sdk PRIVATE winmm setupapi ws2_32)
    else()
        target_sources(ydlidar_tmini_plus_sdk PRIVATE
            src/ydlidar_sdk/core/serial/impl/unix/list_ports_linux.cpp
            src/ydlidar_sdk/core/serial/impl/unix/unix_serial.cpp)
        if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
            target_link_libraries(ydlidar_tmini_plus_sdk PRIVATE rt)
        endif()
    endif()
    set_target_properties(ydlidar_tmini_plus_sdk PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON POSITION_INDEPENDENT_CODE ON)
    target_include_directories(ydlidar_tmini_plus_sdk PUBLIC
        src/ydlidar_sdk src/ydlidar_sdk/core src/ydlidar_sdk/src
        "${CMAKE_CURRENT_BINARY_DIR}/sdk_config")
    target_link_libraries(ydlidar_tmini_plus_sdk PUBLIC Threads::Threads)

    add_executable(ydlidar_tmini_plus_dora_node node/ydlidar_tmini_plus_dora_node.cpp)
    target_compile_features(ydlidar_tmini_plus_dora_node PRIVATE cxx_std_17)
    target_compile_definitions(ydlidar_tmini_plus_dora_node PRIVATE
        YDLIDAR_TMINI_PLUS_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")
    target_include_directories(ydlidar_tmini_plus_dora_node PRIVATE
        src "${DORA_INCLUDE_DIR}")
    target_link_libraries(ydlidar_tmini_plus_dora_node PRIVATE
        ydlidar_tmini_plus_sdk nlohmann_json::nlohmann_json ${YDLIDAR_TMINI_PLUS_YAML_TARGET}
        "${DORA_NODE_API_LIB}" ${CMAKE_DL_LIBS})
endif()

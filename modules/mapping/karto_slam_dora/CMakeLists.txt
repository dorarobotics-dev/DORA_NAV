cmake_minimum_required(VERSION 3.16)
project(karto_slam_dora LANGUAGES CXX)

option(KARTO_BUILD_NODE "Build the Karto 2D Dora node" ON)

find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
find_package(TBB REQUIRED)
if(NOT TARGET nlohmann_json::nlohmann_json)
    find_package(nlohmann_json REQUIRED)
endif()

if(TARGET Ceres::ceres)
    set(KARTO_CERES_TARGET Ceres::ceres)
elseif(TARGET ceres)
    set(KARTO_CERES_TARGET ceres)
else()
    message(FATAL_ERROR "Ceres package did not provide an imported library target")
endif()
if(TARGET yaml-cpp::yaml-cpp)
    set(KARTO_YAML_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(KARTO_YAML_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp package did not provide an imported library target")
endif()

# Keep the algorithm and its Dora support in one module-local core library.
add_library(karto_core STATIC
    src/core/karto/Karto.cpp
    src/core/karto/Mapper.cpp
    src/archive.cpp
    src/ceres_pose_graph_solver.cpp
    src/config.cpp
    src/karto_mapper.cpp
    src/message_adapter.cpp
    src/odometry_buffer.cpp
    src/occupancy_grid.cpp
    src/pose_extrapolator.cpp
    src/runtime.cpp
    src/scan_deskewer.cpp
    src/scan_preparer.cpp)
target_compile_features(karto_core PUBLIC cxx_std_17)
target_include_directories(karto_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/karto")
target_link_libraries(karto_core PUBLIC
    nlohmann_json::nlohmann_json Eigen3::Eigen
    Boost::system Boost::serialization Boost::filesystem Boost::thread TBB::tbb
    ${KARTO_CERES_TARGET} OpenSSL::Crypto Threads::Threads ${KARTO_YAML_TARGET})
target_compile_options(karto_core PRIVATE -Wall -Wextra)

if(KARTO_BUILD_NODE)
    if(NOT EXISTS "${DORA_INCLUDE_DIR}/node_api.h" OR
       NOT EXISTS "${DORA_NODE_API_LIB}")
        message(FATAL_ERROR
            "Karto 2D requires DORA_INCLUDE_DIR and DORA_NODE_API_LIB. "
            "Configure the DORA_NAV root project, or set these paths explicitly.")
    endif()
    add_executable(karto_slam_dora_node src/karto_slam_dora_node.cpp)
    target_compile_definitions(karto_slam_dora_node PRIVATE
        KARTO_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")
    target_include_directories(karto_slam_dora_node PRIVATE "${DORA_INCLUDE_DIR}")
    target_link_libraries(karto_slam_dora_node PRIVATE
        karto_core "${DORA_NODE_API_LIB}" ${CMAKE_DL_LIBS})
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_libraries(karto_slam_dora_node PRIVATE m rt)
    endif()
endif()

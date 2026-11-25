cmake_minimum_required(VERSION 3.16)
project(feetech_kiwi_chassis_dora_node LANGUAGES NONE)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "Feetech Kiwi chassis currently supports Linux targets only")
endif()
if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "Build Feetech Kiwi chassis natively on the target Linux device")
endif()

find_program(FEETECH_CARGO_EXECUTABLE cargo HINTS "$ENV{HOME}/.cargo/bin")
find_program(FEETECH_RUSTC_EXECUTABLE rustc HINTS "$ENV{HOME}/.cargo/bin")
if(NOT FEETECH_CARGO_EXECUTABLE OR NOT FEETECH_RUSTC_EXECUTABLE)
    message(FATAL_ERROR "Feetech Kiwi chassis requires Rust/Cargo; install Cargo or disable BUILD_FEETECH_KIWI_CHASSIS")
endif()

# The Rust node uses the pinned dora-node-api crate; no C SDK link step.
set(FEETECH_MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/sdk_node/Cargo.toml")
set(FEETECH_CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(FEETECH_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}/feetech-kiwi-chassis-sdk-node")
file(GLOB_RECURSE FEETECH_RUST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/sdk_node/src/*.rs")

add_custom_command(
    OUTPUT "${FEETECH_EXECUTABLE}"
    COMMAND "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS
        "CARGO_TARGET_DIR=${FEETECH_CARGO_TARGET_DIR}"
        "RUSTC=${FEETECH_RUSTC_EXECUTABLE}"
        "${FEETECH_CARGO_EXECUTABLE}" build --release --locked
        --manifest-path "${FEETECH_MANIFEST}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${FEETECH_CARGO_TARGET_DIR}/release/feetech-kiwi-chassis-sdk-node"
        "${FEETECH_EXECUTABLE}"
    DEPENDS ${FEETECH_RUST_SOURCES}
        "${FEETECH_MANIFEST}"
        "${CMAKE_CURRENT_SOURCE_DIR}/sdk_node/Cargo.lock"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/sdk_node"
    COMMENT "Building Feetech Kiwi Adora Nano chassis Dora node"
    VERBATIM)
add_custom_target(feetech_kiwi_chassis_dora_node ALL DEPENDS "${FEETECH_EXECUTABLE}")

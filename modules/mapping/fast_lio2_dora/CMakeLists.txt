cmake_minimum_required(VERSION 3.16)
project(fast_lio2_dora LANGUAGES NONE)

find_program(FAST_LIO2_CARGO cargo HINTS "$ENV{HOME}/.cargo/bin")
if(NOT FAST_LIO2_CARGO)
    message(FATAL_ERROR "FAST-LIO2 requires Rust/Cargo 1.95 or newer")
endif()
set(FAST_LIO2_MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml")
set(FAST_LIO2_CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(FAST_LIO2_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}/fast_lio2_dora_node${CMAKE_EXECUTABLE_SUFFIX}")
file(GLOB_RECURSE FAST_LIO2_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.rs"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/fast_lio/Cargo.toml")
add_custom_command(
    OUTPUT "${FAST_LIO2_EXECUTABLE}"
    COMMAND "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS --unset=MFLAGS
        "CARGO_TARGET_DIR=${FAST_LIO2_CARGO_TARGET_DIR}"
        "${FAST_LIO2_CARGO}" build --release --locked --manifest-path "${FAST_LIO2_MANIFEST}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${FAST_LIO2_CARGO_TARGET_DIR}/release/fast_lio2_dora_node${CMAKE_EXECUTABLE_SUFFIX}"
        "${FAST_LIO2_EXECUTABLE}"
    DEPENDS ${FAST_LIO2_SOURCES} "${FAST_LIO2_MANIFEST}" "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.lock"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Building FAST-LIO2 with Dora node API 1.1.0"
    VERBATIM)
add_custom_target(fast_lio2_dora_node ALL DEPENDS "${FAST_LIO2_EXECUTABLE}")

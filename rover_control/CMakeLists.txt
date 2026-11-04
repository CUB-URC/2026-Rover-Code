cmake_minimum_required(VERSION 3.18)
project(rover_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # helps VS Code / clangd find headers

option(ROVER_BUILD_PYTHON "Build the pybind11 Python module" ON)
option(ROVER_BUILD_TESTS  "Build the C++ unit tests"         ON)

# ---------------------------------------------------------------------------
# Core C++ library: no dependencies. Used by the Python module (base station),
# the C++ tests, and later the rover-side ROS 2 bridge in the comms package.
# ---------------------------------------------------------------------------
add_library(rover_core STATIC
    src/protocol.cpp
    src/shaping.cpp
    src/safety.cpp
    src/controller.cpp
    src/command_client.cpp
)
target_include_directories(rover_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Needed because the static lib gets linked into a shared Python module
set_target_properties(rover_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MSVC)
    target_compile_options(rover_core PRIVATE /W4)
else()
    target_compile_options(rover_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Python bindings (pybind11)
# ---------------------------------------------------------------------------
if(ROVER_BUILD_PYTHON)
    find_package(Python 3.9 COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 CONFIG QUIET)
    if(NOT pybind11_FOUND)
        message(STATUS "pybind11 not found locally - fetching from GitHub")
        include(FetchContent)
        FetchContent_Declare(pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG        v2.13.6)
        FetchContent_MakeAvailable(pybind11)
    endif()

    pybind11_add_module(_core python/bindings.cpp)
    target_link_libraries(_core PRIVATE rover_core)

    if(SKBUILD)   # set by scikit-build-core during `pip install .`
        install(TARGETS _core DESTINATION rover)
    endif()
endif()

# ---------------------------------------------------------------------------
# C++ tests (plain asserts, run with `ctest`)
# ---------------------------------------------------------------------------
if(ROVER_BUILD_TESTS AND NOT SKBUILD)
    enable_testing()
    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE rover_core)
    add_test(NAME test_core COMMAND test_core)
endif()

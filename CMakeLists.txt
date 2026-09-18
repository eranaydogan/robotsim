cmake_minimum_required(VERSION 3.20)
project(robotsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Python module is built by default only when invoked from scikit-build-core.
if(DEFINED SKBUILD)
    set(ROBOTSIM_PYTHON_DEFAULT ON)
else()
    set(ROBOTSIM_PYTHON_DEFAULT OFF)
endif()
option(ROBOTSIM_BUILD_PYTHON "Build the Python extension module" ${ROBOTSIM_PYTHON_DEFAULT})
option(BUILD_TESTING "Build the C++ unit tests" OFF)
option(ROBOTSIM_BUILD_BENCHMARKS "Build the C++ benchmarks" OFF)
option(ROBOTSIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# ---------------------------------------------------------------------------
# Compiler options shared by all project targets
# ---------------------------------------------------------------------------
add_library(robotsim_options INTERFACE)
target_compile_features(robotsim_options INTERFACE cxx_std_17)
if(MSVC)
    # /fp:precise is required for deterministic simulation results.
    # /O2 is added by CMake for Release builds; do not add it here.
    target_compile_options(robotsim_options INTERFACE
        /W4
        /permissive-
        /fp:precise
        /utf-8
        /EHsc
        /Zc:__cplusplus
        $<$<BOOL:${ROBOTSIM_WARNINGS_AS_ERRORS}>:/WX>
    )
endif()

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(robotsim_core STATIC
    cpp/src/build_info.cpp
    cpp/src/config.cpp
    cpp/src/geometry.cpp
    cpp/src/platform.cpp
    cpp/src/world.cpp
)
target_include_directories(robotsim_core PUBLIC cpp/include)
target_link_libraries(robotsim_core PRIVATE robotsim_options)
set_target_properties(robotsim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Python extension module
# ---------------------------------------------------------------------------
if(ROBOTSIM_BUILD_PYTHON)
    find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)

    find_package(pybind11 CONFIG QUIET)
    if(NOT pybind11_FOUND)
        execute_process(
            COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
            OUTPUT_VARIABLE ROBOTSIM_PYBIND11_DIR
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        find_package(pybind11 CONFIG REQUIRED HINTS "${ROBOTSIM_PYBIND11_DIR}")
    endif()

    pybind11_add_module(_robotsim MODULE cpp/bindings/module.cpp)
    target_link_libraries(_robotsim PRIVATE robotsim_core robotsim_options)
    install(TARGETS _robotsim LIBRARY DESTINATION robotsim)
endif()

# ---------------------------------------------------------------------------
# C++ unit tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    enable_testing()
    add_executable(robotsim_tests
        cpp/tests/test_main.cpp
        cpp/tests/test_build_info.cpp
        cpp/tests/test_geometry.cpp
        cpp/tests/test_lidar.cpp
        cpp/tests/test_rng.cpp
        cpp/tests/test_step.cpp
        cpp/tests/test_world.cpp
    )
    target_include_directories(robotsim_tests SYSTEM PRIVATE third_party)
    target_link_libraries(robotsim_tests PRIVATE robotsim_core robotsim_options)
    add_test(NAME robotsim_tests COMMAND robotsim_tests)
endif()

# ---------------------------------------------------------------------------
# C++ benchmarks
# ---------------------------------------------------------------------------
if(ROBOTSIM_BUILD_BENCHMARKS)
    add_executable(robotsim_bench cpp/bench/bench_world.cpp)
    target_link_libraries(robotsim_bench PRIVATE robotsim_core robotsim_options)
endif()
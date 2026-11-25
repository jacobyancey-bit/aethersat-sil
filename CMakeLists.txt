cmake_minimum_required(VERSION 3.16)
project(aethersat-sil VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Fetch GoogleTest v1.15.2
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Fetch nlohmann/json v3.11.3
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# Threads
find_package(Threads REQUIRED)

# System libzmq (pkg-config first, fallback to find_library)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ QUIET libzmq)
endif()

if(ZMQ_FOUND)
    set(ZMQ_INCLUDE_DIRS_RESOLVED ${ZMQ_INCLUDE_DIRS})
    set(ZMQ_LIBRARIES_RESOLVED ${ZMQ_LINK_LIBRARIES})
else()
    find_path(ZMQ_INCLUDE_DIR NAMES zmq.h REQUIRED)
    find_library(ZMQ_LIB NAMES zmq REQUIRED)
    set(ZMQ_INCLUDE_DIRS_RESOLVED ${ZMQ_INCLUDE_DIR})
    set(ZMQ_LIBRARIES_RESOLVED ${ZMQ_LIB})
endif()

# Core simulator library
add_library(aethersat_core STATIC
    src/physics/orbital_propagator.cpp
    src/physics/attitude_adcs.cpp
    src/physics/power_eps.cpp
    src/ccsds/space_packet.cpp
    src/bus/mil_std_1553.cpp
    src/bus/can_bus.cpp
    src/bus/uart_bus.cpp
    src/sim/spacecraft_sim.cpp
)

target_include_directories(aethersat_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ZMQ_INCLUDE_DIRS_RESOLVED}
)

target_link_libraries(aethersat_core
    PUBLIC
        nlohmann_json::nlohmann_json
        Threads::Threads
        ${ZMQ_LIBRARIES_RESOLVED}
)

target_compile_options(aethersat_core
    PRIVATE
        -Wall -Wextra -Wpedantic
)

# Simulator executable
add_executable(aethersat_sim src/main.cpp)
target_link_libraries(aethersat_sim PRIVATE aethersat_core)
target_compile_options(aethersat_sim PRIVATE -Wall -Wextra -Wpedantic)

# Testing
enable_testing()
include(GoogleTest)

function(aethersat_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE aethersat_core GTest::gtest_main)
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    gtest_discover_tests(${target_name})
endfunction()

aethersat_add_test(test_physics tests/cpp/test_physics.cpp)
aethersat_add_test(test_ccsds tests/cpp/test_ccsds.cpp)
aethersat_add_test(test_buses tests/cpp/test_buses.cpp)

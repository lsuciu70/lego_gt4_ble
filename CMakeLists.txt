cmake_minimum_required(VERSION 3.22)
project(LWP3-GT4-SDK)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(simpleble REQUIRED)
find_package(Threads REQUIRED)

# SDK Library
add_library(lwp3gt4 src/Lwp3Gt4.cpp src/Lwp3Config.cpp)
target_include_directories(lwp3gt4 PUBLIC include)
# Absolute path so config/gt4.conf is found regardless of the working
# directory the binary is run from.
target_compile_definitions(lwp3gt4 PRIVATE GT4_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/gt4.conf")

# --- THE FIX IS HERE ---
# We link atomic so the compiler can handle the 16-byte Telemetry struct
target_link_libraries(lwp3gt4 
    simpleble::simpleble 
    Threads::Threads 
    atomic
)

# Examples & Tests
add_executable(hello_gt4 examples/hello_gt4.cpp)
target_link_libraries(hello_gt4 lwp3gt4)

add_executable(ble_latency_test tests/ble_latency_test.cpp)
target_link_libraries(ble_latency_test lwp3gt4)

add_executable(async_test tests/async_test.cpp)
target_link_libraries(async_test lwp3gt4)

add_executable(drive_telemetry_test tests/drive_telemetry_test.cpp)
target_link_libraries(drive_telemetry_test lwp3gt4)
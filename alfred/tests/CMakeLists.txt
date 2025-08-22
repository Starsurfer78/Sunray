# Alfred Component Integration Tests
cmake_minimum_required(VERSION 3.14)

project(alfred_tests)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(GTest REQUIRED)
find_package(GMock REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/../src)
include_directories(${CMAKE_SOURCE_DIR}/../../sunray)

# Set firmware path for consistency with main CMakeLists.txt
SET(FIRMWARE_PATH ${CMAKE_SOURCE_DIR}/../../sunray)
SET(EXCLUDE_REGEX "agcm4|due|esp")

# Compiler flags for compatibility with main project
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -x c")
SET(CMAKE_BUILD_TYPE Debug)

# Define test sources
set(TEST_SOURCES
    test_config_structures.cpp
    test_config_error_handling.cpp
    config_validator.cpp
    test_serial_robot_driver.cpp
    test_motor_pid_integration.cpp
    test_end_to_end.cpp
    test_performance.cpp
    mocks/mock_serial.cpp
    mocks/mock_stm32.cpp
    test_main.cpp
)

# Define source files to test (excluding main executable)
file(GLOB_RECURSE sunray_test_cpp ${FIRMWARE_PATH}/**.cpp)
list(FILTER sunray_test_cpp EXCLUDE REGEX ${EXCLUDE_REGEX})
list(FILTER sunray_test_cpp EXCLUDE REGEX "sunray.ino")

file(GLOB_RECURSE sunray_test_c ${FIRMWARE_PATH}/**.c)
list(FILTER sunray_test_c EXCLUDE REGEX ${EXCLUDE_REGEX})

file(GLOB_RECURSE pi_test_sources ${CMAKE_SOURCE_DIR}/../src/**.cpp ${CMAKE_SOURCE_DIR}/../src/**.c)
list(FILTER pi_test_sources EXCLUDE REGEX "wiring_main.cpp")

# Create test executable
add_executable(alfred_tests 
    ${TEST_SOURCES}
    ${pi_test_sources}
    ${sunray_test_cpp}
    ${sunray_test_c}
)

# Link libraries
target_link_libraries(alfred_tests 
    GTest::gtest 
    GTest::gtest_main
    GMock::gmock
    GMock::gmock_main
    pthread
)

# Include directories for tests
target_include_directories(alfred_tests PRIVATE 
    ${CMAKE_SOURCE_DIR}/../src 
    ${FIRMWARE_PATH}/src
    ${CMAKE_SOURCE_DIR}/mocks
)

# Add test discovery
include(GoogleTest)
gtest_discover_tests(alfred_tests)

# Custom test targets
add_custom_target(test_config
    COMMAND alfred_tests --gtest_filter="ConfigStructuresTest.*"
    DEPENDS alfred_tests
    COMMENT "Running config structure tests"
)

add_custom_target(test_serial
    COMMAND alfred_tests --gtest_filter="SerialRobotDriverTest.*"
    DEPENDS alfred_tests
    COMMENT "Running serial robot driver tests"
)

add_custom_target(test_motor_pid
    COMMAND alfred_tests --gtest_filter="MotorPIDTest.*"
    DEPENDS alfred_tests
    COMMENT "Running motor PID integration tests"
)

add_custom_target(test_e2e
    COMMAND alfred_tests --gtest_filter="EndToEndTest.*"
    DEPENDS alfred_tests
    COMMENT "Running end-to-end tests"
)

add_custom_target(test_performance
    COMMAND alfred_tests --gtest_filter="PerformanceTest.*"
    DEPENDS alfred_tests
    COMMENT "Running performance tests"
)

# Coverage target (optional)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    
    add_custom_target(coverage
        COMMAND lcov --directory . --capture --output-file coverage.info
        COMMAND lcov --remove coverage.info '/usr/*' --output-file coverage.info
        COMMAND lcov --list coverage.info
        COMMAND genhtml -o coverage_html coverage.info
        DEPENDS alfred_tests
        COMMENT "Generating test coverage report"
    )
endif()
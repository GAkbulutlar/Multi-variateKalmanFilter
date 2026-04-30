cmake_minimum_required(VERSION 3.15)
project(MultiVariateKalmanFilter CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler-specific warning flags
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Find Eigen3 package. If not installed, user must install it via:
# Ubuntu/Debian: sudo apt-get install libeigen3-dev
# macOS: brew install eigen
# Windows: vcpkg install eigen3
find_package(Eigen3 REQUIRED NO_MODULE)

# Create reusable Kalman filter library.
add_library(KalmanLib
    KalmanFilter.cpp
)
target_include_directories(KalmanLib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(KalmanLib
    PUBLIC
        Eigen3::Eigen
)

# Create simulation executable (consumer of KalmanLib).
add_executable(kalman_filter_app
    main.cpp
)
target_link_libraries(kalman_filter_app
    PRIVATE
        KalmanLib
)

# Test support.
include(CTest)
if(BUILD_TESTING)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(unit_tests
        tests/test_kalman.cpp
    )
    target_link_libraries(unit_tests
        PRIVATE
            KalmanLib
            GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(unit_tests)
endif()

# Output directory for the executable
set_target_properties(kalman_filter_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

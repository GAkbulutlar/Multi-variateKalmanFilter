cmake_minimum_required(VERSION 3.15)
project(MultiVariateKalmanFilter CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Create the Kalman Filter executable
add_executable(kalman_filter_app
    main.cpp
    KalmanFilter.cpp
)

# Link against Eigen3
target_link_libraries(kalman_filter_app Eigen3::Eigen)

# Output directory for the executable
set_target_properties(kalman_filter_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

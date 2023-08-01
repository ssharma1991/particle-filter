cmake_minimum_required(VERSION 3.16)
project(particle_filter
        VERSION 0.0.1
        LANGUAGES CXX
)

# Update the submodules here
include(cmake/UpdateSubmodules.cmake)

# Enable testing for this project
include(CTest)

# Set build type as debug/release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
#set(CMAKE_BUILD_TYPE Debug)
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Create a "proxy" interface target 
# It's used to apply consistent properties throughout the project
add_library(cxx_setup INTERFACE)
target_compile_options(cxx_setup INTERFACE -Wall -Wpedantic -Wextra)
target_compile_features(cxx_setup INTERFACE cxx_std_17)
target_include_directories(cxx_setup INTERFACE ${PROJECT_SOURCE_DIR})

# Make sure CMakeLists.txt exist in each subdirectory
add_subdirectory(src)
add_subdirectory(examples)
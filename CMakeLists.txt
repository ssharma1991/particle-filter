cmake_minimum_required(VERSION 3.10)
project(particle_filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

add_executable(particle_filter src/particle_filter.cpp src/particle_filter_helper.cpp include/params.h)
target_link_libraries(particle_filter ${OpenCV_LIBS} Eigen3::Eigen)
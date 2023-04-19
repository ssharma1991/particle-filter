cmake_minimum_required(VERSION 3.10)
project(particle_filter)

find_package(OpenCV REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

add_executable(particle_filter src/particle_filter.cpp src/particle_filter_helper.cpp include/params.h)
target_link_libraries(particle_filter ${OpenCV_LIBS})
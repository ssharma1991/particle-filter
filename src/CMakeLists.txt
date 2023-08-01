add_library(particle_filter particle_filter.cpp particle_filter_helper.cpp)
target_include_directories(particle_filter PUBLIC ${PROJECT_SOURCE_DIR}/external/eigen-3.4.0)
find_package(OpenCV REQUIRED)
target_link_libraries(particle_filter PUBLIC ${OpenCV_LIBS} cxx_setup)
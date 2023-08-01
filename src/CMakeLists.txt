add_library(particle_filter particle_filter.cpp particle_filter_helper.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(particle_filter PUBLIC ${OpenCV_LIBS} cxx_setup)
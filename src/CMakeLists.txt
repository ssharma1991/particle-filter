add_library(particle_filter particle_filter.cpp particle_filter_helper.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(particle_filter PUBLIC ${OpenCV_LIBS} cxx_setup)

if(BUILD_TESTING)
    add_executable(my_tests particle_filter_tests.cpp)
    target_link_libraries(my_tests PRIVATE particle_filter GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(my_tests)
endif()
add_executable(replay_log replay_log.cpp)
target_include_directories(replay_log PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(replay_log PRIVATE particle_filter cxx_setup)
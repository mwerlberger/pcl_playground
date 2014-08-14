cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(read_and_viz)

add_executable(read_and_viz read_and_viz.cpp)
target_link_libraries(read_and_viz ${PCL_LIBRARIES})


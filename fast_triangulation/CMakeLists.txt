cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(fast_triangulation)

add_executable(fast_triangulation fast_triangulation.cpp)
target_link_libraries(fast_triangulation ${PCL_LIBRARIES})


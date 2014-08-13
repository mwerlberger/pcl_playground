cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(marching_cubes)

add_executable(marching_cubes marching_cubes.cpp)
target_link_libraries(marching_cubes ${PCL_LIBRARIES})


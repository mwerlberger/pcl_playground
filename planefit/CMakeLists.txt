cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(planefit)

add_executable(planefit planefit.cpp)
target_link_libraries(planefit ${PCL_LIBRARIES})


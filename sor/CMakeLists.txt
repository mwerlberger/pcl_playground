cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(sor)

add_executable(sor sor.cpp)
target_link_libraries(sor ${PCL_LIBRARIES})


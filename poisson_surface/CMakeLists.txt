cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(poisson_surface)

add_executable(poisson_surface poisson_surface.cpp)
target_link_libraries(poisson_surface ${PCL_LIBRARIES})


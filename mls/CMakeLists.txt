cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(mls)

add_executable(mls mls.cpp)
target_link_libraries(mls ${PCL_LIBRARIES})


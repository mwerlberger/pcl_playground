cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pcl_playground)

## PCL dependencies
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_subdirectory(sor)
add_subdirectory(mls)
add_subdirectory(planefit)
add_subdirectory(fast_triangulation)
add_subdirectory(poisson_surface)
add_subdirectory(marching_cubes)


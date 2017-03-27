cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_demo)

find_package(PCL 1.2 REQUIRED)

SET(CMAKE_C_COMPILER mpicc)

SET(CMAKE_CXX_COMPILER mpicxx)

include_directories(MPI_INCLUDE_PATH)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES}
                                              ${MPI_LIBRARIES})

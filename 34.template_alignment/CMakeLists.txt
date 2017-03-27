cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(template_alignment)

find_package(PCL 1.2 REQUIRED)
SET(CMAKE_C_COMPILER mpicc)

SET(CMAKE_CXX_COMPILER mpicxx)

include_directories(MPI_INCLUDE_PATH)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (template_alignment template_alignment.cpp)
target_link_libraries (template_alignment ${PCL_LIBRARIES}
    ${MPI_LIBRARIES})

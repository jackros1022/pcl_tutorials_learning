cmake_minimum_required(VERSION 2.8)

project(implicit_shape_model)
find_package(PCL 1.7 REQUIRED)

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_C_COMPILER mpicc)
set(CMAKE_CXX_COMPILER mpicxx)
include_directories(MPI_INCLUDE_PATH)

add_executable(main "main.cpp")
target_link_libraries(main ${PCL_LIBRARIES}
    ${MPI_LIBRAIES})

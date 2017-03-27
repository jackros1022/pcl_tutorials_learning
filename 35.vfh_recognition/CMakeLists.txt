cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# we need FindFLANN.cmake 
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

project(vfh_cluster_classifier)

find_package(PCL 1.2 REQUIRED)
SET(CMAKE_C_COMPILER mpicc)
SET(CMAKE_CXX_COMPILER mpicxx)
include_directories(MPI_INCLUDE_PATH)

include_directories(${PCL_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES("/opt/ros/kinetic/include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(HDF5)
if(HDF5_FOUND)

  find_package(Flann)
  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(build_tree build_tree.cpp)
  target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY}
                                 ${MPI_LIBRARIES})

  add_executable(nearest_neighbors nearest_neighbors.cpp)
  target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} 
                                        ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} 
                                        ${MPI_LIBRARIES}
                                        )
endif(HDF5_FOUND)

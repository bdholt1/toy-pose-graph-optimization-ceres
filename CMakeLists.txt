cmake_minimum_required(VERSION 2.8.3)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set( CMAKE_CXX_FLAGS "-fpermissive" )


include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable( example_2d
        src/example_2d.cpp
        src/pose_graph_2d.cpp )
target_link_libraries( example_2d ${CERES_LIBRARIES}   )

add_executable( example_3d
        src/example_3d.cpp
        src/pose_graph_3d.cpp )
target_link_libraries( example_3d ${CERES_LIBRARIES} )


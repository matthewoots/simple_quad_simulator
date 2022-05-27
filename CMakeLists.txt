cmake_minimum_required(VERSION 3.0.2)
project(quad)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  mavros_msgs
  trajectory_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  nav_msgs
  mavros_msgs
  trajectory_msgs

  DEPENDS
  Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (${PROJECT_NAME}_node 
  src/main.cpp
  src/quad.cpp
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)

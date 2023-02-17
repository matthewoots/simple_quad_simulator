cmake_minimum_required(VERSION 3.8)
project(quad)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  mavros_msgs
  trajectory_msgs
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  nav_msgs
  mavros_msgs
  trajectory_msgs
  pcl_ros

  DEPENDS
  Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable (${PROJECT_NAME}_node 
  src/main.cpp
  src/quad.cpp
  external/zju_laser_sim.cpp
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable (map_generation
  external/zju_random_forest.cpp
)
target_link_libraries(map_generation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable (map_publisher
  src/map_simple_pub.cpp
)
target_link_libraries(map_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

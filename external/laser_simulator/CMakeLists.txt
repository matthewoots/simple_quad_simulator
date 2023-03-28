cmake_minimum_required(VERSION 3.10)
project(laser_simulator)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)

add_library(${PROJECT_NAME} zju_laser_sim.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)
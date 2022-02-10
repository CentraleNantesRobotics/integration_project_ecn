cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED)

include_directories(include)

add_executable(utils src/utils.cpp)



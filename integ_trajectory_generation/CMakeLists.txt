cmake_minimum_required(VERSION 3.0.0)
project(integ_trajectory_generation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    )


catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
    )

include_directories(include ${catkin_INCLUDE_DIRS})

# Set the name to use for the executable.
set(BINNAME1 ${PROJECT_NAME})

# Set the source files to use with the executable.
set(SRCS1 ${SRCS1}  src/generation_main.cpp
                    src/generation_node.cpp
                    src/utils.cpp)

# Build the executable that will be used to run this node.
add_executable(${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})

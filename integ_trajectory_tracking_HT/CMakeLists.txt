cmake_minimum_required(VERSION 3.0.0)
project(integ_trajectory_tracking_HT)

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
#set(SRCS1 ${SRCS1}  src/CTC_controller.cpp)
set(SRCS2 ${SRCS2}  src/PID_controller.cpp)

# Build the executable that will be used to run this node.
add_executable(${BINNAME1} ${SRCS2})

target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})

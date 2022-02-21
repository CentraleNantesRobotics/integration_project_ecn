cmake_minimum_required(VERSION 2.8.3)
project(image_processing)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  rospy
  urdf
)

find_package(VISP REQUIRED COMPONENTS core)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Python libs
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include ${VISP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy cv_bridge roscpp image_transport std_msgs
                    sensor_msgs geometry_msgs
  DEPENDS ${VISP_LIBRARIES} ${OpenCV_LIBS} 
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

## Main C++ library
add_library(${PROJECT_NAME}
                include/ecn_common/color_detector.h
                src/color_detector.cpp
                include
                )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

## Test executables

add_executable(mass_center_detector examples/mass_center_detector.cpp)
target_link_libraries(mass_center_detector ${PROJECT_NAME})

add_executable(center_circle_detector examples/center_circle_detector.cpp)
target_link_libraries(center_circle_detector ${PROJECT_NAME})


#############
## Install ##
#############


 # only install actual library

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

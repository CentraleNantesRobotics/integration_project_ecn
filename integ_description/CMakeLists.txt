cmake_minimum_required(VERSION 3.5)

project(integ_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)



include_directories(${sensor_msgs_INCLUDE_DIRS})


install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

add_executable(integ_CTC src/integ_CTC)

ament_package()

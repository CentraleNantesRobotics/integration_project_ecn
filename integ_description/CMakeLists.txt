cmake_minimum_required(VERSION 3.5)

project(integ_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS launch/sim.sh DESTINATION lib/${PROJECT_NAME})

ament_package()

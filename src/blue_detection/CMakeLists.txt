cmake_minimum_required(VERSION 3.8)
project(blue_detection)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/Blue_Detection.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()

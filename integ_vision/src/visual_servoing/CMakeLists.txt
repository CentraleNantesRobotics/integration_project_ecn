cmake_minimum_required(VERSION 3.5)
project(visual_servoing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS visual_servoing/FeatureDetector
                visual_servoing/draft_tracker
                visual_servoing/tracker
        DESTINATION lib/${PROJECT_NAME})

ament_package()

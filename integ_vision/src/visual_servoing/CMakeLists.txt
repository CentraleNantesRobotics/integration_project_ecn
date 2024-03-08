cmake_minimum_required(VERSION 3.5)
project(visual_servoing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS visual_servoing/FeatureDetector.py
                visual_servoing/draft_tracker.py
                visual_servoing/tracker.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()

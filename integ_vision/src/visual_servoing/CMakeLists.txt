cmake_minimum_required(VERSION 3.5)
project(visual_servoing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS visual_servoing/FeatureDetector
                visual_servoing/draft_tracker
                visual_servoing/tracker
                visual_servoing/cirlcleDetector
        DESTINATION lib/${PROJECT_NAME})

ament_package()

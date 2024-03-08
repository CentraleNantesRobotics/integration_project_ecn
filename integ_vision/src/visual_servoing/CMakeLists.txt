cmake_minimum_required(VERSION 3.5)
project(visual_servoing)

find_package(ament_cmake REQUIRED)

<<<<<<< HEAD
install(PROGRAMS visual_servoing/FeatureDetector.py
                visual_servoing/draft_tracker.py
                visual_servoing/tracker.py
=======
install(PROGRAMS visual_servoing/FeatureDetector
                visual_servoing/draft_tracker
                visual_servoing/tracker
>>>>>>> a9e1ef318512a0223dcd4bb5019dfe7be9c833ec
        DESTINATION lib/${PROJECT_NAME})

ament_package()

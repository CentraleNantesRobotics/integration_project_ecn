cmake_minimum_required(VERSION 3.8)
project(integ_trajec)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(integ_trajec_srv src/trajectory_generation_service.cpp include/polynomial_interpolator.cpp)

ament_auto_add_executable(integ_trajec_clt src/trajectory_generation_client.cpp)

ament_auto_add_executable(goal_pose src/goal_pose.cpp)


install(TARGETS
    integ_trajec_srv
    integ_trajec_clt
    goal_pose
    DESTINATION lib/${PROJECT_NAME})

ament_auto_package()

cmake_minimum_required(VERSION 3.8)
project(integ_trajec)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)
ament_auto_add_executable(integ_trajec_srv src/trajectory_generation_service.cpp src/polynomial_interpolator.cpp)

ament_auto_add_executable(integ_trajec_clt src/trajectory_generation_client.cpp)

install(TARGETS
    integ_trajec_srv
    integ_trajec_clt
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()

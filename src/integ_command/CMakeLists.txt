cmake_minimum_required(VERSION 3.8)
project(integ_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)


include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(include/ )



ament_auto_add_executable(integ_command src/integ_CTC.cpp)
target_include_directories(integ_command  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include )

install(DIRECTORY scripts
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
        
ament_auto_package()


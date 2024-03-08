cmake_minimum_required(VERSION 3.8)
project(pid)

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
find_package(baxter_core_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



find_package(Eigen3)

set(CMAKE_CXX_STANDARD 17)

include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(include/ ${EIGEN3_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PIDcommand.msg"
 )


#ament_auto_add_executable(pid src/integ_CTC.cpp)
#target_include_directories(pid  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include )
install(PROGRAMS	
		src/pid_node.py
		DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY scripts
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
        
ament_auto_package()


/*
 * ROS pkg to control 2R robot joints using a Computed Torque controller for trajectory tracking
 * Authors: Joe Ferreira Scholtz and Julio Chi
 */

//includes
#include <cmath>
#include <std_msgs/Float64.h>
#include "ros/ros.h"

int main(int argc, char** argv){
    //Initialize ROS Node
    ros::init(argc,argv,"pid_controller");

    return 0;
}

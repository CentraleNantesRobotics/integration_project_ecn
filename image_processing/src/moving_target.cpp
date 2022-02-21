#include "ros/ros.h"
#include <cstdlib>
#include <gazebo_ros/PhysicsConfig.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    client.waitForExistence();
    gazebo_msgs::SetModelState setmodelstate;
    auto & modelstate{setmodelstate.request.model_state};
    modelstate.model_name = "cible_simple";
    modelstate.reference_frame = "world";

    auto &pose{modelstate.pose};


    const auto w = M_PI/4;
    const auto r = 0.1;

    ros::Rate rate(10);

    while(ros::ok())
    {

    const auto t{ros::Time::now().toSec()};

    pose.position.x = 2.0;
    pose.position.y = r*(5*sin(w*t)-4*sin(4*w*t));
    pose.position.z = 1.4 + r*(5*cos(w*t)-4*cos(4*w*t));


    client.call(setmodelstate);
    ros::spinOnce();

    rate.sleep();
    }
}

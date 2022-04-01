//example pgm to set a model state in Gazebo
// could also do w/ rosservice call gazebo/set_model_state
#include <ros/ros.h> //ALWAYS need to include this
//#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <math.h>

double circle_radius = 0.3;

int main(int argc, char **argv) {
    ros::init(argc, argv, "init_model_state");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.005);
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);

      ROS_INFO("waiting for set_model_state service");
    }
    ROS_INFO("set_model_state service exists");

    ros::ServiceClient set_model_state_client =
       nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState model_state_srv_msg;

    double speed = 10; //speed is the time increment in seconds between each iteration
    double begin_time = ros::Time::now().toSec();
    double time = begin_time;
    while (true)
    {
        time = ros::Time::now().toSec() - begin_time;


        model_state_srv_msg.request.model_state.model_name = "target";
        model_state_srv_msg.request.model_state.pose.position.x = 1;
        model_state_srv_msg.request.model_state.pose.position.y = circle_radius * sin(time*speed);
        model_state_srv_msg.request.model_state.pose.position.z = 1 + circle_radius * cos(time*speed);
        model_state_srv_msg.request.model_state.reference_frame = "world";
        set_model_state_client.call(model_state_srv_msg);
        ROS_INFO("waiting for set_model_state service");

            //make sure service call was successful
            bool result = model_state_srv_msg.response.success;
            if (!result)
                ROS_WARN("service call to set_model_state failed!");
            else
                ROS_INFO("Done");
    }
    //hard code, or could prompt, or could have command-line arg here:




}

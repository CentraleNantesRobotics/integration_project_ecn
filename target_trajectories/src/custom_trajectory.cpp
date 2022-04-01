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

//this is a list of the (xyz) pos you want your target to move to
std::vector<std::vector<double>> custom_trajectory{
    { 1, 1 , 0.5 }, //pos1(x,y,z)
    { 1, -1, 0.5 }, //pos2
    { 1, 0, 1 } //pos3 etc...
};
//After going to the last point, the target goes back to the first point and redo the cycle undefinitely
;

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

    double speed = 1; //speed is the time increment in seconds between each iteration
    std::vector<double> current_pos = custom_trajectory[0];
    std::vector<double> new_pos = custom_trajectory[0];
    int next_pos_index = 0;
    std::vector<double> direction{0,0,0};
    double norm = 0; //this is used to normalize the direction vector

    //time handling is only to have a speed independant of time
    double last_time = ros::Time::now().toSec();
    double elapsed_time = ros::Time::now().toSec() - last_time;


    while (custom_trajectory.size()>1)
    {
        if (current_pos == custom_trajectory[next_pos_index]) //Check if we reached the position
        {
            //If so then update index and direction to correspond to next position
            next_pos_index = (next_pos_index + 1) % custom_trajectory.size();
            for (int i =0; i <3; i++) direction[i] = custom_trajectory[next_pos_index][i] - current_pos[i];

            norm = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

            for (int i =0; i <3; i++) direction[i]/=norm;
        }

        //check if target is going to go farther than next custom trajectory pos
        if ((direction[0] * (current_pos[0] + speed*elapsed_time*direction[0] - custom_trajectory[next_pos_index][0] )>= 0)
           & (direction[1] * (current_pos[1] + speed*elapsed_time*direction[1] - custom_trajectory[next_pos_index][1] )>= 0)
           & (direction[2] * (current_pos[2] + speed*elapsed_time*direction[2] - custom_trajectory[next_pos_index][2] )>= 0) )
            new_pos = custom_trajectory[next_pos_index]; //if so update target's pose to the custom trajectory's next pos
        else
            for (int i =0; i <3; i++) new_pos[i] = current_pos[i] + speed*elapsed_time*direction[i]; //else the target continue going towards next pos

        model_state_srv_msg.request.model_state.model_name = "target";
        model_state_srv_msg.request.model_state.pose.position.x = new_pos[0];
        model_state_srv_msg.request.model_state.pose.position.y = new_pos[1];
        model_state_srv_msg.request.model_state.pose.position.z = new_pos[2];
        model_state_srv_msg.request.model_state.reference_frame = "world";
        set_model_state_client.call(model_state_srv_msg);
        ROS_INFO("waiting for set_model_state service");

            //make sure service call was successful
            bool result = model_state_srv_msg.response.success;
            if (!result)
                ROS_WARN("service call to set_model_state failed!");
            else
                ROS_INFO("Done");
                current_pos = new_pos;
                elapsed_time = ros::Time::now().toSec() - last_time;
                last_time = ros::Time::now().toSec();

        //increase time at end of iteration
    }
    ROS_INFO("Custom trajectory's size<2, you should edit the code to set a working custom_trajectory");
    //hard code, or could prompt, or could have command-line arg here:




}

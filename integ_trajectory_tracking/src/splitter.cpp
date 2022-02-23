/*
 * ROS Node to split jointStates into floats to use in the built pid controller
 * Authors: Joe Ferreira Scholtz and Julio Chi
 */

//includes
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

class Splitter{
public:
    Splitter(){

        //Node Handle in private namespace to get parameters
        nh_private = ros::NodeHandle("~");

        //getting parameters
        nh_private.param<std::string>("raw_joint_state_topic_name",rawJointStateTopicName,"/jointStates");
        nh_private.param<std::string>("raw_setpoint_topic_name",rawSetpointTopicName,"/setpoint");

        nh_private.param<std::string>("single_state_topic_name",singleStateTopicName,"/first_joint_state_topic");
        nh_private.param<std::string>("single_dstate_topic_name",singleDStateTopicName,"/first_joint_dstate_topic");

        nh_private.param<std::string>("single_setpoint_topic_name",singleSetpointTopicName,"/first_joint_setpoint_topic");
        nh_private.param<std::string>("single_dsetpoint_topic_name",singleDSetpointTopicName,"/first_joint_dsetpoint_topic");
        
        nh_private.param<std::string>("joint_name_to_remap",jointNameToRemap,"/elbow");

        //Topics to publish
        singleStatePub = nh.advertise<std_msgs::Float64>(singleStateTopicName, 10);
        singleDStatePub = nh.advertise<std_msgs::Float64>(singleDStateTopicName, 10);

        singleSetpointPub = nh.advertise<std_msgs::Float64>(singleSetpointTopicName, 10);
        singleDSetpointPub = nh.advertise<std_msgs::Float64>(singleDSetpointTopicName, 10);

        //Topics to subscribe
        rawJointStateSub = nh.subscribe(rawJointStateTopicName, 1, &Splitter::rawJointStateCallback, this);
        rawSetpointSub = nh.subscribe(rawSetpointTopicName, 1, &Splitter::rawSetpointCallback, this);
    }
    void rawJointStateCallback(const sensor_msgs::JointState& jointState){
        std::vector<std::string> names = jointState.name;
        // Find if "jointNameToRemap" is a joint name in "jointState"
        auto it = std::find(names.begin(),names.end(),jointNameToRemap);
        if(it != names.end()){
            //if it's the case, get it's index to acess the corresponding position and velocity of this joint
            int index{(int)(it - names.begin())};
            std_msgs::Float64 position;
            position.data = jointState.position.at(index);

            std_msgs::Float64 velocity;
            velocity.data = jointState.velocity.at(index);
            //then publish
            singleStatePub.publish(position);
            singleDStatePub.publish(velocity);
        }
    }
    void rawSetpointCallback(const sensor_msgs::JointState& setpoint){
        std::vector<std::string> names = setpoint.name;
        // Find if "jointNameToRemap" is a joint name in "setpoint"
        auto it = std::find(names.begin(),names.end(),jointNameToRemap);
        if(it != names.end()){
            //if it's the case, get it's index to acess the corresponding position and velocity of this joint
            int index{(int)(it - names.begin())};
            std_msgs::Float64 position;
            position.data = setpoint.position.at(index);

            std_msgs::Float64 velocity;
            velocity.data = setpoint.velocity.at(index);
            //then publish
            singleSetpointPub.publish(position);
            singleDSetpointPub.publish(velocity);
        }
    }
private:
    //ROS Node stuff
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher singleStatePub, singleDStatePub, singleSetpointPub, singleDSetpointPub;
    ros::Subscriber rawJointStateSub, rawSetpointSub;
    //Parameters
    std::string rawJointStateTopicName, rawSetpointTopicName, singleStateTopicName,singleDStateTopicName,
                singleSetpointTopicName, singleDSetpointTopicName, jointNameToRemap;
};

int main(int argc, char** argv){
    //Initialize ROS Node
    ros::init(argc,argv,"splitter");

    Splitter splitter;
    //loop
    double f = 10; //rate frequency
    ros::Rate r(f);
    while (ros::ok()){
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

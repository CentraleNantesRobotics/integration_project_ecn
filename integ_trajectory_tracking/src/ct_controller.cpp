/*
 * ROS pkg to control 2R robot joints using a Computed Torque controller for trajectory tracking
 * Authors: Joe Ferreira Scholtz and Julio Chi
 */

//includes
#include <cmath>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

class CT_controller{
public:
    CT_controller(){

        //Node Handle in private namespace to get parameters
        nh_private = ros::NodeHandle("~");

        //getting parameters
        nh_private.param<std::string>("joint_state_topic",jointStateTopic,"/state");
        nh_private.param<std::string>("setpoint_topic",setpointTopicName,"/trajectory");

        nh_private.param<std::string>("first_pid_command_topic",firstPidCommandTopic,"/first_pid_command");
        nh_private.param<std::string>("second_pid_command_topic",secondPidCommandTopic,"/second_pid_command");

        nh_private.param<std::string>("first_joint_command_topic",firstJointCommandTopic,"/joint1_effort_controller/command");
        nh_private.param<std::string>("second_joint_command_topic",secondJointCommandTopic,"/joint2_effort_controller/command");

        //Subscribers
        rawJointStateSub = nh.subscribe(jointStateTopic, 10, &CT_controller::jointStateCallback, this);
        setpointSub = nh.subscribe(setpointTopicName, 10, &CT_controller::setpointCallback, this);

        firstPidSubscriber = nh.subscribe(firstPidCommandTopic, 10, &CT_controller::firstPidCallback, this);
        secondPidSubscriber = nh.subscribe(secondPidCommandTopic, 10, &CT_controller::secondPidCallback, this);

        //Publishers
        firstJointCommandPub = nh.advertise<std_msgs::Float64>(firstJointCommandTopic, 10);
        secondJointCommandPub = nh.advertise<std_msgs::Float64>(secondJointCommandTopic, 10);

    }
    //Robot joint state callback
    void jointStateCallback(const sensor_msgs::JointState& jointState_){
        jointState = jointState_;
        newMsgOrState = true;
    }
    //trajectory setpoint callback
    void setpointCallback(const sensor_msgs::JointState& setpoint_){
        setpoint = setpoint_;
        newMsgOrState = true;
    }
    //get first PID command
    void firstPidCallback(const std_msgs::Float64& input){
        firstPidCommandValue = input.data;
        newMsgOrState = true;
    }
    //get second PID command
    void secondPidCallback(const std_msgs::Float64& input){
        secondPidCommandValue = input.data;
        newMsgOrState = true;
    }
    void rawJointStateCallback(const sensor_msgs::JointState& jointState){

    }
    void compute(){
        if(!newMsgOrState) return;
        //jointState, setpoint, firstPidCommandValue, secondPidCommandValue
        //firstJointCommandPub.publish, secondJointCommandPub.publish

        /* use M and N */

        newMsgOrState = false;
    }
private:
    //ROS Node stuff
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher firstJointCommandPub, secondJointCommandPub;
    ros::Subscriber setpointSub, rawJointStateSub, firstPidSubscriber, secondPidSubscriber;
    //Parameters
    std::string jointStateTopic, setpointTopicName,
                firstPidCommandTopic, secondPidCommandTopic,
                firstJointCommandTopic, secondJointCommandTopic;
    //Values
    sensor_msgs::JointState jointState, setpoint;
    float firstPidCommandValue{0.0}, secondPidCommandValue{0.0};
    bool newMsgOrState{false};
};

int main(int argc, char** argv){
    //Initialize ROS Node
    ros::init(argc,argv,"ct_controller");

    CT_controller ct_controller;

    //loop
    double f = 10; //rate frequency
    ros::Rate r(f);

    while (ros::ok()){
        //Compute output
        ct_controller.compute();

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

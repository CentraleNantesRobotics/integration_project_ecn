#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>
#include <sstream>
#include <iostream>


#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
//inutile mais peut servir pour creer nos propres messages
//#include <Suivi_traj_EK/Commande.h>


using namespace std;

// global variables for subscriber
sensor_msgs::JointState robot_state;
sensor_msgs::JointState robot_trajectory;
std_msgs::Float64 torque_q1_command, torque_q2_command;

void robot_stateCallback(const sensor_msgs::JointStatePtr & msg)
{
    robot_state = *msg;
}

void robot_trajectoryCallback(const sensor_msgs::JointStatePtr & msg)
{
    robot_trajectory = *msg;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "PID_controller");
    ros::NodeHandle nh;

    // subscriber robot_state
    ros::Subscriber robot_state_sub = nh.subscribe ("/joint_states", 10, robot_stateCallback);

    // subscriber Trajectoire
    ros::Subscriber robot_trajectory_sub = nh.subscribe ("/trajectory", 10, robot_trajectoryCallback);

    // publisher effort q1
    ros::Publisher torque1_publisher = nh.advertise<std_msgs::Float64>("/shoulder_effort_controller/command", 10);

    // publisher effort q2
    ros::Publisher torque2_publisher = nh.advertise<std_msgs::Float64>("/elbow_effort_controller/command", 10);

    // choice of PID coefficients
    float Kp0=120, Kp1=100, Kd0=0.1, Kd1=0.1, Ki0=0.1, Ki1=0.1, Te=0.01;


    float integral_error[2] = {};
    float position_error[2] = {}, velocity_error[2] = {};


    ros::Rate rate(1/Te);

    robot_state.position.resize(2);
    robot_state.velocity.resize(2);

    robot_trajectory.position.resize(2);
    robot_trajectory.velocity.resize(2);
}


    while (ros::ok())
    {
        //errors for the first motor (shoulder)
        position_error[0] = robot_trajectory.position[0] - robot_state.position[0];
        velocity_error[0] = robot_trajectory.velocity[0] - robot_state.velocity[0];
        integral_error[0] += position_error[0]*Te;

        //calculus for the first torque (shoulder)
        torque_q1_command.data = Kp0*position_error[0] + Kd0*velocity_error[0]/Te + Ki0*integral_error[0];

        //cout<<"commande P: "<<Kp0*position_error[0]<<endl;
        //cout<<"commande D: "<<Kd0*velocity_error[0]/Te<<endl;
        //cout<<"commande I: "<<Ki0*integral_error[0]<<endl;

        //errors for the second motor (elbow)
        position_error[1] = robot_trajectory.position[1] - robot_state.position[1];
        velocity_error[1] = robot_trajectory.velocity[1] - robot_state.velocity[1];
        integral_error[1] += position_error[1]*Te;

        //calculus for the second torque (shoulder)
        torque_q2_command.data = Kp1*position_error[1] + Kd1*velocity_error[1]/Te + Ki1*integral_error[1];
;

        //cout<<"commande P: "<<Kp1*position_error[1]<<endl;
        //cout<<"commande D: "<<Kd1*velocity_error[1]/Te<<endl;
        //cout<<"commande I: "<<Ki1*integral_error[1]<<endl;


        //cout<<"robot_state position q1: "<<robot_state.position[0]<<endl;
        //cout<<"robot_state position q2: "<<robot_state.position[1]<<endl;

        //cout<<"error position q1: "<<position_error[0]<<endl;
        //cout<<"error position q2: "<<position_error[1]<<endl;
        
        
        // publish setpoint
        torque1_publisher.publish(torque_q1_command);
        torque2_publisher.publish(torque_q2_command);

        ros::spinOnce();
        rate.sleep();
    return 0;
    }



#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>
#include <sstream>
#include <iostream>
#include <control_toolbox/SetPidGains.h>
#include <cmath>

//#include <gkd_models/Dynamic.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/JointState.h>
//inutile mais peut servir pour creer nos propres messages
//#include <Suivi_traj_EK/Commande.h>


using namespace std;
using namespace control_toolbox;

// global variables for subscriber
sensor_msgs::JointState robot_state;
sensor_msgs::JointState robot_trajectory;
sensor_msgs::JointState commande;
std_msgs::Float64 torque_q1_command;
std_msgs::Float64 torque_q2_command;
sensor_msgs::JointState jt_state;
//gkd_models::Dynamic srv;
control_toolbox::SetPidGainsRequest gains;


void robot_stateCallback(const sensor_msgs::JointStatePtr & msg)
{
    robot_state = *msg;
}

void robot_trajectoryCallback(const sensor_msgs::JointStatePtr & msg) //changer de message
{
    robot_trajectory = *msg;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "CTC_controller");
    ros::NodeHandle nh;

    // subscriber Etat
    // Position, velocity, acceleration of each motors
    ros::Subscriber robot_state_sub = nh.subscribe ("/joint_states", 10, robot_stateCallback);

    // subscriber Trajectoire
    // Position, velocity, acceleration desired of each motors
    ros::Subscriber robot_trajectory_sub = nh.subscribe ("/trajectory", 10, robot_trajectoryCallback);

    // ros::ServiceClient client = nh.serviceClient<>("");
    // publisher effort q1 (Torque of first motor)
    ros::Publisher torque1_publisher = nh.advertise<std_msgs::Float64>("/shoulder_effort_controller/command", 10);

    // publisher effort q2 (Torque of second motor)
    ros::Publisher torque2_publisher = nh.advertise<std_msgs::Float64>("/elbow_effort_controller/command", 10);


    float Te=0.01;

    // choice of PID coefficients (Index 1 for first motor, Index 2 for second motors)
    float Kp1=120, Kp2=100, Kd1=0.1, Kd2=0.1, Ki1=0.1, Ki2=0.1;

    float integral_error[2] = {};
    float position_error[2] = {}, velocity_error[2] = {};
    float sum_before_matrices[2]={};

    ros::Rate rate(1/Te);

    torque_q1_command.data = 0;
    torque_q2_command.data = 0;

    //effort = acceleration

    robot_state.position.resize(2);
    robot_state.velocity.resize(2);
    robot_state.effort.resize(2);

    robot_trajectory.position.resize(2);
    robot_trajectory.velocity.resize(2);
    robot_trajectory.effort.resize(2);

    //calculus of errors sums

    position_error[0] = robot_trajectory.position[0] - robot_state.position[0];
    velocity_error[0] = robot_trajectory.velocity[0] - robot_state.velocity[0];
    integral_error[0] += position_error[0]*Te;

    position_error[1] = robot_trajectory.position[1] - robot_state.position[1];
    velocity_error[1] = robot_trajectory.velocity[1] - robot_state.velocity[1];
    integral_error[1] += position_error[1]*Te;

    sum_before_matrices[0]=Kp1*position_error[0] + Kd1*velocity_error[0]/Te + Ki1*integral_error[0]+robot_trajectory.effort[0];
    sum_before_matrices[1]=Kp2*position_error[1] + Kd2*velocity_error[1]/Te + Ki2*integral_error[1]+robot_trajectory.effort[1];

/*
    //ask for the matrices M and N
    N=...;
    M=...;

    //compute the torque
    torque_q1_command = ;
    torque_q2_command = ;
    */

    // publish setpoint
    torque1_publisher.publish(torque_q1_command);
    torque2_publisher.publish(torque_q2_command);

    ros::spinOnce();
    rate.sleep();
    return 0;
}

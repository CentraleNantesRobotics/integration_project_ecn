#include <ros/ros.h>
#include <integ_trajectory_generation/generation_node.h>
#include <geometry_msgs/Pose2D.h>
#include <stdlib.h>

GenerationNode::GenerationNode()
{
    // Freq
    freq_ = 100;

    // Member initialisations
    publisher_ = nh_.advertise<sensor_msgs::JointState>("trajectory", 1000);
    state_subscriber_ = nh_.subscribe("tf", 1000, &GenerationNode::stateSubscribingCallback, this);
    waypoint_subscriber_ = nh_.subscribe("waypoints", 1000, &GenerationNode::waypointSubscribingCallback, this);

    // Timer initialisations (pb sur les timers)
    //publishingTimer_ = nh_.createTimer(ros::Duration(1/freq_), &GenerationNode::publishingCallback);
    //computingTimer_ = nh_.createTimer(ros::Duration(1.00), &GenerationNode::computingCallback);

    // Waypoint et Buffer
    current_waypoint_.x = 5.0;
    current_waypoint_.y = 5.0;
    buffer_.push_back({0, 0});

}

void GenerationNode::stateSubscribingCallback(const sensor_msgs::JointState& msg)
{
    q_ = msg.position;
    dq_ = msg.velocity;
}

void GenerationNode::waypointSubscribingCallback(const geometry_msgs::Pose2D& msg)
{
    current_waypoint_.x = msg.x;
    current_waypoint_.y = msg.y;
}

void GenerationNode::computingCallback()
{
    // COEUR DE L'ALGORITHME
}

void GenerationNode::publishingCallback()
{
    // Stamp
    //current_trajectory_[0].header.stamp = ros::Time::now();

    // Publish
    //publisher_.publish(current_trajectory_[0]);

    // Remove
    //current_trajectory_.erase(current_trajectory_.begin());

}





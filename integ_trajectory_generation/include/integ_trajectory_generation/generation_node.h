#ifndef GENERATION_NODE_H
#define GENERATION_NODE_H

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <vector>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <math.h>


// Useful structure
struct Vector2f
{
    float x;
    float y;
};



class GenerationNode
{

public:
    // Construction
    GenerationNode();

    // Callbacks
    void stateSubscribingCallback(const sensor_msgs::JointState&);
    void waypointSubscribingCallback(const geometry_msgs::Pose2D&);
    void publishingCallback();
    void computingCallback();


private:
    // Buffer
    std::vector<Vector2f> buffer_ ;

    // Variables
    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> ddq_;
    geometry_msgs::Pose2D current_waypoint_;
    std::vector<sensor_msgs::JointState> current_trajectory_;

    // Parameters
    double freq_;
    double vmax1, vmax2;
    double amax1, amax2;

    // NH
    ros::NodeHandle nh_;
    // Members
    ros::Publisher publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber waypoint_subscriber_;
    // Timers
    ros::Timer publishingTimer_;
    ros::Timer computingTimer_;

    // MGD + MGI
    float l1_, l2_;

    void mgd(const float &q1, const float &q2, float &x, float &y)
    {
        x = l1_*std::cos(q1) + l2_*std::cos(q1 + q2);
        y = l1_*std::sin(q1) + l2_*std::sin(q1 + q2);
    };

    void mgi(const float &x, const float &y, float &q1, float &q2)
    {
        q2 = std::acos((l1_*l1_ + l2_*l2_)/(2*l1_*l2_));
        q1 = std::atan2(y, x) - q2 ;
    };

};

#endif // GENERATION_NODE_H


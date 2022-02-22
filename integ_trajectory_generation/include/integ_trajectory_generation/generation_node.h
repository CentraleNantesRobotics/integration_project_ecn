#ifndef GENERATION_NODE_H
#define GENERATION_NODE_H

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <vector>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <stdlib.h>


class GenerationNode
{

public:
    /******** Construction ********/
    GenerationNode();

    /******** Callbacks ********/
    void stateSubscribingCallback(const sensor_msgs::JointState&);
    void waypointSubscribingCallback(const geometry_msgs::Pose2D&);
    void publishingCallback();
    void computingCallback(const int, const int);

    /******** Utilities ********/
    void compute_tf();
    void compute_ta();
    void compute_td();
    void compute_ti();
    void rearrangeTimes();
    void nextWaypoint_update();
    bool waypointReached();

    /******** MGD + MGI ********/
    std::vector<double> mgd(const std::vector<double> &q)
    {
        double x = l1_*std::cos(q[0]) + l2_*std::cos(q[0] + q[1]);
        double y = l1_*std::sin(q[0]) + l2_*std::sin(q[0] + q[1]);

        return {x, y};
    };

    std::vector<double> mgi(const geometry_msgs::Pose2D &X)
    {
        double q2 = std::acos((l1_*l1_ + l2_*l2_)/(2*l1_*l2_));
        double q1 = std::atan2(X.x, X.y) - q2 ;

        return {q1, q2};
    };

private:

    /******** Buffers ********/
    std::vector<std::vector<double>> positions_buffer_ ;
    std::vector<std::vector<double>> velocities_buffer_;
    std::vector<std::vector<double>> accelerations_buffer_;

    /******** Variables ********/
    sensor_msgs::JointState current_joints_states_;
    sensor_msgs::JointState next_joints_states_;
    geometry_msgs::Pose2D current_waypoint_;
    std::vector<double> tf_, ta_, td_, ti_;
    std::vector<int> config_;
    std::vector<double> vmax_temp_;
    std::vector<double> amax_temp_;
    double timeSinceArrival_;

    /******** Parameters ********/

    // Definition of arms lengths
    const double l1_ = 0.5;
    const double l2_ = 0.5;

    // Definition of frequencies and durations
    const double publishing_freq_ = 1000;
    const double publishing_duration_ = 1/publishing_freq_;
    const double computing_freq_ = 1000;
    const double computing_duration_ = 1/computing_freq_;

    // Definition of the tresholds : we consider that a number is null if it is smaller than the threshold
    const double time_threshold_ = 0.05;
    const double metric_threshold_  = 0.01;

    // Definition of maximal velocities and accelerations
    const std::vector<double> vmax_ = {10, 10};
    const std::vector<double> amax_ = {10, 10};

    // Definition of some constants indicating the type of the function followed by the angular speed of a joint
    const int TRAPEZOIDAL = 0;
    const int BANGBANG = 1;

    /******** ROS Stuff ********/

    // Node Handle
    ros::NodeHandle nh_;
    // Members
    ros::Publisher publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber waypoint_subscriber_;
    // Timers
    ros::Timer publishingTimer_;
    ros::Timer computingTimer_;



};

#endif // GENERATION_NODE_H


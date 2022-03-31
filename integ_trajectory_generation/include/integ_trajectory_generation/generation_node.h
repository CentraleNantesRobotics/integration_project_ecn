/* -------------------------------------------------------------------
 * This file defines the GenerationNode class, along with two basic
 * functions that are not inherent to this node.
 * -------------------------------------------------------------------*/

#ifndef GENERATION_NODE_H
#define GENERATION_NODE_H

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <vector>
#include <iostream>
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
    void computingCallback();


    /******** Utilities ********/
    std::vector<double> mgd(const std::vector<double>&);
    std::vector<double> mgi(const geometry_msgs::Pose2D&);
    void compute_tf();
    void compute_ta();
    void compute_td();
    void compute_ti();
    void rearrangeTimes();
    bool waypointReached();
    void popBuffers();
    void nextWaypoint_update();


    /******** Public parameters ********/

    // Definition of frequencies and durations
    const double publishing_freq_ = 100;
    const double publishing_duration_ = 1/publishing_freq_;

private:

    /******** Private parameters ********/

    // Definition of arms lengths
    const double l1_ = 0.8;
    const double l2_ = 0.6;

    // Definition of the tresholds : we consider that a number is null if it is smaller than the threshold
    const double time_threshold_ = 0.05;
    const double metric_threshold_  = 0.01;

    // Definition of maximal velocities and accelerations
    const std::vector<double> vmax_ = {5, 5};
    const std::vector<double> amax_ = {5, 5};

    // Definition of some constants indicating the type of the function followed by the angular speed of a joint
    const int TRAPEZOIDAL = 0;
    const int BANGBANG = 1;


    /******** Buffers ********/
    std::vector<std::vector<double>> positions_buffer_ ;
    std::vector<std::vector<double>> velocities_buffer_;
    std::vector<std::vector<double>> accelerations_buffer_;


    /******** Variables and their initial value ********/
    sensor_msgs::JointState current_joints_states_;
    sensor_msgs::JointState next_joints_states_;
    geometry_msgs::Pose2D current_waypoint_;
    std::vector<double> tf_, ta_, td_, ti_;
    std::vector<int> config_ = {TRAPEZOIDAL, TRAPEZOIDAL};
    std::vector<double> vmax_temp_ = vmax_;
    std::vector<double> amax_temp_ = amax_;
    std::vector<double> dmax_temp_ = {-amax_[0], -amax_[1]};
    double timeSinceArrival_ = time_threshold_;
    bool joints_states_init = false;


    /******** ROS Stuff ********/

    // Node Handle
    ros::NodeHandle nh_;

    // Members
    ros::Publisher state_publisher_;
    ros::Publisher trajectory_publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber waypoint_subscriber_;

};


// Other basic functions that aren't inherent to the GeneratioNode class
int findIndex(const std::string& , const std::vector<std::string>&);
std::vector<double> substractVectors(const std::vector<double>&, const std::vector<double>&);

#endif // GENERATION_NODE_H


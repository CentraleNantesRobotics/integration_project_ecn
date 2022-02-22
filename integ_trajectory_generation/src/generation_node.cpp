#include <integ_trajectory_generation/generation_node.h>

GenerationNode::GenerationNode()
{
    /******** Variables initialisation ********/
    for (auto state : {current_joints_states_ , next_joints_states_}){
        state.name = {"arm1", "arm2"};
        state.position = {0, 0};
        state.velocity = {0, 0};
        state.effort = {0, 0};
    }
    config_ = {TRAPEZOIDAL, TRAPEZOIDAL};
    timeSinceArrival_ = 0;
    vmax_temp_ = vmax_;
    amax_temp_ = amax_;

    /******** ROS Stuff initialisations ********/
    publisher_ = nh_.advertise<sensor_msgs::JointState>("trajectory", 1000);
    state_subscriber_ = nh_.subscribe("tf", 1000, &GenerationNode::stateSubscribingCallback, this);
    waypoint_subscriber_ = nh_.subscribe("waypoints", 1000, &GenerationNode::waypointSubscribingCallback, this);
    // TO DO : Problème sur timers
    // publishingTimer_ = nh_.createTimer(ros::Duration(publishing_duration_), &GenerationNode::publishingCallback);
    // computingTimer_ = nh_.createTimer(ros::Duration(computing_duration_), &GenerationNode::computingCallback);


    /******** While there's no actual subscription ********/
    // Waypoint
    current_waypoint_.x = 5.0;
    current_waypoint_.y = 5.0;
    // Buffer initialisations
    positions_buffer_.push_back(mgi(current_waypoint_));
    velocities_buffer_.push_back({0, 0});
    accelerations_buffer_.push_back({0, 0});
    // Times initialisations
    compute_tf();
    compute_ta();
    compute_td();
    compute_ti();

}

void GenerationNode::stateSubscribingCallback(const sensor_msgs::JointState& msg)
{
    current_joints_states_ = msg;
}

void GenerationNode::waypointSubscribingCallback(const geometry_msgs::Pose2D& msg)
{
    current_waypoint_ = msg;
    positions_buffer_.push_back(mgi(current_waypoint_));
    // Might change but for now we want a null velocity and acceleration when arriving at each waypoint
    velocities_buffer_.push_back({0, 0});
    accelerations_buffer_.push_back({0, 0});
}

void GenerationNode::nextWaypoint_update()
{
    // Compute times
    compute_tf();
    compute_ta();
    compute_td();
    compute_ti();

    // Rearrange times
    rearrangeTimes();

    // Reset time since arrival
    timeSinceArrival_ = 0;
}

void GenerationNode::publishingCallback()
{
    if (waypointReached()) {
        nextWaypoint_update();
    }else{
        timeSinceArrival_ += publishing_duration_;
    }

    // Compute next joints states for trajectory tracking step
    if (tf_[0] < tf_[1]){
        computingCallback(0, 1);
    }else{
        computingCallback(1, 0);
    }

    // Stamp
    next_joints_states_.header.stamp = ros::Time::now();

    // Publish
    publisher_.publish(next_joints_states_);

}

#include <integ_trajectory_generation/generation_node.h>

GenerationNode::GenerationNode()
{
    /******** Variables initialisation ********/
    current_joints_states_.name = {"shoulder", "elbow"};
    current_joints_states_.position = {0, 0};
    current_joints_states_.velocity = {0, 0};
    current_joints_states_.effort = {0, 0};

    next_joints_states_.name = {"shoulder", "elbow"};
    next_joints_states_.position = {0, 0};
    next_joints_states_.velocity = {0, 0};
    next_joints_states_.effort = {0, 0};

    config_ = {TRAPEZOIDAL, TRAPEZOIDAL};
    timeSinceArrival_ = 0;
    vmax_temp_ = vmax_;
    amax_temp_ = amax_;
    dmax_temp_ = {-amax_[0], -amax_[1]};

    /******** ROS Stuff initialisations ********/
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/state", 1000);
    trajectory_publisher_ = nh_.advertise<sensor_msgs::JointState>("/trajectory", 1000);
    state_subscriber_ = nh_.subscribe("/tf", 1000, &GenerationNode::stateSubscribingCallback, this);
    waypoint_subscriber_ = nh_.subscribe("waypoints", 1000, &GenerationNode::waypointSubscribingCallback, this);

    /******** While there are no actual subscriptions ********/
    // Waypoint
    auto temp = mgd({0.5, 0.8});
    current_waypoint_.x = temp[0];
    current_waypoint_.y = temp[1];
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
    current_joints_states_.position = msg.position;
    current_joints_states_.velocity = msg.velocity;
    current_joints_states_.effort = msg.effort;
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
    timeSinceArrival_ = time_threshold_;
}


void GenerationNode::publishingCallback()
{

    if ( (positions_buffer_.size() == 0) || (waypointReached() && (positions_buffer_.size() == 1)) ) {
        next_joints_states_ = current_joints_states_;
    } else if (waypointReached()) {
        nextWaypoint_update();
    } else {
        timeSinceArrival_ += publishing_duration_;
    }

    // Compute next joints states for trajectory tracking step
    computingCallback();

    // Stamp
    current_joints_states_.header.stamp = ros::Time::now();
    next_joints_states_.header.stamp = ros::Time::now();

    // Publish
    state_publisher_.publish(current_joints_states_);
    trajectory_publisher_.publish(next_joints_states_);
}

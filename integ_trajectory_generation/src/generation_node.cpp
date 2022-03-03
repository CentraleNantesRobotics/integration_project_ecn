#include <integ_trajectory_generation/generation_node.h>

GenerationNode::GenerationNode()
{
    /******** Variables initialisation ********/
    for (auto state : {&current_joints_states_, &next_joints_states_}){
        state->name = {"elbow", "shoulder"};
        state->position.resize(2);
        state->velocity.resize(2);
        state->effort.resize(2);
    }

    config_ = {TRAPEZOIDAL, TRAPEZOIDAL};
    timeSinceArrival_ = 0;
    vmax_temp_ = vmax_;
    amax_temp_ = amax_;
    dmax_temp_ = {-amax_[0], -amax_[1]};

    for (auto time: {&tf_, &ta_, &td_, &ti_}){
        time->resize(2);
    }


    /******** ROS Stuff initialisations ********/
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/state", 1000);
    trajectory_publisher_ = nh_.advertise<sensor_msgs::JointState>("/trajectory", 1000);
    state_subscriber_ = nh_.subscribe("/joint_states", 1000, &GenerationNode::stateSubscribingCallback, this);
    waypoint_subscriber_ = nh_.subscribe("waypoints", 1000, &GenerationNode::waypointSubscribingCallback, this);

    /******** While there are no actual waypoint subscriptions ********/
    // Waypoint
    std::vector<double> temp = {1, 1};
    current_waypoint_.x = temp[0];
    current_waypoint_.y = temp[1];
    // Buffer initialisations
    positions_buffer_.push_back(mgi(current_waypoint_));
    velocities_buffer_.push_back({0, 0});
    accelerations_buffer_.push_back({0, 0});

}

void GenerationNode::stateSubscribingCallback(const sensor_msgs::JointState& msg)
{
    for(int i = 0; i < 2; i++)
    {
        const int idx{findIndex(current_joints_states_.name[i], msg.name)};

        current_joints_states_.position[i] = msg.position[idx];
        current_joints_states_.velocity[i] = msg.velocity[idx];
        current_joints_states_.effort[i] = msg.effort[idx];
    }

    if (!joint_state_received)
        joint_state_received = true;
}


void GenerationNode::waypointSubscribingCallback(const geometry_msgs::Pose2D& msg)
{
    // Might change but for now we want a null velocity and acceleration when arriving at each waypoint
    current_waypoint_ = msg;
    positions_buffer_.push_back(mgi(current_waypoint_));
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
    if (joint_state_received){

        if (positions_buffer_.size() == 0) {
            // No waypoint was given
            next_joints_states_ = current_joints_states_;

        } else if (waypointReached()){

            if (positions_buffer_.size() == 1) {
                // Last waypoint was reached
                popBuffers();
                next_joints_states_ = current_joints_states_;
                timeSinceArrival_ = time_threshold_;

            } else {
                // There is another one
                popBuffers();
                nextWaypoint_update();
                computingCallback();

            }

        } else {
            // 'Normal' situation
            timeSinceArrival_ += publishing_duration_;
            computingCallback();
        }

        // Stamp
        current_joints_states_.header.stamp = ros::Time::now();
        next_joints_states_.header.stamp = ros::Time::now();

        // Publish
        state_publisher_.publish(current_joints_states_);
        trajectory_publisher_.publish(next_joints_states_);
    }
}

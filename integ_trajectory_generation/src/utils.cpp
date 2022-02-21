/* -------------------------------------------------------------------
 * This document is an utility document, which gathers several utility
 * functions needed for integ_trajectory_generation project.
 * -------------------------------------------------------------------*/

#include <integ_trajectory_generation/generation_node.h>

/* This function calculates the time tf needed for the arm to reach the goal angle, using the trapezoidal
 * definition of the angular speed or the bang-bang definition if the arm does not need to move enough.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point va
 *          - angular speed at the goal point vb
 *          - angle of the starting point qa
 *          - angle of the goal point qb
 * Output : time needed for the arm to reach the goal point tf   */

void GenerationNode::compute_tf()
{
    // Result
    std::vector<double> tf;

    // Compute
    std::vector<double> q_waypoint = positions_buffer_[0];
    std::vector<double> dq_waypoint = velocities_buffer_[0];
    std::vector<double> ddq_waypoint = accelerations_buffer_[0];

    for (int i=0; i<int(q_waypoint.size()); i++){

        const double qa = current_joints_states_.position[i];
        const double qb = q_waypoint[i];
        const double va = current_joints_states_.velocity[i];
        const double vb = velocities_buffer_[0][i];

        const float deltaQ = qb- qa;
        const float deltaQLim = (2*va*(vmax_[i]-va) + 2*vb*(vmax_[i]-vb) + (vb-va)*(2*vmax_[i]-vb-va))/(2*amax_[i]);

        if (deltaQ < deltaQLim){
            config_[i] = BANGBANG;
            tf.push_back(((2*vmax_[i]+va-vb) + 2*sqrt(pow((2*vmax_[i]-va-vb)/2,2) - pow(2*vmax_[i]-va-vb,2) - 5*amax_[i]*deltaQ))/(5*amax_[i]));
        }
        else
        {
            config_[i] = TRAPEZOIDAL;
            tf.push_back((deltaQ + (vb-va)*(2*vmax_[i]-vb-va))/(2*amax_[i]));
        }

    }

    tf_ = tf;
}


/* This function calculates the duration of acceleration ta for a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmaxq1
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point va
 * Output : duration of acceleration ta for a trapezoidal configuration   */

void GenerationNode::compute_ta()
{
    // Result
    std::vector<double> ta;

    for(int i=0; i<2; i++){
        const double va = current_joints_states_.velocity[i];
        ta.push_back((vmax_[i]-va)/amax_[i]);
    }

    ta_ = ta;
}


/* This function calculates the duration of deceleration tf-td for a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the goal point vb
 * Output : duration of deceleration tf-td for a trapezoidal configuration   */

void GenerationNode::compute_td()
{
    // Result
    std::vector<double> td;

    for(int i=0; i<2; i++){
        const double vb = velocities_buffer_[0][i];
        td.push_back(tf_[i] - (vmax_[i]-vb)/amax_[i]);
    }

    td_ = td;
}


/* This function calculates the duration of acceleration ti for a bang-bang configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point va
 *          - angular speed at the goal point vb
 *          - total time needed for the movemement tf
 * Output : duration of acceleration ti for a bang-bang configuration   */

void GenerationNode::compute_ti()
{
    // Result
    std::vector<double> ti;

    for(int i=0; i<2; i++){
        const double va = current_joints_states_.velocity[i];
        const double vb = velocities_buffer_[0][i];
        ti.push_back((2*vmax_[i]-va-vb)/amax_[i] - tf_[i]/2);
    }

    ti_ = ti;
}


// TO DO : Corps de fonction + Header
void GenerationNode::rearrangeTimes()
{

}

// TO DO : Header
bool GenerationNode::waypointReached()
{
    std::vector<double> pose = mgd(current_joints_states_.position);
    double metric = sqrt( pow(pose[0]-current_waypoint_.x,2) + pow(pose[1]-current_waypoint_.y,2) );

    return (metric < metric_threshold_);
}

                                                                                                                                    // A FAIRE : écrire la description de la fonction
// TO DO : Header + T O U T
sensor_msgs::JointState GenerationNode::computingCallback(const int fastIndex, const int slowIndex)
{

    if (config_[0] == TRAPEZOIDAL && config_[1] == TRAPEZOIDAL)
    {
        for (int i=0; i<2; i++){

            if (ta_[i] > time_threshold_) //the joints are still in their accelerating phase
            {

                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * amax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2);

            } else if(td_[i] > time_threshold_) { //the joints are in the phase of constant angular speed

                next_joints_states_.velocity[i] = vmax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2);

                // TO DO : modifier les conditions qui vérifient dans quelle phase du trapèze on est

                v1 = vmax1;                                                                                                         // <-- ATTENTION : à modifier car pas vmax ici
                q1 = q1 + time_for_upload*v1;

                v2 = vmax2;
                q2 = q2 + time_for_upload*v2;
            } else { //the joints are in the decelarating phase
                float v1_temp = v1;
                v1 = v1 - time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q1 = q1 + time_for_upload*(v1+v1_temp)/2;

                float v2_temp = v2;
                v2 = v2 - time_for_upload*amax2;
                q2 = q2 + time_for_upload*(v2+v2_temp)/2;
            }

    } else if (config1 == BANGBANG && config2 == BANGBANG)
    {
        const float ti = compute_intermediate_time(vmax2, amax2, v2, v2fin, tf);

        if (ti > time_threshold) //the joints are still in their accelerating phase
        {
            float v1_temp = v1;
            v1 = v1 + time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
            q1 = q1 + time_for_upload*(v1+v1_temp)/2;

            float v2_temp = v2;
            v2 = v2 + time_for_upload*amax2;
            q2 = q2 + time_for_upload*(v2+v2_temp)/2;
        } else { //the joints are in the decelarating phase
            float v1_temp = v1;
            v1 = v1 - time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
            q1 = q1 + time_for_upload*(v1+v1_temp)/2;

            float v2_temp = v2;
            v2 = v2 - time_for_upload*amax2;
            q2 = q2 + time_for_upload*(v2+v2_temp)/2;
        }
    } else {
                                                                                                                                //  <-- A FAIRE : pas d'idée si profils différents
    }
}

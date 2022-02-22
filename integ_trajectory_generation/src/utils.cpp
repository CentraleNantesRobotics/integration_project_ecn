/* -------------------------------------------------------------------
 * This document is an utility document, which gathers several utility
 * functions needed for integ_trajectory_generation project.
 * -------------------------------------------------------------------*/

#include <integ_trajectory_generation/generation_node.h>

/* This function calculates the vector tf of respective time needed by each arm to reach it's goal angle, using the trapezoidal
 * definition of the angular speed or the bang-bang definition if the arm does not need to move enough.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 *          - angular speed at the goal point dqb
 *          - angle of the starting point qa
 *          - angle of the goal point qb
 * Output : vector tf of respective time needed by each arm to reach it's goal angle   */

void GenerationNode::compute_tf()
{
    // Result
    std::vector<double> tf;

    for (int i=0; i<2; i++){

        const double qa = current_joints_states_.position[i];
        const double qb = positions_buffer_[0][i];
        const double dqa = current_joints_states_.velocity[i];
        const double dqb = velocities_buffer_[0][i];
        // const double ddqa = current_joints_states_.effort[i];
        // const double ddqb = accelerations_buffer_[0][i];

        const float deltaQ = qb- qa;
        const float deltaQLim = (2*dqa*(vmax_[i]-dqa) + 2*dqb*(vmax_[i]-dqb) + (dqb-dqa)*(2*vmax_[i]-dqb-dqa))/(2*amax_[i]);

        if (deltaQ < deltaQLim){
            config_[i] = BANGBANG;
            tf.push_back(((2*vmax_[i]+dqa-dqb) + 2*sqrt(pow((2*vmax_[i]-dqa-dqb)/2,2) - pow(2*vmax_[i]-dqa-dqb,2) - 5*amax_[i]*deltaQ))/(5*amax_[i]));
        } else {
            config_[i] = TRAPEZOIDAL;
            tf.push_back((deltaQ + (dqb-dqa)*(2*vmax_[i]-dqb-dqa))/(2*amax_[i]));
        }

    }

    // Update tf_ attribute
    tf_ = tf;
}


/* This function calculates the vector ta of respective duration of acceleration for each arm in a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmaxq1
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 * Output : vector ta of respective duration of acceleration for each arm in a trapezoidal configuration */

void GenerationNode::compute_ta()
{
    // Result
    std::vector<double> ta;

    for(int i=0; i<2; i++){

        if (config_[i] == TRAPEZOIDAL){
            const double dqa = current_joints_states_.velocity[i];
            ta.push_back((vmax_[i]-dqa)/amax_[i]);
        } else {
            ta.push_back(-1);
        }
   }

    // Update ta_ attribute
    ta_ = ta;
}


/* This function calculates the vector td of respective duration until deceleration for each arm in a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the goal point dqb
 * Output : vector td of respective duration until deceleration for each arm in a trapezoidal configuration   */

void GenerationNode::compute_td()
{
    // Result
    std::vector<double> td;

    for(int i=0; i<2; i++){

        if (config_[i] == TRAPEZOIDAL) {
            const double dqb = velocities_buffer_[0][i];
            td.push_back(tf_[i] - (vmax_[i]-dqb)/amax_[i]);

        } else {
            td.push_back(-1);
        }
    }

    // Update td_ attribute
    td_ = td;
}


/* This function calculates the vector ti of respective duration of acceleration ti for a bang-bang configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 *          - angular speed at the goal point dqb
 *          - total time needed for the movemement tf
 * Output : duration of acceleration ti for a bang-bang configuration   */

void GenerationNode::compute_ti()
{
    // Result
    std::vector<double> ti;

    for(int i=0; i<2; i++){

        if (config_[i] == BANGBANG){
            const double dqa = current_joints_states_.velocity[i];
            const double dqb = velocities_buffer_[0][i];
            ti.push_back((2*vmax_[i]-dqa-dqb)/amax_[i] - tf_[i]/2);

        } else {
            ti.push_back(-1);
        }
    }

    // Update ti_ attribute
    ti_ = ti;
}


/* This function rearranges the tf, ta, td, and ti vectors and modifies the acceleration and velocity vectors accordingly.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 *          - angular speed at the goal point dqb
 *          - total time needed for the movemement tf
 * Output : - rearranged times
 *          - modified accelerations
 *          - modified velocities */

void GenerationNode::rearrangeTimes()
{

}

/* This function indicates if the next waypoint has been reached.
 *
 * Inputs : - current joints states
 *          - current waypoint
 *          - metric threshold
 * Output : true if the waypoint is reached, false otherwise   */

bool GenerationNode::waypointReached()
{
    std::vector<double> pose = mgd(current_joints_states_.position);
    double metric = sqrt( pow(pose[0]-current_waypoint_.x,2) + pow(pose[1]-current_waypoint_.y,2) );

    return (metric < metric_threshold_);
}

                                                                                                                                    // A FAIRE : écrire la description de la fonction
// TO DO : Header + T O U T
void GenerationNode::computingCallback(const int fastIndex, const int slowIndex)
{

    if (config_[0] == TRAPEZOIDAL && config_[1] == TRAPEZOIDAL)
    {
        for (int i : {fastIndex, slowIndex}){

            if (ta_[i] > time_threshold_) { //the joints are still in their accelerating phase

                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * amax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            } else if(td_[i] > time_threshold_) { //the joints are in the phase of constant angular speed

                next_joints_states_.velocity[i] = vmax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

                // TO DO : modifier les conditions qui vérifient dans quelle phase du trapèze on est

             } else { //the joints are in the decelarating phase

                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] - publishing_duration_ * amax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            }
        }

    } else if (config_[0] == BANGBANG && config_[1] == BANGBANG) {

        for (int i : {fastIndex, slowIndex}){

            if (ti_[i] > time_threshold_) //the joints are still in their accelerating phase
            {
                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * amax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

             } else if (ti_[i] < time_threshold_) { //the joints are in the decelarating phase

                next_joints_states_.velocity[i] = vmax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            } else {
                // TO DO:

                // next_joints_states_.velocity[i] = current_joints_states_.velocity[i] - publishing_duration_ * amax_temp_[i];
                // next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            }                                                                                                           //  <-- A FAIRE : pas d'idée si profils différents
        }
    } else {

        // TO DO:
    }
}

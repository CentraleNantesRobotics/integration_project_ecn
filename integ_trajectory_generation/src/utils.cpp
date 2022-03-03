/* -------------------------------------------------------------------
 * This document is an utility document, which gathers several utility
 * functions needed for integ_trajectory_generation project.
 * -------------------------------------------------------------------*/

#include <integ_trajectory_generation/generation_node.h>


/* This function calculates the vector tf of respective time needed by each joint to reach it's goal angle, using the trapezoidal
 * definition of the angular speed or the bang-bang definition if the joint does not need to move enough.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 *          - angular speed at the goal point dqb
 *          - angle of the starting point qa
 *          - angle of the goal point qb
 * Output : vector tf of respective time needed by each joint to reach it's goal angle   */

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




/* This function calculates the vector ta of respective duration of acceleration for each joint in a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmaxq1
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 * Output : vector ta of respective duration of acceleration for each joint in a trapezoidal configuration */

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




/* This function calculates the vector td of respective duration until deceleration for each joint in a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the goal point dqb
 * Output : vector td of respective duration until deceleration for each joint in a trapezoidal configuration   */

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



/* This function subtracts two vectors, value by value.
 *
 * Inputs : two vectors
 * Output : the vector of their substraction.            */

std::vector<double> substractVectors(const std::vector<double> &vec1, const std::vector<double> &vec2){
    std::vector<double> vecSub;

    if (vec1.size()==vec2.size()){
        for (int i=0; i<int(vec1.size()); i++){
            vecSub[i] = vec1[i] - vec2[i];
        }
    } else {
        std::cout << "Erreur dans les dimensions de vecteur" << std::endl;
    }

    return vecSub;
}



/* This function rearranges the tf, ta, td, and ti vectors and modifies the maximum acceleration and velocity vectors accordingly.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point dqa
 *          - angular speed at the goal point dqb
 *          - total time needed for the movemement tf
 * Output : - rearranged times
 *          - modified max accelerations
 *          - modified max velocities */

void GenerationNode::rearrangeTimes()
{

    // Results
    std::vector<double> tf, ta, td, ti;

    if (config_[0] == TRAPEZOIDAL && config_[1] == TRAPEZOIDAL){

        for (int i=0; i<2; i++){
            ta[i] = *std::max_element(ta_.begin(), ta_.end());
            td[i] = ta[i] + *std::max_element(substractVectors(td_, ta_).begin(), substractVectors(td_, ta_).end());
            tf[i] = td[i] + *std::max_element(substractVectors(tf_, td_).begin(), substractVectors(tf_, td_).end());

            dmax_temp_[i] = 1/((tf[i]-td[i])*(-3*ta[i]/2 + td[i]/2 + tf[i]/2)) *
                            (current_joints_states_.velocity[i]*(ta[i]-ta_[i]) + ta[i]/2 * (velocities_buffer_[0][i]-current_joints_states_.velocity[i]) +
                            amax_temp_[i]/2 * (pow(tf_[i],2) - pow(td_[i],2) - pow(ta_[i],2)) + velocities_buffer_[0][i] * (tf[i] - ta[i]) - vmax_temp_[i] * (td_[i]-ta_[i])
                            - (tf_[i] - td_[i]) * (velocities_buffer_[0][i] + amax_temp_[i]*tf_[i]));

            amax_temp_[i] = (velocities_buffer_[0][i] - current_joints_states_.velocity[i] - dmax_temp_[i] * (tf[i] - td[i])) / ta[i];

            vmax_temp_[i] = velocities_buffer_[0][i] - dmax_temp_[i] * (tf[i] - td[i]);
        }

    } else if (config_[0] == BANGBANG && config_[1] == BANGBANG){

        for (int i=0; i<2; i++){
            ti[i] = *std::max_element(ti_.begin(), ti_.end());
            tf[i] = ti[i] + *std::max_element(substractVectors(tf_, ti_).begin(), substractVectors(tf_, ti_).end());

            dmax_temp_[i] = 2/(tf[i] * (tf[i] + 3*ti[i])) *
                            (amax_temp_[i]*(pow(ti_[i],2)+pow(tf_[i]+ti_[i],2))/2 - ti[i]*(velocities_buffer_[0][i]-current_joints_states_.velocity[i])/2
                            - current_joints_states_.velocity[i] * (ti[i] - ti_[i]) - velocities_buffer_[0][i] * (tf[i] - ti[i] - tf_[i] +ti_[i]));

            amax_temp_[i] = (velocities_buffer_[0][i] - current_joints_states_.velocity[i] - dmax_temp_[i] * (tf[i] - ti[i])) / ti[i];

        }

    } else {

        // A triangle basically is a trapeze without a plateau
        int index_B;
        if (config_[0] == BANGBANG){
            index_B = 0;
        }else{
            index_B = 1;
        }

        config_[0] = TRAPEZOIDAL;
        vmax_temp_[index_B] = current_joints_states_.velocity[index_B] + amax_temp_[index_B]*ti_[index_B];
        ta_[index_B] = ti_[index_B];
        td_[index_B] = ti_[index_B];
    }

    // Update members
    tf_ = tf;
    ta_ = ta;
    td_ = td;
    ti_ = ti;

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


/* This function removes the first element of the member buffers.
 *
 * Inputs : said buffers
 * Output : modified buffers                                   */

void GenerationNode::popBuffers()
{
    positions_buffer_.erase(positions_buffer_.begin());
    velocities_buffer_.erase(velocities_buffer_.begin());
    accelerations_buffer_.erase(accelerations_buffer_.begin());
}


/* This function finds the index corresponding to an element in a vector
 *
 * Inputs : - element looked for
 *          - vector analysed
 * Output : the index corresponding to this element or the size of the vector if it wasn't found */

int GenerationNode::findIndex(const std::string &name, const std::vector<std::string> & names)
{
  const auto elem = std::find(names.begin(), names.end(), name);
  return std::distance(names.begin(), elem);
}


/* This function computes the next joints states according to the configurations and the various vectors.
 *
 * Inputs : - current joint states
 *          - rearranged times
 *          - modified max accelerations
 *          - modified max velocities
 * Output : next_joints_states   */

void GenerationNode::computingCallback()
{

    if (config_[0] == TRAPEZOIDAL && config_[1] == TRAPEZOIDAL)
    {
        for (int i=0; i<2; i++){

            if (ta_[i] > timeSinceArrival_) { //the joints are still in their accelerating phase

                next_joints_states_.effort[i] = amax_temp_[i];
                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * amax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            } else if(td_[i] > timeSinceArrival_) { //the joints are in the phase of constant angular speed

                next_joints_states_.effort[i] = 0;
                next_joints_states_.velocity[i] = vmax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

             } else { //the joints are in the decelerating phase

                next_joints_states_.effort[i] = dmax_temp_[i];
                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * dmax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            }
        }

    } else if (config_[0] == BANGBANG && config_[1] == BANGBANG) {

        for (int i=0; i<2; i++){

            if (ti_[i] > timeSinceArrival_) //the joints are still in their accelerating phase
            {
                next_joints_states_.effort[i] = amax_temp_[i];
                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * amax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

             } else { //the joints are in the decelarating phase

                next_joints_states_.effort[i] = dmax_temp_[i];
                next_joints_states_.velocity[i] = current_joints_states_.velocity[i] + publishing_duration_ * dmax_temp_[i];
                next_joints_states_.position[i] = current_joints_states_.position[i] + publishing_duration_ * (next_joints_states_.velocity[i] + current_joints_states_.velocity[i])/2;

            }
        }

    } else {

        std::cout << "Odd" << std::endl;

    }
}




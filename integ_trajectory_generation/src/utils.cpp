/* -------------------------------------------------------------------
 * This document is an utility document, which gathers several utility
 * functions needed for integ_trajectory_generation project.
 * -------------------------------------------------------------------*/

#include <cmath>

using namespace std;

// Definition of some constants indicating the type of the function followed by the angular speed of a joint
const int TRAPEZOIDAL = 0;
const int BANGBANG = 1;

// Definition of the frequency of publishing and the corresponding time between two publications
const int frequency_of_upload = 50;                                                                      //  <-- needs to be modified to correspond to the value indicated in the node
const float time_for_upload = 1/frequency_of_upload;

// Definition of the tresholds : we consider that a number is null if it is smaller than the threshold
const float time_threshold = 0.05;                                                                       //  <-- needs to be modified to improve precision

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

float compute_tf(const float &vmax, const float &amax, const float &va, const float &vb, const float &qa, const float &qb)
{
    const float deltaQ = qb - qa;
    const float deltaQLim = (2*va*(vmax-va) + 2*vb*(vmax-vb) + (vb-va)*(2*vmax-vb-va))/(2*amax);

    if (deltaQ < deltaQLim)
    {
        return ((2*vmax+va-vb) + 2*sqrt(pow((2*vmax-va-vb)/2,2) - pow(2*vmax-va-vb,2) - 5*amax*deltaQ))/(5*amax);
    } else {
        return (deltaQ + (vb-va)*(2*vmax-vb-va)/(2*amax));
    }
}


/* This function calculates the duration of acceleration t1 for a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmaxq1
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point va
 * Output : duration of acceleration t1 for a trapezoidal configuration   */

float compute_acceleration_duration(const float &vmax, const float &amax, const float &va)
{
    return (vmax-va)/amax;
}


/* This function calculates the duration of deceleration tf-t2 for a trapezoidal configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the goal point vb
 * Output : duration of deceleration tf-t2 for a trapezoidal configuration   */

float compute_deceleration_duration(const float &vmax, const float &amax, const float &vb)
{
    return (vmax-vb)/amax;
}


/* This function calculates the duration of acceleration ti for a bang-bang configuration.
 *
 * Inputs : - max angular speed of the joint vmax
 *          - max angular acceleration of the joint amax
 *          - angular speed at the starting point va
 *          - angular speed at the goal point vb
 *          - total time needed for the movemement tf
 * Output : duration of acceleration ti for a bang-bang configuration   */

float compute_intermediate_time(const float &vmax, const float &amax, const float &va, const float &vb, const float &tf)
{
    return ((2*vmax-va-vb)/amax - tf)/2;
}


                                                                                                                                    // A FAIRE : écrire la description de la fonction



void actu_joints(const float &tf1, const float &tf2, const float &config1, const float &config2, const float &vmax1, const float &vmax2,
                 const float &amax1, const float &amax2, const float &v1fin, const float &v2fin, float q1, float q2, float v1, float v2)
{
    if (tf1 <= tf2) // the second joint needs a longer time to reach its goal
    {
        const float tf = tf2;

        if (config1 == TRAPEZOIDAL && config2 == TRAPEZOIDAL)
        {
            const float t1 = compute_acceleration_duration(vmax2, amax2, v2);
            const float t2 = tf - compute_deceleration_duration(vmax2, amax2, v2fin);

            if (t1 > time_threshold) //the joints are still in their accelerating phase
            {
                float v1_sauv = v1;
                v1 = v1 + time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 + time_for_upload*amax2;
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            } else if(t2 > time_threshold) { //the joints are in the phase of constant angular speed
                v1 = vmax1;                                                                                                         // <-- ATTENTION : à modifier car pas vmax ici
                q1 = q1 + time_for_upload*v1;

                v2 = vmax2;
                q2 = q2 + time_for_upload*v2;
            } else { //the joints are in the decelarating phase
                float v1_sauv = v1;
                v1 = v1 - time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 - time_for_upload*amax2;
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            }

        } else if (config1 == BANGBANG && config2 == BANGBANG)
        {
            const float ti = compute_intermediate_time(vmax2, amax2, v2, v2fin, tf);

            if (ti > time_threshold) //the joints are still in their accelerating phase
            {
                float v1_sauv = v1;
                v1 = v1 + time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 + time_for_upload*amax2;
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            } else { //the joints are in the decelarating phase
                float v1_sauv = v1;
                v1 = v1 - time_for_upload*amax1;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 - time_for_upload*amax2;
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            }
        } else {
                                                                                                                                    //  <-- A FAIRE : pas d'idée si profils différents
        }


    } else { // the fisrt joint needs a longer time to reach its goal
        const float tf = tf1;

        if (config1 == TRAPEZOIDAL && config2 == TRAPEZOIDAL)
        {
            const float t1 = compute_acceleration_duration(vmax1, amax1, v1);
            const float t2 = tf - compute_deceleration_duration(vmax1, amax1, v1fin);

            if (t1 > time_threshold) //the joints are still in their accelerating phase
            {
                float v1_sauv = v1;
                v1 = v1 + time_for_upload*amax1;
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 + time_for_upload*amax2;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            } else if(t2 > time_threshold) { //the joints are in the phase of constant angular speed
                v1 = vmax1;
                q1 = q1 + time_for_upload*v1;

                v2 = vmax2;                                                                                                         // <-- ATTENTION : à modifier car pas vmax ici
                q2 = q2 + time_for_upload*v2;
            } else { //the joints are in the decelarating phase
                float v1_sauv = v1;
                v1 = v1 - time_for_upload*amax1;
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 - time_for_upload*amax2;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            }

        } else if (config1 == BANGBANG && config2 == BANGBANG)
        {
            const float ti = compute_intermediate_time(vmax2, amax2, v2, v2fin, tf);

            if (ti > time_threshold) //the joints are still in their accelerating phase
            {
                float v1_sauv = v1;
                v1 = v1 + time_for_upload*amax1;
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 + time_for_upload*amax2;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            } else { //the joints are in the decelarating phase
                float v1_sauv = v1;
                v1 = v1 - time_for_upload*amax1;
                q1 = q1 + time_for_upload*(v1+v1_sauv)/2;

                float v2_sauv = v2;
                v2 = v2 - time_for_upload*amax2;                                                                                    // <-- ATTENTION : à modifier car pas amax ici
                q2 = q2 + time_for_upload*(v2+v2_sauv)/2;
            }

        } else {
                                                                                                                                    //  <-- A FAIRE : pas d'idée si profils différents
        }

    }

}

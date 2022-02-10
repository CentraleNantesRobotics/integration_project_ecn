/* -------------------------------------------------------------------
 * This document is an utility document, which gathers several utility
 * functions needed for integ_trajectory_generation project.
 * -------------------------------------------------------------------*/

#include <cmath>

using namespace std;

/* This function calculates the time tf needed for the arm to reach the goal angle.
 * Inputs : - max angle speed of the motor vmax
 *          - max angle acceleration of the motor amax
 *          - angle speed at the starting point va
 *          - angle speed at the goal point vb
 *          - angle of the starting point qa
 *          - angle of the goal point qb
 * Output : time needed for the arm to reach the goal point tf   */

int compute_tf(const int &vmax, const int &amax, const int &va, const int &vb, const int &qa, const int &qb)
{
    const int deltaQ = qb - qa;
    const int deltaQLim = (2*va*(vmax-va) + 2*vb*(vmax-vb) + (vb-va)*(2*vmax-vb-va))/(2*amax);

    if (deltaQ < deltaQLim)
    {
        return (2*vmax-va-vb)/(5*amax) - (2*va)/(5*amax) + 2/(5*amax)*sqrt(pow((2*vmax-va-vb)/2,2) - pow(2*vmax-va-vb,2) - 5*amax*deltaQ);
    } else {
        return (deltaQ + (vb-va)*(2*vmax-vb-va)/(2*amax));
    }
}

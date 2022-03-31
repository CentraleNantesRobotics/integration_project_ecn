#include <calc.h>
#include <math.h>

/****************************************************************
 ** Calculating all the necessary for the Visual Servo Control **
 ****************************************************************/

vpVelocityTwistMatrix GetW(){ // Is the velocity twist matrix between the camera's frame and the link 2's frame
    vpRotationMatrix RotCam3x3;
    RotCam3x3[0][0] =  0; RotCam3x3[0][1] =  0; RotCam3x3[0][2] =  1;
    RotCam3x3[1][0] =  -1; RotCam3x3[1][1] =  0; RotCam3x3[1][2] =  0;
    RotCam3x3[2][0] =  0; RotCam3x3[2][1] =  -1; RotCam3x3[2][2] =  0;
    return vpVelocityTwistMatrix(RotCam3x3);}
// Get R is the velocity twist matrix between the link 2's frame and the world frame.
vpVelocityTwistMatrix GetR(double q1,double q2){return vpVelocityTwistMatrix(0,0,0,-(q1+q2),0,0);}

// GetJac is the 6 by 6 Jacobien Matrix of the 2R robot withtout the camera, characterizing the velocity
//of the end-effector.
vpMatrix GetJac(double q1, double q2, double l1, double l2)
{
	vpMatrix Jac(6, 2);

	double s1 = sin(q1);
	double c1 = cos(q1);
	double s12 = sin(q1 + q2);
	double c12 = cos(q1 + q2);

    Jac[1][0] = -1 * l1 * s1 - l2 * s12;
    Jac[1][1] = -1 * l2 * s12;
    Jac[2][0] = l1 * c1 + l2 * c12;
    Jac[2][1] = l2 * c12;
    Jac[3][0] = 1;
    Jac[3][1] = 1;

	return Jac;
}

using namespace std;

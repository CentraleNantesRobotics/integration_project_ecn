#include <calc.h>
#include <math.h>

/****************************************************************
 ** Calculating all the necessary for the Visual Servo Control **
 ****************************************************************/


vpVelocityTwistMatrix GetW(){
    vpRotationMatrix RotCam3x3;
    RotCam3x3[0][0] =  0; RotCam3x3[0][1] =  0; RotCam3x3[0][2] =  1;
    RotCam3x3[1][0] =  -1; RotCam3x3[1][1] =  0; RotCam3x3[1][2] =  0;
    RotCam3x3[2][0] =  0; RotCam3x3[2][1] =  -1; RotCam3x3[2][2] =  0;
    return vpVelocityTwistMatrix(RotCam3x3);}
vpVelocityTwistMatrix GetR(double q1,double q2){return vpVelocityTwistMatrix(0,0,0,-(q1+q2),0,0);}

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
std::pair<double, double> MGD(double l1, double l2, double q1, double q2){return  std::pair<double, double> {l1*cos(q1)+l2*cos(q1+q2), l1*sin(q1)+l2*sin(q1+q2)};}

std::pair<double, double> MGI(double Y, double Z, double l1, double l2)
{
    double q2d = acos((Y*Y+ Z*Z-l1*l1-l2*l2)/(2*l1*l2));
    double q1d = atan2(Z, Y)-(l1+l2*cos(q2d))/(pow(Y*Y+Z*Z,0.5));
    return  std::pair<double, double> {q1d, q2d};
}

std::pair<double, double> Deplacement(double l1, double l2, double q1, double q2,double f, const double x, const double y)
{
    std::pair<double, double> Pc = MGD(l1,l2,q1,q2);
    double Y = Pc.first - cos(q1+q2)*(x*(1+1/f))-sin(q1+q2)*(y*(1+1/f));
    double Z = Pc.second - cos(q1+q2)*(y*(1+1/f))-sin(q1+q2)*(x*(1+1/f));

    return std::pair<double, double> {Y, Z};
}

using namespace std;

#include <calc.h>
#include <math.h>

/****************************************************************
 ** Calculating all the necessary for the Visual Servo Control **
 ****************************************************************/

vpVelocityTwistMatrix GetRotCam(){return vpVelocityTwistMatrix(0,0,0,0,-M_PI/2,0);}  // Le repère cam dans gazebo est différent du repère caméra usuel.
vpVelocityTwistMatrix GetW(double offset){return vpVelocityTwistMatrix(-offset,0,0,M_PI,0,0);}
vpVelocityTwistMatrix GetR(double q1,double q2){return vpVelocityTwistMatrix(0,0,0,0,0,-(q1+q2));}
vpMatrix GetJac(double q1, double q2, double l1, double l2)
{
	vpMatrix Jac(6, 2);

	double s1 = sin(q1);
	double c1 = cos(q1);
	double s12 = sin(q1 + q2);
	double c12 = cos(q1 + q2);

	Jac[0][0] = -1 * l1 * s1 - l2 * s12;
	Jac[0][1] = -1 * l2 * s12;
    Jac[1][0] = l1 * c1 + l2 * c12;
	Jac[1][1] = l2 * c12;
	Jac[5][0] = 1;
	Jac[5][1] = 1;

	return Jac;
}
std::pair<double, double> MGD(double l1, double l2, double q1, double q2){return {l1*cos(q1)+l2*cos(q1+q2), l1*sin(q1)+l2*sin(q1+q2)};}

std::pair<double, double> MGI(double Y, double Z, double l1, double l2)
{
    double q2d = acos((Y*Y+ Z*Z-l1*l1-l2*l2)/(2*l1*l2));
    double q1d = atan2(Z, Y)-(l1+l2*cos(q2d))/(pow(Y*Y+Z*Z,0.5));
    return {q1d, q2d};
}

std::pair<double, double> Deplacement(double l1, double l2, double q1, double q2,double f)//,double x, double y)
{
    //std::pair<double, double> Pc = MGD(l1,l2,q1,q2);
    //double Y = Pc.first - cos(q1+q2)*(x*(1+1/f))-sin(q1+q2)*(y*(1+1/f));
    //double Z = Pc.second - cos(q1+q2)*(y*(1+1/f))-sin(q1+q2)*(x*(1+1/f));

    //return {Y, Z};
    return {0,0};
}


//vpMatrix GetRbc(tf::TransformListener &listener)
//{
//	vpMatrix Rbc(6, 6);
//	tf::StampedTransform transform;
//	listener.waitForTransform("/camera_link", "/world", ros::Time(0), ros::Duration(4.0));
//	listener.lookupTransform("/camera_link", "/world", ros::Time(0), transform);
//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//		{
//			Rbc[i][j] = transform.getBasis()[i][j];
//			Rbc[i + 3][j + 3] = Rbc[i][j];
//		}
//	return Rbc;
//}

using namespace std;

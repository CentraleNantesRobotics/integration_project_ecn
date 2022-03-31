#ifndef CALC_H
#define CALC_H

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <functional>
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visp/vpCameraParameters.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <ecn_common/color_detector.h>

using namespace std;

//vpMatrix GetRbc(tf::TransformListener &listener);
vpRotationMatrix GetRotLink2ToCam();
vpVelocityTwistMatrix GetRotCamToGazebo(vpRotationMatrix &R);
vpVelocityTwistMatrix GetW();
vpVelocityTwistMatrix GetR(double q1, double q2);

vpMatrix GetJac(double q1, double q2, double l1, double l2);

//std::pair<double, double> MGD(double l1, double l2, double q1, double q2);
//std::pair<double, double> MGI(double Y, double Z, double l1, double l2);
//std::pair<double, double> Deplacement(double l1, double l2, double q1, double q2,double f, const double x, const double y);//,double x, double y);

#endif

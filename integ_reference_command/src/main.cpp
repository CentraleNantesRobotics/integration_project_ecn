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
#include <calc.h>

using namespace std;
// Initializing the data we need.
vpColVector q(2);
vpFeaturePoint s;

void TargetPointMessageCallback(const geometry_msgs::Pose2D message_s)
{
    //this function only sets the received message message_s to the vpFeaturePoint format
    // The frame of the camera on Gazebo is different from the frame we used, so we used a rotation along y of pi/2.
    // Don't forget to send the correct velocities value to the publisher
    s.set_x(message_s.x);
    s.set_y(message_s.y);
    s.set_Z(1);             // is set to 1 for the current problem only.
}

void JointStatesMessageCallback(const sensor_msgs::JointState message_s)
{
    //this function only sets the received message message_s to the vpFeaturePoint format
    q[0] = message_s.position[0];
    q[1] = message_s.position[1];
}

int main(int argc, char **argv)
{
    // Parameters (modify here if needed) NEED TO CHANGE THEM SO WE GET THEM THROUGH ROS
    cout << "TEST LIGNE DE CODE" << endl;
    double lambda = 50;
    double l1 = 0.8;
    double l2 = 0.6;
    double focal = 0.01;

    // ROS part    featurePointPositionSub.
    ros::init(argc, argv, "integ_reference_command");
    ros::NodeHandle node;
    ros::Publisher velocityPub = node.advertise<sensor_msgs::JointState>("camera_trajectory", 1000);



    ros::Subscriber featurePointPositionSub = node.subscribe("masscenter", 1000, TargetPointMessageCallback);
    ros::Subscriber jointStatesSub = node.subscribe("state", 1000, JointStatesMessageCallback);

	tf::TransformListener listener;
	ros::Rate loop_rate(10);


	while (ros::ok())
    {
        vpMatrix L = s.interaction(); //L is the interaction matrix
		sensor_msgs::JointState targetJointState; //this is what we will publish

        vpVelocityTwistMatrix W = GetW();
        vpVelocityTwistMatrix R = GetR(q[0],q[1]);

        // Computing what we need to get the joints velocity.

        auto sPoint = - lambda * s.get_s();
        vpMatrix J = GetJac(q[0], q[1], l1, l2);
        vpMatrix Jc = W * R * J;
        vpMatrix Js = L * Jc;
        auto qPoint = Js.pseudoInverse() * sPoint;

       //Displacement to get to the center of the target.
        std::pair<double, double> deplacement = Deplacement(l1,l2,q[0],q[1],focal, s.get_x(), s.get_y());
        // MGI to get the value of the angles.
        std::pair<double, double> angles = MGI(deplacement.first, deplacement.second, l1, l2);

       // Valeurs à renvoyer
        targetJointState.name.push_back("joint 1");
        targetJointState.name.push_back("joint 2");
        targetJointState.velocity.push_back(qPoint[0]);
        targetJointState.velocity.push_back(qPoint[1]);
        targetJointState.position.push_back(angles.first);
        targetJointState.position.push_back(angles.second);
        velocityPub.publish(targetJointState);

        cout << "target speed: " << endl;
        cout << qPoint << endl;
        cout << "target position: " << endl;
        cout << angles.first << "  "<< angles.second << endl;


        ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;
}

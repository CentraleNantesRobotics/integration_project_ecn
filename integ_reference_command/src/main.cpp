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

/* This file is the main code and the main node for the reference command.
 * We apply the Visual Servo Control methode to determine the velocity and the position of the joints needed for the
 * camera to shoot the target.
 * To achieve that, the calc.cpp and calc.h files are used to compute the needed matrices for the algorithm.
 * A node/topic could be created to get the parameter of the robot but we choose to add them manually.
 * You can find the course about Visual Servo Control on the webiste of Oliver Kermorgant.
 */

using namespace std;
// Initializing the data we need.
vpColVector q(2);
vpFeaturePoint s;

void TargetPointMessageCallback(const geometry_msgs::Pose2D message_s)
{
    //this function only sets the received message message_s to the vpFeaturePoint format
    // Don't forget to send the correct velocities value to the publisher
    s.set_x(message_s.x);
    s.set_y(message_s.y);
    s.set_Z(1);             // is set to 1 for the current problem.
}

void JointStatesMessageCallback(const sensor_msgs::JointState message_s)
{
    //this function only sets the received message message_s to the vpFeaturePoint format
    q[0] = message_s.position[0];
    q[1] = message_s.position[1];
}

int main(int argc, char **argv)
{
    // Parameters (modify here if needed)

    double lambda = 50;
    double l1 = 0.8;
    double l2 = 0.6;

    double deltaT = 0.1; // sampling period
    // ROS part    featurePointPositionSub.
    ros::init(argc, argv, "integ_reference_command");
    ros::NodeHandle node;
    ros::Publisher velocityPub = node.advertise<sensor_msgs::JointState>("/camera_trajectory", 10);

    //This part is only to test with the simulation_trajectory and should be deleted when the code works.
    ros::Subscriber featurePointPositionSub = node.subscribe("/trajSim", 10, TargetPointMessageCallback);
    ros::Subscriber jointStatesSub = node.subscribe("/stateSim", 10, JointStatesMessageCallback);

    //This part should be kept and should be uncommented when the code works.
    //ros::Subscriber featurePointPositionSub = node.subscribe("masscenter", 1000, TargetPointMessageCallback);
    //ros::Subscriber jointStatesSub = node.subscribe("state", 1000, JointStatesMessageCallback);

	tf::TransformListener listener;
    ros::Rate loop_rate(1/deltaT);

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

        // desired position
        vpColVector angles(2);
        angles[0]= q[0] + 0.5*deltaT*qPoint[0];
        angles[1]= q[1] + 0.5*deltaT*qPoint[1];

        // Publishing the values.
        targetJointState.name.push_back("joint 1");
        targetJointState.name.push_back("joint 2");
        targetJointState.velocity.push_back(qPoint[0]);
        targetJointState.velocity.push_back(qPoint[1]);
        targetJointState.position.push_back(angles[0]);
        targetJointState.position.push_back(angles[1]);
        velocityPub.publish(targetJointState);

        cout << "target speed: " << endl;
        cout << qPoint << endl;
        cout << "target position: " << endl;
        cout << angles << endl;

        ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;
}

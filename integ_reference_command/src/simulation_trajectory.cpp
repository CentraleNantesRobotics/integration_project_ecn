#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <ctime>
#include <math.h>
#include <iostream>

std::pair<double, double> pointOnCircle(int &t)
{
    return {0.5*cos(2*M_PI*t/48), 0.5*sin(2*M_PI*t/48)};
}

std::pair<double, double> MGI(double Y, double Z, double l1, double l2)
{
    double q2d = acos((Y*Y+ Z*Z-l1*l1-l2*l2)/(2*l1*l2));
    double q1d = atan2(Z, Y)-(l1+l2*cos(q2d))/(pow(Y*Y+Z*Z,0.5));
    return  std::pair<double, double> {q1d, q2d};
}

int main(int argc, char **argv)
{
    static int timer = 0;

    ros::init(argc, argv, "ref_command_simulation_trajectory");
    ros::NodeHandle node;

    ros::Publisher circleTrajectory = node.advertise<geometry_msgs::Pose2D>("/trajSim", 10);
    ros::Publisher statePublisher = node.advertise<sensor_msgs::JointState>("/stateSim", 10);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Pose2D toPublish;
        sensor_msgs::JointState jointToPublish;

        timer += 1;
        std::pair<double, double> weAreRollingDown = pointOnCircle(timer);
        toPublish.x = weAreRollingDown.first;
        toPublish.y = weAreRollingDown.second;
        auto jointState = MGI(toPublish.x, toPublish.y, 0.6, 0.8);
        jointToPublish.position.push_back(jointState.first);
        jointToPublish.position.push_back(jointState.second);

        circleTrajectory.publish(toPublish);
        statePublisher.publish(jointToPublish);
        std::cout << toPublish.x << ", " << toPublish.y << std::endl;

        //ros::spin();
        loop_rate.sleep();
    }



}

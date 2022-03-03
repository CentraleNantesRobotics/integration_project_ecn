#include <ros/ros.h>
#include <integ_trajectory_generation/generation_node.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "integ_trajectory_generation");

    ros::NodeHandle nh;

    GenerationNode node;

    const double frequency = 10;
    ros::Rate rate(frequency);

    while (ros::ok())
    {
        node.publishingCallback();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

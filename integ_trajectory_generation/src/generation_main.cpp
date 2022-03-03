/* -------------------------------------------------------------------
 * This file serves as the main file for the generation of
 * trajectories. It holds only the main function, which is pretty
 * classic for a ros project.
 * -------------------------------------------------------------------*/

#include <ros/ros.h>
#include <integ_trajectory_generation/generation_node.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generation");

    GenerationNode node;

    ros::Rate rate(node.publishing_freq_);

    while (ros::ok())
    {
        node.publishingCallback();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

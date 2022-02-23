#include <ros/ros.h>
#include <integ_trajectory_generation/generation_node.h>

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "integ_trajectory_generation");

  GenerationNode node;

  ros::spin();

  return 0;

}

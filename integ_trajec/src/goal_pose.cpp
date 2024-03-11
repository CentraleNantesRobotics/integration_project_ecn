#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class GoalPosePublisher : public rclcpp::Node
{
public :
    GoalPosePublisher() : Node("goal_pose_publisher")
    {
        publisher = this->create_publisher<sensor_msgs::msg::JointState>("desired_pose", 10);

        while (q1 != 0)
        {
            std::cout << "Entre la position desiree de Q1" << std::endl << ": ";
            std::cin >> q1;

            std::cout << "Entre la position desiree de Q2" << std::endl << ": ";
            std::cin >> q2;

            std::cout << "Fin du node" << std::endl;

            auto goal_pose = sensor_msgs::msg::JointState();

            goal_pose.position.push_back(q1);
            goal_pose.position.push_back(q2);

            publisher->publish(goal_pose);

        }
    }

    double q1 = 1, q2 = 1;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPosePublisher>());
  rclcpp::shutdown();
  return 0;
}


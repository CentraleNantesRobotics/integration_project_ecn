#include <functional>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "integ_msgs/srv/srv_trajec.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class TrajectoryClient : public rclcpp::Node
{
  public:
    TrajectoryClient()
    : Node("generation_trajectory_client")
    {
        client = this->create_client<integ_msgs::srv::SrvTrajec>("srvtrajec");

        publisher_waypoint = this->create_publisher<sensor_msgs::msg::JointState>("desired_joint_state",10);

        subscriber_initial_pose = this->create_subscription<sensor_msgs::msg::JointState>("joint_state", 10, std::bind(&TrajectoryClient::callback_initial_pose, this, _1));

        subscriber_goal_pose = this->create_subscription<sensor_msgs::msg::JointState>("desired_pose", 10, std::bind(&TrajectoryClient::callback_goal_pose, this, _1));
    }

  private:

    void callback_initial_pose(const sensor_msgs::msg::JointState::SharedPtr initial_pose)
    {
        request->initial_pose = *initial_pose;
    }

    void callback_goal_pose(const sensor_msgs::msg::JointState::SharedPtr goal_pose)
    {
        request->goal_pose = *goal_pose;

        auto result = client->async_send_request(request);
        response->trajectory = result.get()->trajectory;

        auto send_waypoints = sensor_msgs::msg::JointState();

        send_waypoints.header = response->trajectory.header;
        send_waypoints.name = response->trajectory.joint_names;

        for (auto &way_point : response->trajectory.points)
        {
            send_waypoints.position.push_back(way_point.positions[0]);
            send_waypoints.position.push_back(way_point.positions[1]);

            send_waypoints.velocity.push_back(way_point.velocities[0]);
            send_waypoints.velocity.push_back(way_point.velocities[1]);

            send_waypoints.effort.push_back(way_point.accelerations[0]);
            send_waypoints.effort.push_back(way_point.accelerations[1]);

            publisher_waypoint->publish(send_waypoints);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

        }


    }

    rclcpp::Client<integ_msgs::srv::SrvTrajec>::SharedPtr client;
    integ_msgs::srv::SrvTrajec::Request::SharedPtr request;
    integ_msgs::srv::SrvTrajec::Response::SharedPtr response;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_waypoint;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_initial_pose;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_goal_pose;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryClient>());
  rclcpp::shutdown();
  return 0;
}

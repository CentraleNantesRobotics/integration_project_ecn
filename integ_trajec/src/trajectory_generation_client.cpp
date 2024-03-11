#include <functional>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "integ_msgs/srv/srv_trajec.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class TrajectoryClient : public rclcpp::Node
{
  public:
    TrajectoryClient()
    : Node("generation_trajectory_client")
    {
        client = this->create_client<integ_msgs::srv::SrvTrajec>("srvtrajec");

        publisher_waypoint = this->create_publisher<sensor_msgs::msg::JointState>("/scara/desired_joint_states",10);

        subscriber_initial_pose = this->create_subscription<sensor_msgs::msg::JointState>("/scara/joint_states", 10, std::bind(&TrajectoryClient::callback_initial_pose, this, _1));

        q1_ = create_subscription<std_msgs::msg::Float64>(
                    "/q1_chosen",
                    10,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    q1Callback(msg);
        });

        q2_ = create_subscription<std_msgs::msg::Float64>(
                    "/q2_chosen",
                    10,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    q2Callback(msg);
                    });
    }

  private:

    double q1;
    double q2;

    //Callback pour le Kp, utile dans le cas d'une optimisation par sliderpublisher par exemple ou pour un calcul externe du Kp
    void q1Callback(std_msgs::msg::Float64::SharedPtr k){
        q1=k->data;
        callback_goal_pose();

    }

    //Callback pour le Kd, utile dans le cas d'une optimisation par sliderpublisher par exemple ou pour un calcul externe du Kd
    void q2Callback(std_msgs::msg::Float64::SharedPtr k){
        q2=k->data;
        callback_goal_pose();

    }

    void callback_initial_pose(const sensor_msgs::msg::JointState::SharedPtr initial_pose)
    {
        request->initial_pose = *initial_pose;
    }

    void callback_goal_pose()
    {

        request->goal_pose.position.push_back(q1);
        request->goal_pose.position.push_back(q2);


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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr q1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr q2_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryClient>());
  rclcpp::shutdown();
  return 0;
}

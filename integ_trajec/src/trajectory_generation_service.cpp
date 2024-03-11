#include "rclcpp/rclcpp.hpp"
#include "integ_msgs/srv/srv_trajec.hpp"

#include "polynomial_interpolator.h"


using namespace std::chrono_literals;

class GenerationTrajectory : public rclcpp::Node
{
  public:
    GenerationTrajectory()
    : Node("generation_trajectory_service")
    {

        this->declare_parameter("polynome_order", 1);
        this->declare_parameter("type_interpolator", "polynomial");
        this->declare_parameter("work_time", 1.);

        tf = get_parameter("work_time").as_double();

        service = this->create_service<integ_msgs::srv::SrvTrajec>("srvtrajec", [this](const std::shared_ptr<integ_msgs::srv::SrvTrajec::Request> request,
                                                                   std::shared_ptr<integ_msgs::srv::SrvTrajec::Response> response){this->generation_trajectory(request,response);});

    }

  private:

    void generation_trajectory(const std::shared_ptr<integ_msgs::srv::SrvTrajec::Request> request,
                               std::shared_ptr<integ_msgs::srv::SrvTrajec::Response> response)
    {
        int order = get_parameter("polynome_order").as_int();
        if (get_parameter("type_interpolator").as_string() == "polynomial")
        {
            response->trajectory.points.resize(interpolation_values(order, 0).size());

            int t=0;
            for (auto &waypoint : response->trajectory.points)
            {
                response->trajectory.header = request->initial_pose.header;

                response->trajectory.joint_names = request->initial_pose.name;

                waypoint.positions.push_back(request->initial_pose.position[0] + interpolation_values(order, 0)[t]*(request->goal_pose.position[0] - request->initial_pose.position[0]));
                waypoint.positions.push_back(request->initial_pose.position[1] + interpolation_values(order, 0)[t]*(request->goal_pose.position[1] - request->initial_pose.position[1]));

                waypoint.velocities.push_back(interpolation_values(order, 1)[t]*(request->goal_pose.position[0] - request->initial_pose.position[0]));
                waypoint.velocities.push_back(interpolation_values(order, 1)[t]*(request->goal_pose.position[1] - request->initial_pose.position[1]));

                waypoint.accelerations.push_back(interpolation_values(order, 2)[t]*(request->goal_pose.position[0] - request->initial_pose.position[0]));
                waypoint.accelerations.push_back(interpolation_values(order, 2)[t]*(request->goal_pose.position[1] - request->initial_pose.position[1]));

                double reel_time = t*tf/interpolation_values(order, 0).size();

                int sec = reel_time;
                int nanosec = (reel_time - sec)*std::pow(10,9);

                waypoint.time_from_start.sec = sec;
                waypoint.time_from_start.nanosec =nanosec;
            }
        }
    }

    rclcpp::Service<integ_msgs::srv::SrvTrajec>::SharedPtr service;
    double tf;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenerationTrajectory>());
  rclcpp::shutdown();
  return 0;
}

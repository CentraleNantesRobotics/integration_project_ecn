#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std::chrono_literals;

class GenerationTrajectory : public rclcpp::Node
{
  public:
    GnerationTrajectory()
    : Node("gneration_trajectory")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("desired_joint_states", 10);

      initial_position = this->create_subscriber<sensor_msgs::msg::JointState>("objectives_points",10);

      final_position = this->create_subscriber<sensor_msgs::msg::JointState>("objectives_points",10);

      timer_ = this->create_wall_timer(500ms, std::bind(&GenerationTrajectory::generation_points, this));

      this->declare_parameter("polynome_order", 1);
    }

  private:
    void generation_points()
    {

        std::vector<double> coefficients = define_coefficients(get_parameter("polynome_order").as_int());

        auto publish_point = sensor_msgs::msg::JointState();

        publish_point.header = initial_position.header;

        for (int i = 0; i<2; i++)
        {
            publish_point.name[i] = initial_position.name[i];

            r = interpolator(coefficients,t,tf,0);
            r_dot = interpolator(coefficients,t,tf,1);
            r_ddot = interpolator(coefficients,t,tf,2);

            publish_point.position[i] = initial_position.position[i] + r(final_position[i] - initial_position[i]);
            publish_point.velocity[i] = r_dot(final_position[i] - initial_position[i]);
            publish_point.effort[i] = r_ddot(final_position[i] - initial_position[i]);
        }

        publisher_->publish(publish_point);


    }

    std::vector<double> define_coefficients(int order)
    {
        std::vector<double> coefficients(6,0);
        switch (order)
        {
            case 1:
                coefficients[1] = 1;

            case 3:
                coefficients[2] = 3;
                coefficients[3] = -2;

            case 5:
                coefficients[3] = 10;
                coefficients[4] = -15;
                coefficients[5] = 6;
        }
        return (coefficients);
    }

    double interpolator (std::vector<double> coefficients, double temps, double transfer_time, int derive)
    {
        double r = 0;

        switch (derive)
        {
        case 0:
            for (int i = 0; i < coefficients.size(); i++)
            {
                r += coefficients[i]*std::pow(stdtemps/transfer_time,i);
            }

        case 1:
            for (int i = 1; i < coefficients.size(); i++)
            {
                r += i*coefficients[i]*std::pow(stdtemps/transfer_time,i-1);
            }

        case 2:
            for (int i = 2; i < coefficients.size(); i++)
            {
                r += i*(i-1)*coefficients[i]*std::pow(stdtemps/transfer_time,i-2);
            }

        }
        return(r);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscriber<sensor_msgs::msg::JointState>::SharedPtr initial_position;
    rclpcpp::Subscriber<sensor_msgs::msg::JointState>::SharedPtr final_position;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenerationTrajectory>());
  rclcpp::shutdown();
  return 0;
}

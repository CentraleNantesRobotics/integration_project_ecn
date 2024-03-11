//integ_CTC_test is a trajectory test for Computed Torque Control node designed for INTEG project for Centrale Nantes Robotics
//Thibault LEBLANC & Julien COUPEAUX & Luca MIMOUNI & Baptiste LARDINOIT, Version 1.0.2, March 2024


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Core>
using namespace std::placeholders;



class crash_test : public rclcpp::Node {

public :

    crash_test()
        : Node("crash_test"){

    //Initialisation du publisher
    traj_test_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/scara/desired_joint_states", 10);

    // Start a timer to publish the sine wave
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Adjust the period to your needs
        std::bind(&crash_test::publishSineWave, this));

    }


private :

    void publishSineWave() {
        auto message = sensor_msgs::msg::JointState();

        message.header.stamp = this->get_clock()->now();

        // Trajectoir d'une sinusoide de frequence 'freq' et d'amplitude 'amplitude'
        message.name.push_back("trajectoire");
        message.position.push_back(amplitude * std::sin(2.0 * PI * freq * this->get_clock()->now().seconds()));
        message.velocity.push_back(amplitude * 2.0 * PI * freq * std::cos(2.0 * PI * freq * this->get_clock()->now().seconds()));
        message.effort.push_back(- amplitude * 4.0 * pow(PI,2) * pow(freq,2) * std::sin(2.0 * PI * freq * this->get_clock()->now().seconds()));


        //Publish the message
        traj_test_publisher_->publish(message);
    }


    const double amplitude = 1.0;
    const double PI = 3.14;
    const double freq = 1.0;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr traj_test_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


};




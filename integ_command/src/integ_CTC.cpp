//integ_CTC is a Computed Torque Control node designed for INTEG project for Centrale Nantes Robotics
//Thibault LEBLANC & Julien COUPEAUX, Version 1.0.2, February 2024

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "yaml-cpp/yaml.h"

class ComputedTorqueControl : public rclcpp::Node {
public:
    ComputedTorqueControl()
        : Node("computed_torque_control") {

        // Chemin vers le fichier YAML
                std::string yaml_file_path = "/params/dmi.yml";
        // Modèle dynamique inverse à partir du fichier YAML
        inverse_dynamics_model_ = loadInverseDynamicsModel(yaml_file_path);


        desired_jointstate_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/scara/desired_joint_states", 10, std::bind(&ComputedTorqueControl::jointStateCallback, this, std::placeholders::_1)); // A revoir en fonction du nom des topics des gens qui font la trajectoire

        // Initialiser les subscription, les publisher, le contrôleur, etc.
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/scara/joint_states", 10, std::bind(&ComputedTorqueControl::jointStateCallback, this, std::placeholders::_1));

        computed_torque_publisher_joint1_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/scara/computed_torque_joint1", 10);

        computed_torque_publisher_joint2_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/scara/computed_torque_joint2", 10);

        controlTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ComputedTorqueControl::controlCallback, this));
    }

private:
    std::vector<std::vector<double>> loadInverseDynamicsModel(const std::string& yaml_file_path) {
        try {
            YAML::Node config = YAML::LoadFile(yaml_file_path);
            return config["inverse_dynamics_model"].as<std::vector<std::vector<double>>>();
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erreur lors de la lecture du fichier YAML : %s", e.what());
            return {};
        }
    }


     void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state, const sensor_msgs::msg::JointState::SharedPtr desired_joint_state) {
        double kp;
        double kd;
        double pd1 = desired_joint_state->position[1];
        double pd2 = desired_joint_state->position[2];
        double vd1 = desired_joint_state->velocity[1];
        double vd2 = desired_joint_state->velocity[2];

        double real_pos1 = joint_state->position[1];
        double real_pos2 = joint_state->position[2];
        double real_vel1 = joint_state->velocity[1];
        double real_vel2 = joint_state->velocity[2];
    }

    void controlCallback() {
        // Calcul du couple en utilisant le contrôle par couple calculé
        // inverse_dynamics_model_ est à utiliser
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint1;
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint2;

        // computed_torque_msg_joint1.data = computed_torque_calculation_function_joint1(inverse_dynamics_model_);
        // computed_torque_msg_joint2.data = computed_torque_calculation_function_joint2(inverse_dynamics_model_);

        computed_torque_publisher_joint1_->publish(computed_torque_msg_joint1);
        computed_torque_publisher_joint2_->publish(computed_torque_msg_joint2);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_jointstate_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr computed_torque_publisher_joint1_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr computed_torque_publisher_joint2_;
    rclcpp::TimerBase::SharedPtr controlTimer_;

    std::vector<std::vector<double>> inverse_dynamics_model_;
};

int main(int argc, char *argv[]) {
        rclcpp::init(0, nullptr);
        rclcpp::spin(std::make_shared<ComputedTorqueControl>());
        rclcpp::shutdown();
        return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "yaml-cpp/yaml.h"

class ComputedTorqueControl : public rclcpp::Node {
public:
    ComputedTorqueControl(const std::string& yaml_file_path)
        : Node("computed_torque_control") {

        // Charger le modèle dynamique inverse à partir du fichier YAML
        inverse_dynamics_model_ = loadInverseDynamicsModel(yaml_file_path);

        // Initialiser les abonnements, les publications, le contrôleur, etc.
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

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state) {
        // Mettez à jour les états des joints en fonction de votre robot
        // ...
    }

    void controlCallback() {
        // Calculer le couple en utilisant le contrôle de couple calculé
        // Utilisez inverse_dynamics_model_ dans votre calcul de contrôle
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint1;
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint2;

        // computed_torque_msg_joint1.data = computed_torque_calculation_function_joint1(inverse_dynamics_model_);
        // computed_torque_msg_joint2.data = computed_torque_calculation_function_joint2(inverse_dynamics_model_);

        computed_torque_publisher_joint1_->publish(computed_torque_msg_joint1);
        computed_torque_publisher_joint2_->publish(computed_torque_msg_joint2);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr computed_torque_publisher_joint1_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr computed_torque_publisher_joint2_;
    rclcpp::TimerBase::SharedPtr controlTimer_;

    std::vector<std::vector<double>> inverse_dynamics_model_;
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("computed_torque_control_main"), "Veuillez fournir le chemin vers le fichier YAML.");
        return 1;
    }

    rclcpp::init(argc, argv);

    // Passez le chemin vers le fichier YAML lors de la création du nœud
    rclcpp::spin(std::make_shared<ComputedTorqueControl>(argv[1]));

    rclcpp::shutdown();
    return 0;
}

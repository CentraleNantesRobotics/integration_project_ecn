#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ComputedTorqueControl : public rclcpp::Node {
public:
    // Ajoutez une nouvelle variable pour le modèle dynamique inverse
    ComputedTorqueControl(const std::vector<std::vector<double>>& inverse_dynamics_model)
        : Node("computed_torque_control"), inverse_dynamics_model_(inverse_dynamics_model) {

        // ... (le reste du code reste inchangé)

    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state) {
        // ... (le reste du code reste inchangé)
    }

    void controlCallback() {
        // Utilisez inverse_dynamics_model_ dans votre calcul de contrôle
        // (Adapter cette fonction en fonction de votre contrôleur)
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint1;
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint2;

        // Utilisez inverse_dynamics_model_ dans votre calcul de contrôle
        // computed_torque_msg_joint1.data = computed_torque_calculation_function_joint1(inverse_dynamics_model_);
        // computed_torque_msg_joint2.data = computed_torque_calculation_function_joint2(inverse_dynamics_model_);

        computed_torque_publisher_joint1_->publish(computed_torque_msg_joint1);
        computed_torque_publisher_joint2_->publish(computed_torque_msg_joint2);
    }

    void desiredTorquesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr desired_torques) {
        // ... (le reste du code reste inchangé)
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr computed_torque_publisher_joint1_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr computed_torque_publisher_joint2_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_torques_subscriber_;
    rclcpp::TimerBase::SharedPtr controlTimer_;

    // Nouvelle variable pour stocker le modèle dynamique inverse
    std::vector<std::vector<double>> inverse_dynamics_model_;
};

int main(int argc, char *argv[]) {
    // Exemple de modèle dynamique inverse (à remplacer par votre propre modèle)
    std::vector<std::vector<double>> my_inverse_dynamics_model = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    rclcpp::init(argc, argv);

    // Passez le modèle dynamique inverse lors de la création du nœud
    rclcpp::spin(std::make_shared<ComputedTorqueControl>(my_inverse_dynamics_model));

    rclcpp::shutdown();
    return 0;
}

//integ_CTC is a Computed Torque Control node designed for INTEG project for Centrale Nantes Robotics
//Thibault LEBLANC & Julien COUPEAUX, Version 1.0.2, February 2024

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Core>

class ComputedTorqueControl : public rclcpp::Node {
public:
    ComputedTorqueControl()
        : Node("computed_torque_control") {


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
            std::chrono::milliseconds(10), std::bind(&ComputedTorqueControl::jointStateCallback, this));
    }

private:

     void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state, const sensor_msgs::msg::JointState::SharedPtr desired_joint_state) {
        //définition des gains
        double kp = 1.;
        double kd = 1.;
        //définition des angles et vitesses désirées
        double pd1 = desired_joint_state->position[1];
        double pd2 = desired_joint_state->position[2];
        double vd1 = desired_joint_state->velocity[1];
        double vd2 = desired_joint_state->velocity[2];
        //définition de l'état
        double real_pos1 = joint_state->position[1];
        double real_pos2 = joint_state->position[2];
        double real_vel1 = joint_state->velocity[1];
        double real_vel2 = joint_state->velocity[2];
        //définition des variables
        double g = 0;
        double m1=1.;
        double m2=1.;
        double l1=1.;
        double l2=1.;

        //définition de la matrice de gravité
        Eigen::MatrixXd gravity;
        double g11 =-1*(m1+m2)*g*l1*sin(real_pos1)-m2*g*l2*sin(real_pos1+real_pos2);
        double g22 =-1*m2*g*l2*sin(real_pos1+real_pos2);
        gravity(0,0)=g11;
        gravity(1,0)=g22;

        //définition de la matrice d'intertie
        Eigen::MatrixXd inertia;
        double i11 = (m1+m2)*l1*l1+m2*l2*l2+2*m2*l1*l2*cos(real_pos2);
        double i12 = m2*l2*l2+m2*l1*l2*cos(real_pos2);
        double i21 = i12;
        double i22 = m2*l2*l2;
        inertia(0,0)=i11;
        inertia(0,1)=i12;
        inertia(1,0)=i21;
        inertia(1,1)=i22;

        //définition de la matrice de coriolis
        Eigen::MatrixXd coriolis;
        double c11 =-1*m2*l1*l2*(2*real_vel1*real_vel2+real_vel1*real_vel1)*sin(real_pos2);
        double c22 =-1*m2*l1*l2*real_vel1*real_vel2*sin(real_pos2);
        coriolis(0,0)=c11;
        coriolis(1,0)=c22;


        // Calcul du couple en utilisant le contrôle par couple calculé
        // inverse_dynamics_model_ est à utiliser
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint1;
        std_msgs::msg::Float64MultiArray computed_torque_msg_joint2;

        Eigen::MatrixXd  estimated_vel;
        estimated_vel(0)=kp*(pd1-real_pos1)-kd*real_vel1;
        estimated_vel(1)=kp*(pd2-real_pos2)-kd*real_vel2;

        Eigen::Matrix<double,2,1> computed_troque;
        computed_troque=inertia*estimated_vel+coriolis+gravity;



        computed_torque_msg_joint1.data = computed_troque(0,0);
        computed_torque_msg_joint2.data = computed_troque(1,0);

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
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ComputedTorqueControl>());
        rclcpp::shutdown();
        return 0;
}

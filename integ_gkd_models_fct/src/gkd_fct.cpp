#include <integ_gkd_models_fct/gkd_fct.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <integ_gkd_models_fct/dyn_mats.h>
#include <visp/vpMatrix.h>

integ_gkd_models_fct::dyn_mats Dynamic(sensor_msgs::JointState req){

    YAML::Node lconf = YAML::LoadFile("RobotParam.yml");
    auto l1 = lconf["l1"].as<double>();
    auto l2 = lconf["l2"].as<double>();
    auto m1 = lconf["m1"].as<double>();
    auto m2 = lconf["m2"].as<double>();
    auto Iz1 = lconf["Iz1"].as<double>();
    auto Iz2 = lconf["Iz2"].as<double>();
    auto g = lconf["g"].as<double>();
    auto c1 = lconf["c1"].as<double>();
    auto c2 = lconf["c2"].as<double>();

    auto theta = req.position;
    auto theta_d = req.velocity;
    auto theta_d_d = req.effort;


    auto Z1 = m1*pow(c1,2) + m2*(pow(l1,2)+pow(c2,2)+2*l1*c2*cos(theta[1])) + Iz1 + Iz2;
    auto Z2 = m2*(pow(c2,2)+l1*c2*cos(theta[1])) + Iz2;
    auto Z3 = m2*pow(c2,2) + Iz2;
    auto Z4 = m2*c2*g*cos(theta[0]+theta[1])+(m1*c1+m2*l1)*g*cos(theta[0]);
    auto Z5 = m2*c2*g*cos(theta[0]+theta[1]);
    auto h = -0.5*m2*l1*l2*sin(theta[1]);

 //   double C[2][2] = {{h * theta_d[1], h * (theta_d[0]+theta_d[1]) },{ -h * theta_d[0], 0}};
 //   double G[2][1] = {{Z4}, {Z5}};

    vpMatrix C(2,2);
    C[1][1] = h * theta_d[1]; C[1][2] = h * (theta_d[0]+theta_d[1]);
    C[2][1] = -h * theta_d[1]; C[2][2] =0;
    vpMatrix G(2,1);
    G[1][1] = Z4; G[2][1] = Z5;
    vpMatrix theta_p(2,1);
    theta_p[1][1] = theta_d[0];
    theta_p[2][1] = theta_d[1];

    auto N = C*theta_p+G;

    integ_gkd_models_fct::dyn_mats output;
    output.m_coeffs = { Z1 , Z2 , Z2 , Z3 };
    output.n_coeffs = {N[1][1], N[2][1]};

    return(output);
};

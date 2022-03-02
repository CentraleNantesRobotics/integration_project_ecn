/*
 * ROS pkg to control 2R robot joints using a Computed Torque controller for trajectory tracking
 * Authors: Joe Ferreira Scholtz and Julio Chi
 */

//includes
#include <cmath>
#include <std_msgs/Float64.h>
#include "ros/ros.h"

//PID Gains
struct Gains{
    double Kp;
    double Ki;
    double Kd;
};

class CT_controller{
public:
    //CT_controller(double Kp1, double Ki1, double Kd1, double Kp2, double Ki2, double Kd2) : gains{{Kp1,Ki1,Kd1},{Kp2,Ki2,Kd2}}{
    CT_controller(){

        //Node Handle in private namespace to get parameters
        nh_private = ros::NodeHandle("~");

        //getting parameters
        nh_private.param<double>("Kp1",gains[0].Kp,0);
        nh_private.param<double>("Ki1",gains[0].Ki,0);
        nh_private.param<double>("Kd1",gains[0].Kd,0);

        nh_private.param<double>("Kp2",gains[1].Kp,0);
        nh_private.param<double>("Ki2",gains[1].Ki,0);
        nh_private.param<double>("Kd2",gains[1].Kd,0);

        nh_private.param<std::string>("trajectory_topic",trajTopic,"/trajectory");
        nh_private.param<std::string>("command_topic",commandTopic,"/command");

        //Topic you want to publish
        commandPublisher = nh.advertise<std_msgs::Float64>(commandTopic, 10);

        //Topic you want to subscribe
        trajSubscriber = nh.subscribe(trajTopic, 1, &CT_controller::callback, this);
    }
    void callback(const std_msgs::Float64& input){
        std::cout << "callback: " << input << "\n";
    }
    void compute(){

    }
    void publish(){
        std_msgs::Float64 output;
        output.data = 1.0;
        commandPublisher.publish(output);
    }
private:
    //ROS Node stuff
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher commandPublisher;
    ros::Subscriber trajSubscriber;
    //Parameters
    Gains gains[2];
    std::string trajTopic, commandTopic;
};

int main(int argc, char** argv){
    //Initialize ROS Node
    ros::init(argc,argv,"ct_controller");

    CT_controller ct_controller;

    //loop
    double f = 10; //rate frequency
    ros::Rate r(f);
    while (ros::ok()){
        //Compute output
        ct_controller.compute();
        //Send command
        ct_controller.publish();

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
/*
 * ROS pkg to control 2R robot joints using a PID controller for trajectory tracking
 * Authors: Joe Ferreira Scholtz and Julio Chi
 */

//includes
#include <cmath>
#include "ros/ros.h"

struct Gains{
    double Kp;
    double Ki;
    double Kd;
}

class PID_controller{
public:
    PID_controller(double Kp1, double Ki1, double Kd1, double Kp2, double Ki2, double Kd2):
    gains[0].Kp(Kp1){

    }
private:
    Gains gains[2];
    double a;

};

int main(int argc, char** argv){
    //roscpp instances
    ros::init(argc,argv,"pid_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    //getting parameters
    double f = 10;
    double Kp1, Ki1, Kd1, Kp2, Ki2, Kd2;

    nh_private<double>("Kp1",Kp1,0);
    nh_private<double>("Ki1",Ki1,0);
    nh_private<double>("Kd1",Kd1,0);

    nh_private<double>("Kp2",Kp2,0);
    nh_private<double>("Ki2",Ki2,0);
    nh_private<double>("Kd2",Kd2,0);

    //instanciate pid_controller
    PID_controller pid_controller(Kp1, Ki1, Kd1, Kp2, Ki2, Kd2);

    //loop
    ros::Rate r(f);
    while (ros::ok()){
        //Compute output
        //Send command
        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
#include <ecn_common/color_detector.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>

using namespace std;
//using ecn::ColorDetector;

cv::Mat im;

bool im_ok;
ros::Time stamp;

void readImage(const sensor_msgs::ImageConstPtr msg)
{
    im_ok = true;
    im = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std_msgs::Header h = msg->header;
    stamp = h.stamp;
}

int main(int argc, char** argv)
{
    // subscribe to images
    ros::init(argc, argv, "color_detector");

    im_ok = false;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imsub = it.subscribe("/robot/camera1/image_raw", 1, &readImage);
    ros::Publisher impub = nh.advertise<geometry_msgs::PointStamped>("masscenter", 1000);
    ros::Publisher impub2 = nh.advertise<geometry_msgs::Vector3>("masscenter_speed", 1000);

    // init color detector
    geometry_msgs::PointStamped mass_center;
    geometry_msgs::PointStamped last_mass_center;
    geometry_msgs::Vector3 mass_center_speed;
    double dt;

    int r = 255, g = 0, b = 0;
    if(argc == 4)
    {
        r = atoi(argv[1]);
        g = atoi(argv[2]);
        b = atoi(argv[3]);
    }
    ecn::ColorDetector cd(r, g, b);
    cd.showSegmentation();  // also gives trackbars for saturation / value
    cd.showOutput();
    cd.fitCircle();

    ros::Rate loop(10);


    while(ros::ok())
    {
        if(im_ok)
        {

            mass_center = cd.process(im);
            mass_center.header.stamp = stamp;

            if(mass_center.header.seq > 0)
            {
                mass_center.point.x = (mass_center.point.x - cd.cam.u0) * cd.cam.ipx;
                mass_center.point.y = (mass_center.point.y - cd.cam.v0) * cd.cam.ipy;
                dt = mass_center.header.stamp.toSec()-last_mass_center.header.stamp.toSec();
                mass_center_speed.x = (mass_center.point.x - last_mass_center.point.x)/dt;
                mass_center_speed.y = (mass_center.point.y - last_mass_center.point.y)/dt;
            }
            impub.publish(mass_center);
            impub2.publish(mass_center_speed);
            last_mass_center = mass_center;

        }

        loop.sleep();
        ros::spinOnce();
    }
}

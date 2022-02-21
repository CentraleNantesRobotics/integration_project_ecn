#include <ecn_common/color_detector.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <geometry_msgs/Point.h>


namespace ecn
{

using std::vector;

void ColorDetector::detectColor(int r, int g, int b)
{
    // convert color to HSV
    const float cmax = std::max(r, std::max(g,b));
    const float cmin = std::min(r, std::min(g,b));
    const float d = cmax - cmin;

    int h = 0;
    if(d)
    {
        if(cmax == r)
            h = 30*(fmod((g-b)/d,6));
        else if(cmax == g)
            h = 30*((b-r)/d + 2);
        else
            h = 30*((r-g)/d + 4);
    }

    // build inRange bounds for hue
    int hthr = 10;
    hue_ = {std::max(h-hthr,0),std::min(h+hthr,179)};

    // other segmentation for h
    if(h < hthr)
    {
        hue_.push_back(179+h-hthr);
        hue_.push_back(179);
    }
    else if(h+hthr > 179)
    {
        hue_.push_back(0);
        hue_.push_back(h+hthr-179);
    }
}

void ColorDetector::showSegmentation()
{
     show_segment_ = true;
     // init display
     cv::namedWindow("Color detector - range");
     cv::createTrackbar( "Saturation", "Color detector - range", &sat_, 255);
     cv::createTrackbar( "Value", "Color detector - range", &val_, 255);
     cv::setTrackbarPos("Saturation", "Color detector - range", 130);
     cv::setTrackbarPos("Value", "Color detector - range", 95);
}



std::vector<cv::Point> ColorDetector::findMainContour(const cv::Mat &_im)
{
    cv::cvtColor(_im, img_, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(img_, img_, cv::Size(9,9), 2);

    if(hue_.size())
    {
        // segment for detection of given RGB (from Hue)
        cv::inRange(img_, cv::Scalar(hue_[0], sat_, val_),
                cv::Scalar(hue_[1], 255, 255), seg1_);
        // add for 2nd detection if near red
        if(hue_.size() == 4)
        {
            cv::inRange(img_, cv::Scalar(hue_[2], sat_, val_),
                    cv::Scalar(hue_[3], 255, 255), seg2_);
            seg1_ += seg2_;
        }

        if(show_segment_)
        {
            cv::imshow("Color detector - range", seg1_);
            if(!show_output_)
                cv::waitKey(1);
        }

        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours( seg1_, contours, hierarchy, CV_RETR_CCOMP,
                          CV_CHAIN_APPROX_SIMPLE);

        // pop all children
        bool found = true;
        while(found)
        {
            found = false;
            for(unsigned int i=0;i<hierarchy.size();++i)
            {
                if(hierarchy[i][3] > -1)
                {
                    found = true;
                    hierarchy.erase(hierarchy.begin()+i,hierarchy.begin()+i+1);
                    contours.erase(contours.begin()+i, contours.begin()+i+1);
                    break;
                }
            }
        }

        if(contours.size())
        {
            // get largest contour
            auto largest = std::max_element(
                        contours.begin(), contours.end(),
                        [](const vector<cv::Point> &c1, const vector<cv::Point> &c2)
            {return cv::contourArea(c1) < cv::contourArea(c2);});
            int idx = std::distance(contours.begin(), largest);

            return contours[idx];
        }
        else
        {
            std::cout << "Color detector: No object was found\n";
        }
    }
    else
    {
        std::cout << "Color detector: RGB to detect was not defined\n";

    }
    return std::vector<cv::Point>();
}


geometry_msgs::Point ColorDetector::process(const cv::Mat &_im)
{
    geometry_msgs::Point mass_center;
    cv::Mat im_proc;
    mass_center = process(_im, im_proc, false);
    return mass_center;
}


geometry_msgs::Point ColorDetector::process(const cv::Mat &_im, cv::Mat &_im_processed, bool write_output)
{
    geometry_msgs::Point mass_center;
    bool circletest = false;
    auto contour = findMainContour(_im);

    if(!contour.size())
    {
        if(show_output_ || write_output)
            _im.copyTo(_im_processed);
        mass_center.z = 0;
        return mass_center;
    }

    if(fit_circle_)
    {
        circletest = true;
        cv::Point2f pt;float radius;
        cv::minEnclosingCircle(contour, pt, radius);

        std::cout << "Found radius: " << radius << std::endl;
        std::cout << "Found Center: " << pt << std::endl;

        // filter output
        x_ = .5*(x_ + pt.x);
        y_ = .5*(y_ + pt.y);
        area_ = .5*(area_ + radius*radius*M_PI);

        mass_center.x = pt.x;
        mass_center.y = pt.y;

        // write if needed
        if(show_output_ || write_output)
        {
            _im.copyTo(_im_processed);
            cv::circle(_im_processed, pt, radius, ccolor, 2);
        }
    }
    else
    {
        cv::Moments m = cv::moments(contour, false);
        // filter output
        x_ = 0.5*(x_ + m.m10/m.m00);
        y_ = 0.5*(y_ + m.m01/m.m00);
        area_ = 0.5*(area_ + m.m00);

        // write if needed
        if(show_output_ || write_output)
        {
            _im.copyTo(_im_processed);
            std::vector<std::vector<cv::Point>> conts(1, contour);
            cv::drawContours(_im_processed, conts, 0, ccolor, 2);
        }
    }

    if(show_output_)
    {
        cv::Moments m = cv::moments(contour, false);
        if (!circletest){
            double x = m.m10/m.m00;
            double y = m.m01/m.m00;
            mass_center.x = x;
            mass_center.y = y;
            std::cout<<"Center of mass coordinates :" << x << " ; " << y << std::endl;
        }
        cv::imshow("Color detector output",_im_processed);
        cv::waitKey(1);
    }
    mass_center.z = 1;
    return mass_center;
}
}

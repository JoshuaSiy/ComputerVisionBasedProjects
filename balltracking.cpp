#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat hsv_image;
  cv::Mat orgimage;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::imshow("image", cv_ptr->image);
  cv::waitKey(3);
  orgimage=cv_ptr->image;
  cv::medianBlur(orgimage, orgimage, 3);
  cv::cvtColor(orgimage, hsv_image, cv::COLOR_BGR2HSV);
  cv::imshow("HSV", hsv_image);
  cv::Mat lower_red_hue_range;
  cv::Mat upper_red_hue_range;
  cv::Mat red_hue_image;
  cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
  cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(255, 255, 255), upper_red_hue_range); 
  cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
  cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
  cv::imshow("rhi", red_hue_image);
  std::vector<Vec3f> circles;
  cv::HoughCircles(red_hue_image, circles, cv::HOUGH_GRADIENT, 1,
                 red_hue_image.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 5, 50 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[0];
        Point center = Point(c[0], c[1]); 
	std::cout << center.x<< ":" << center.y<< std::endl;
        // circle center
        circle( orgimage, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( orgimage, center, radius, Scalar(0,0,255), 3, LINE_AA);
    }


  cv::imshow("original", orgimage);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ROS_INFO("Printing");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
  ros::spin();

}

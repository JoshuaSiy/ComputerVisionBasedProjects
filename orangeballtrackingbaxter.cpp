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
#include <sensor_msgs/image_encodings.h>
#include <imagetransport/Ameter.h>

using namespace cv;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle ah;
  ros::NodeHandle v;
  ros::Publisher expub=v.advertise<imagetransport::Ameter>("/center_offset",1);
  imagetransport::Ameter msg2;
  cv::Mat hsv_image;
  cv::Mat orgimage;
  cv::Mat oranged;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  
  cv::waitKey(3);
  orgimage=cv_ptr->image;
  cv::medianBlur(orgimage, orgimage, 3);
  cv::cvtColor(orgimage, hsv_image, cv::COLOR_BGR2HSV);
  //cv::imshow("HSV", hsv_image);
  cv::inRange(hsv_image, cv::Scalar(3, 100, 100), cv::Scalar(7, 255, 255), oranged);
  cv::GaussianBlur(oranged, oranged, cv::Size(11, 11), 0);
  cv::dilate(oranged, oranged, 0);        // Dilate Filter Effect
  cv::erode(oranged, oranged, 0);         //
  cv::imshow("orangefilter", oranged);
  std::vector<Vec3f> circles;
  cv::HoughCircles(oranged, circles, cv::HOUGH_GRADIENT, 1,
                 oranged.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30,10, 200 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[0];
        Point center = Point(c[0], c[1]); 
	std::cout << center.x-640<< ":" << -(center.y-400)<< std::endl; //center.y will be subjected to change depending on application
        msg2.metx= (center.x-640) *0.0002645833;
        msg2.mety= (center.y-400) *0.0002645833;
        // circle center
        circle( orgimage, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( orgimage, center, radius, Scalar(0,0,255), 3, LINE_AA);
    }
   expub.publish(msg2);
   cv_ptr->encoding = "bgr8";
   cv_ptr->header.frame_id = "/balltracking";
   cv::imshow("original", cv_ptr->image);

   image_transport::ImageTransport bt(ah);
   image_transport::Publisher pub = bt.advertise("/balltracker", 1);
   pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char **argv)
 {
   ros::init(argc, argv, "image_listener");
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   
   image_transport::Subscriber sub = it.subscribe("/cameras/head_camera/image",1, imageCallback);
   ros::spin();

 } 

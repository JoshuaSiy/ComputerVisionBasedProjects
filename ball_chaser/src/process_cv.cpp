#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
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



// Define a global client that can request services
ros::ServiceClient client;
//image_transport::Subscriber sub;
image_transport::Publisher pub;
// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget service;
    service.request.linear_x = lin_x;
    service.request.angular_z = ang_z;
    // Call command_robot service
    if (!client.call(service)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::ImageConstPtr& img)
{
 cv_bridge::CvImagePtr cv_ptr;
 cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); 
 cv::Mat imgOriginal=cv_ptr->image;  // Input image
 cv::imshow("view2", imgOriginal);
 cv::waitKey(30);
 cv::Mat hsvImg;    // HSV Image
 cv::Mat threshImg;   // Thresh Image

 std::vector<cv::Vec3f> v3fCircles;;  // 3 element vector of floats, this will be the pass by reference output of HoughCircles()


 int lowH = 0;       // Set Hue
 int highH = 0;

 int lowS = 0;       // Set Saturation
 int highS = 0;

 int lowV =200;       // Set Value
 int highV = 255;
 cv::cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);
 cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), threshImg);

 cv::GaussianBlur(threshImg, threshImg, cv::Size(3, 3), 0);   //Blur Effect
 cv::dilate(threshImg, threshImg, 0);        // Dilate Filter Effect
 cv::erode(threshImg, threshImg, 0);         // Erode Filter Effect
    float lin_x;
    float ang_z;
  // fill circles vector with all circles in processed image
 cv::HoughCircles(threshImg,v3fCircles,CV_HOUGH_GRADIENT,2,threshImg.rows / 4,100,50,10,100);
//    // Loop through each pixel in the image and check if there's a bright white one
//    // Then, identify if this pixel falls in the left, mid, or right side of the image
//    // Depending on the white ball position, call the drive_bot function and pass velocities to it
//    // Request a stop when there's no white ball seen by the camera

  if(v3fCircles.size()==0){
	float lin_x=0;
	float ang_z=0;
	}
  for (int i = 0; i < v3fCircles.size(); i++) {      // for each circle
               
   std::cout << "Ball position X = "<< v3fCircles[i][0]   // x position of center point of circle
    <<",\tY = "<< v3fCircles[i][1]        // y position of center point of circle
    <<",\tRadius = "<< v3fCircles[i][2]<< "\n";     // radius of circle

                    // draw small green circle at center of object detected
   cv::circle(imgOriginal,            // draw on original image
    cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),  // center point of circle
    3,                // radius of circle in pixels
    cv::Scalar(0, 255, 0),           // draw green
    CV_FILLED);              // thickness

                    // draw red circle around object detected 
   cv::circle(imgOriginal,            // draw on original image
    cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),  // center point of circle
    (int)v3fCircles[i][2],           // radius of circle in pixels
    cv::Scalar(0, 0, 255),           // draw red
    3);                // thickness
    

    if (v3fCircles[i][0]<390 && v3fCircles[i][2]<55) 
	{//Turn Left
  
        ang_z = 0.2;
	}
    else if (v3fCircles[i][0]>410 && v3fCircles[i][2]<55) 
	{
        // Turn Right
        
        ang_z = -0.2;
	}
    else 
	{
          if(v3fCircles[i][2]>55)
		{
    		lin_x= 0;
 		ang_z=0;
		}
	else
		{
		lin_x=0.5;
		ang_z=0;
		}
        }

    drive_robot(lin_x, ang_z);
  } 
cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("threshImg", CV_WINDOW_AUTOSIZE); 

     /* Create trackbars in "threshImg" window to adjust according to object and environment.*/
  cv::createTrackbar("LowH", "threshImg", &lowH, 179); //Hue (0 - 179)
  cv::createTrackbar("HighH", "threshImg", &highH, 179);

  cv::createTrackbar("LowS", "threshImg", &lowS, 255); //Saturation (0 - 255)
  cv::createTrackbar("HighS", "threshImg", &highS, 255);

  cv::createTrackbar("LowV", "threshImg", &lowV, 255); //Value (0 - 255)
  cv::createTrackbar("HighV", "threshImg", &highV, 255);
  

  cv::imshow("imgOriginal", imgOriginal);     // show windows
  cv::imshow("threshImg", threshImg);

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_cv");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    //ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
   image_transport::Subscriber sub=it.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    // Handle ROS communication events
    ros::spin();

    return 0;
}

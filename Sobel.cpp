#include <ros/ros.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include<cmath>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
//using namespace cv;
int xGradient(cv::Mat image, int x, int y)
{
    return image.at<uchar>(y-1, x-1) +
                2*image.at<uchar>(y, x-1) +
                 image.at<uchar>(y+1, x-1) -
                  image.at<uchar>(y-1, x+1) -
                   2*image.at<uchar>(y, x+1) -
                    image.at<uchar>(y+1, x+1);
}

// Computes the y component of the gradient vector
// at a given point in a image
// returns gradient in the y direction

int yGradient(cv::Mat image, int x, int y)
{
    return image.at<uchar>(y-1, x-1) +
                2*image.at<uchar>(y-1, x) +
                 image.at<uchar>(y-1, x+1) -
                  image.at<uchar>(y+1, x-1) -
                   2*image.at<uchar>(y+1, x) -
                    image.at<uchar>(y+1, x+1);
}
int main(int argc, char** argv)
{

    cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
    for(;;)
    {
          cv::Mat frame,frame2_gray ,frame2, dst,grad,grad_x,grad_y,abs_grad_x,abs_grad_y;
          int gx, gy, sum;
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;
          int c;

          cap >> frame;
          if( frame.empty() ) break; // end of video stream
          //cv::imshow("this is you, smile! :)", frame);
          cv::GaussianBlur( frame, frame2, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
          cv::cvtColor(frame2,frame2_gray, CV_BGR2GRAY);
          cv::Sobel( frame2_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
          cv::Sobel( frame2_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
          cv::convertScaleAbs( grad_x, abs_grad_x );
          cv::convertScaleAbs( grad_y, abs_grad_y );
          cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	        cv::namedWindow("final");
          cv::imshow("final", grad);
          if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC

    }

    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}

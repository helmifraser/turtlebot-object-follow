// Pragma once means the source file is only compiled once
#pragma once

//Include all relevant libraries and input/output streams
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <math.h> 

class FollowClass
{
	// Define all private variables, which will only be available to the source file
	private:
  	image_transport::Subscriber image_sub_;
 	ros::Publisher pub_vel;
  	geometry_msgs::Twist cmd_stored;
  	int iLastX, posX, iLastY, posY;
	int iLowH,iHighH,iLowS,iHighS,iLowV,iHighV;
	int iLowHa,iHighHa,iLowSa,iHighSa,iLowVa,iHighVa;

	int go, vector, Slider_Init_Area, morphsize, Colour;
	std::string OPENCV_WINDOW1, OPENCV_WINDOW2, OPENCV_WINDOW3;
  	double dArea, ObjectInitArea;

  	// Define all private variables and methods, these will be available to any class or method which creates the 
	// object.
	public:
	FollowClass(ros::NodeHandle nh_, image_transport::ImageTransport it_);
	~FollowClass();
	void movement();
	float divisor(float number, int power);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);

};

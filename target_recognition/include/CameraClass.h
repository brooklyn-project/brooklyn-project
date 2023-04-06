#pragma once
#ifndef CAMERACLASS_H
#define CAMERACLASS_H

#include <ros/ros.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <vector>
#include "CollectDataClass.h"

struct TargetPixels{
	cv::Point center; // in pixels, int center.x and int center.y
	char faceColor; // r = sad face (critical), g = happy face (noncritical), b = empty
	TargetPixels(cv::Point center, char faceColor) : center(center), faceColor(faceColor){};
};

class Camera {
private:
	//Threshold value for targets
	int hmin_, smin_, vmin_;
	int hmax_, smax_, vmax_;
	ros::Subscriber image_sub;
	ros::Publisher debug_image_pub;


public:
	Camera(ros::NodeHandle *nh);

	~Camera();

	//This function is goig to differentiate between sad and happy face 
	void differentiateFaces(cv::Mat& image, char* faceColor);

	//This function will draw the contour points on an inputted image
	void drawContourOnImage(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, cv::Scalar color, int thickness, int index);

	//This function will obtain the hierachy and contour values using unique interger values (0 - 4) and (1 - 4) (#RetrievalModes, #ContourApproximationModes)
	void contourXandY(const int RetrievalMode, const int ContourApprox, cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);

	//This function is ooing to print out how many targets and critical targets are in the image
	std::vector<TargetPixels> analyse_and_draw(cv::Mat& image);

	void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
};


#endif // !CAMERACLASS_H
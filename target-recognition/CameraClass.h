#pragma once
#ifndef CAMERACLASS_H
#define CAMERACLASS_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include "./CollectDataClass.h"

class Camera {
private:
	//Threshold value for targets
	int hmin_, smin_, vmin_;
	int hmax_, smax_, vmax_;


public:
	Camera();

	~Camera();

	//This function will detect the objects based on an inputted video .mp4 file
	void analyze_and_draw_video(const std::string& videoPath);

	//This function detect objects from the camera
	void analyze_and_draw_camera();

	//This function is goig to differentiate between sad and happy face 
	void differentiateFaces(cv::Mat& image, char* faceColor);

	//This function will draw the contour points on an inputted image
	void drawContourOnImage(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, cv::Scalar color, int thickness, int index);

	//This function will obtain the hierachy and contour values using unique interger values (0 - 4) and (1 - 4) (#RetrievalModes, #ContourApproximationModes)
	void contourXandY(const int RetrievalMode, const int ContourApprox, cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);

	//This function is ooing to print out how many targets and critical targets are in the image
	void analyse_and_draw(cv::Mat& image, ReturnData* resultCollector = nullptr);

};


#endif // !CAMERACLASS_H
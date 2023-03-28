#pragma once
#ifndef CAMERACLASS_H
#define CAMERACLASS_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>

class Camera {
private:
	//Threshold value for targets
	int hmin, smin, vmin;
	int hmax, smax, vmax;

	//This is going to keep the number of tagerts
	int targetFound;

	//This is going to keep the number of critical targets
	int criticalFound;
public:
	Camera();

	~Camera();

//These functions will be used for the senior design program

	//
	
	//This function will detect the objects based on an inputted video .mp4 file
	void analyze_and_draw_video(const std::string & videoPath, const bool debug = false);

	//This function detect objects from the camera
	void analyze_and_draw_camera(bool showImage = true);
	
	//This function is ooing to print out how many targets and critical targets are in the image
	void analyse_and_draw(cv::Mat& image, bool debug = false, bool showImage = false);

	//This function will draw the contour points on an inputted image
	void drawContourOnImage(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, cv::Scalar color, int thickness, int index);

	//This function will create a matrix from an inputted photo path
	cv::Mat createImgMat(const std::string& photoPath);

	//This function will calibrate the thresholds based on the inputted test pictures
	void calibrateFromPhoto(const std::string& photoPath);

	//This function will obtain the hierachy and contour values using the RETR_EXTERNAL (0) and CHAIN_APPROX_NONE (1) (#RetrievalModes, #ContourApproximationModes)
	void contour0and1(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);

	//This function will obtain the hierachy and contour values using the RETR_LIST (1) and CHAIN_APPROX_SIMPLE (2) (#RetrievalModes, #ContourApproximationModes)
	void contour1and2(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);
	
	//This function will obtain the hierachy and contour values using the RETR_CCOMP (2) and CHAIN_APPROX_TC89_L1 (3) (#RetrievalModes, #ContourApproximationModes)
	void contour2and3(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);

	//This function will obtain the hierachy and contour values using the RETR_TREE (3) and CHAIN_APPROX_TC89_KCOS (4) (#RetrievalModes, #ContourApproximationModes)
	void contour3and4(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);

	//This function will obtain the hierachy and contour values using unique interger values (0 - 4) and (1 - 4) (#RetrievalModes, #ContourApproximationModes)
	void contourXandY(const int RetrievalMode, const int ContourApprox, const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy);

	//This function will return if the contour vector has some info
	bool isContoursEmpty(const std::vector<std::vector<cv::Point>>& contour);

	//This function will loop through the contour list and draw the contours onto the image
	void loop_and_draw_contour_and_image(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, const cv::Scalar& toi, const cv::Scalar& crit);


//These functions will be used to debug
	//This function I will write short scripts to test my logic
	void testFunction();

	//This function will be used to return a string that describes the bits of an photo matrix type "From StackOverflow"
	std::string type2str(const int type);

	//This function will be used to print ou the bits of a photo matrix
	void getMatInfo(const cv::Mat& image);

	//This function will be used to print out the bits of a photo path, this function will use the help of the functon above "getMatInfo"
	void getPhotoinfo(const std::string& photoPath);

	//This function will be used to print out the contour values of an image
	void printContours(const std::vector<std::vector<cv::Point>>& controus);

	//This function will calculate the area of contour when traversing through the vector of vectors
	void calcAreaVectorofVector(const std::vector<std::vector<cv::Point>>& contour);

	//This function will set the threshold values all to 0
	void resetThresholdVals();
};


#endif // !CAMERACLASS_H

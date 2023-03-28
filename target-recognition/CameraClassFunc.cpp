#include "./CameraClass.h"

Camera::Camera() {
	//Threshhold for background of TOI after it was calibrated
	hmin = 73, smin = 71, vmin = 0;
	hmax = 179, smax = 255, vmax = 255;
	
	//Initalizing the number of TOI found
	targetFound = 0;

	//Initializing the number of critical TOI
	criticalFound = 0;

	//End of function
	return;
}

Camera::~Camera() {
	std::cout << "\nDeleting the CAMERA object\n";

	return;
}


void Camera::analyze_and_draw_video(const std::string& videoPath, const bool debug) {
	//These are the toi and critical targets
	int toi = 0;
	int criticalTagerts = 0;

	return;
}

void Camera::analyze_and_draw_camera(bool showImage) {
	//Create a camera object
	cv::VideoCapture cameraCapture(0);

	//Check that the camera is working properly
	if (!cameraCapture.isOpened()) {
		std::cout << "The Camera is not working\n";
		exit(1);
	}

	//Create a matrix image
	cv::Mat image, imgHSV, imgMask;

	//Create space for contour and hiercahy
	std::vector<std::vector<cv::Point>> contour;
	std::vector<cv::Vec4i> hiearchy;

	//Color of TOI and Critical
	cv::Scalar toiColor(255, 0, 0); //color blue
	cv::Scalar critColor(0, 0, 255); //color red


	while (true) {
		//Input what the camera is outputting onto the "image" variable
		cameraCapture >> image;

		//Convert to HSV
		cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
		cv::Scalar lower(hmin, smin, vmin);
		cv::Scalar upper(hmax, smax, vmax);
		cv::inRange(imgHSV, lower, upper, imgMask);

		//This will get the contour value
		contourXandY(0, 2, imgMask, contour, hiearchy);

		if (isContoursEmpty(contour)) {
			continue;
		}

		//Loop through the contour
		loop_and_draw_contour_and_image(image, contour, toiColor, critColor);

		//Show the image
		cv::imshow("Live Camera Feed:", image);

		//Break out of the image
		if (cv::waitKey(25) == 27) {
			break;
		}
	}

	return;
}

void Camera::analyse_and_draw(cv::Mat& image, bool debug, bool showImage) {
	//This is the number of toi and critical targets found
	int toi = 0;
	int critical = 0;

	//Create an HSV, and mask
	cv::Mat imgHSV, imgMask;
	cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
	cv::Scalar lower(hmin, smin, vmin);
	cv::Scalar upper(hmax, smax, vmax);
	cv::inRange(imgHSV, lower, upper, imgMask);

	//Obtain the contour points
	std::vector<std::vector<cv::Point>> contour;
	std::vector<cv::Vec4i> hiearchy;

	//This will get the contour value
	contourXandY(0, 2, imgMask, contour, hiearchy);

	//Checks if the contour array is empty
	if (isContoursEmpty(contour)) {
		if (debug) {
			std::cout << "The image has #toi: " << toi << " and #critical: " << critical << "\n\n";
		}
		return;
	}

	//This will be the color that will be used to box in the toi or critical target
	cv::Scalar toiColor(255, 0, 0 ); //color blue
	cv::Scalar critColor(0, 0, 255); //color red

	//Printing the number of contour points 
	//std::cout << "This contour list has: " << contour.size() << " # of sets\n";

	//Loop to go through every contour list and identify what is a toi and which one is critical target
	for (size_t i = 0; i < contour.size(); i++) {
		//Getting the contour area 
		double contourArea = cv::contourArea(contour[i]);

		//This will print out the 
		//std::cout << "The contour area for i: " <<  i + 1 << " is: " << contourArea << "\n";

		if (contourArea < 2000) { //This is a random blob
			continue;
		}else if (contourArea < 2300) { //This is a toi
			toi++;
			if (showImage) {
				drawContourOnImage(image, contour, toiColor, 2, (int)i);
			}
		}else if (contourArea < 2500) { //This is a critical
			toi++;
			critical++;
			if (showImage) {
				drawContourOnImage(image, contour, critColor, 2, (int)i);
			}
		}
	}

	//This will print out information about the inputted message
	if (debug) {
		std::cout << "The image has #toi: " << toi << " and #critical: " << critical << "\n\n";
	}
	
	//Then show the image
	if (showImage) {
		cv::imshow("New Image", image);
		cv::waitKey(0);
	}

	return;
}

void Camera::drawContourOnImage(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, cv::Scalar color, int thickness, int index) {
	cv::drawContours(image, contour, index, color, thickness);

	return;
}


cv::Mat Camera::createImgMat(const std::string& photoPath) {
	cv::Mat image = cv::imread(photoPath);

	if (image.empty()) {
		std::cout << "\nThe image: " << photoPath << " does not exists!\n";
		exit(1);
	}

	return image;
}

void Camera::calibrateFromPhoto(const std::string& photoPath) {
	//Read in the image
	cv::Mat inputtedImage = cv::imread(photoPath);

	//Check if the image didnt show up
	if (inputtedImage.empty()) {
		std::cout << "\nThe inputted image: " << photoPath << " did not open!\n";
		exit(1);
	}

	//Create the HSV of the mask
	cv::Mat imgHSV;
	cv::cvtColor(inputtedImage, imgHSV, cv::COLOR_BGR2HSV);

	//This is the creation of the imgMask
	cv::Mat imgMask;

	//Calibrate the mask threshold values with trackbars
	cv::namedWindow("Trackbars", (640, 200));
	cv::createTrackbar("Hue Min", "Trackbars", &hmin, 179);
	cv::createTrackbar("Hue Max", "Trackbars", &hmax, 179);
	cv::createTrackbar("Sat Min", "Trackbars", &smin, 255);
	cv::createTrackbar("Sat Max", "Trackbars", &smax, 255);
	cv::createTrackbar("Val Min", "Trackbars", &vmin, 255);
	cv::createTrackbar("Val Max", "Trackbars", &vmax, 255);

	//Adjusting the values of the threshold
	while (true) {
		//New threshold values
		cv::Scalar lower(hmin, smin, vmin);
		cv::Scalar upper(hmax, smax, vmax);

		//This is the mask
		cv::inRange(imgHSV, lower, upper, imgMask);

		//Display the mask to the user
		cv::imshow("Mask Photo", imgMask);

		//Waitkey if "esc" is pressed the program will end
		if (cv::waitKey(30) == 27) {
			break;
		}
	}

	//Setting the values from the trackback to the private variables
	hmin = cv::getTrackbarPos("Hue Min", "Trackbars");
	hmax = cv::getTrackbarPos("Hue Max", "Trackbars");
	smin = cv::getTrackbarPos("Sat Min", "Trackbars");
	smax = cv::getTrackbarPos("Sat Max", "Trackbars");
	vmin = cv::getTrackbarPos("Val Min", "Trackbars");
	vmax = cv::getTrackbarPos("Val Max", "Trackbars");

	//The next part is to obtain the contours values, need to allocate space for contours and hiearchy
	std::vector<std::vector<cv::Point>> contour;
	std::vector<cv::Vec4i> hiearchy;
	//Print out the contour value from "contour0and1"
	contour0and1(imgMask, contour, hiearchy);

	//Print out the contour value from "contour0and1"
	contour1and2(imgMask, contour, hiearchy);

	//Print out the contour value from "contour0and1"
	contour2and3(imgMask, contour, hiearchy);

	//Print out the contour value from "contour0and1"
	contour3and4(imgMask, contour, hiearchy);

	return;
}

void Camera::contour0and1(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy) {
	std::cout << "\nPrinting the contour points using: RETR_EXTERNAL and CHAIN_APPROX_NONE!\n";
	cv::findContours(imageMask, contour, hiearchy, 0, 1);
	printContours(contour);

	return;
}

void Camera::contour1and2(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy) {
	std::cout << "\nPrinting the contour points using: RETR_LIST and CHAIN_APPROX_SIMPLE!\n";
	cv::findContours(imageMask, contour, hiearchy, 1, 2);
	printContours(contour);

	return;
}

void Camera::contour2and3(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy) {
	std::cout << "\nPrinting the contour points using: RETR_CCOMP and CHAIN_APPROX_TC89_L1!\n";
	cv::findContours(imageMask, contour, hiearchy, 2, 3);
	printContours(contour);

	return;
}

void Camera::contour3and4(const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy) {
	std::cout << "\nPrinting the contour points using: RETR_TREE and CHAIN_APPROX_TC89_KCOS!\n";
	cv::findContours(imageMask, contour, hiearchy, 3, 4);
	printContours(contour);

	return;
}

void Camera::contourXandY(int RetrievalMode, int ContourApprox, const cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy) {
	cv::findContours(imageMask, contour, hiearchy, RetrievalMode, ContourApprox);
	//printContours(contour);

	return;
}

bool Camera::isContoursEmpty(const std::vector<std::vector<cv::Point>>& contour) {
	return contour.size() == 0;
}

void Camera::loop_and_draw_contour_and_image(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, const cv::Scalar& toi, const cv::Scalar& crit) {
	//Loop to go through every contour list and identify what is a toi and which one is critical target
	for (size_t i = 0; i < contour.size(); i++) {
		//Getting the contour area 
		double contourArea = cv::contourArea(contour[i]);

		//This will print out the 
		//std::cout << "The contour area for i: " <<  i + 1 << " is: " << contourArea << "\n";

		if (contourArea < 2000) { //This is a random blob
			continue;
		}
		else if (contourArea < 2300) { //This is a toi
			drawContourOnImage(image, contour, toi, 2, (int) i);
			
		}
		else if (contourArea < 2500) { //This is a critical
			drawContourOnImage(image, contour, crit, 2, (int) i);
			
		}
	}

	return;
}


void Camera::testFunction() {
	/*
	This function is where I will do testing
	*/

	//Create a dark image
	cv::Mat image(500, 500, 16, cv::Scalar(255, 255, 255));
	cv::Rect recta(30, 30, 100, 200);
	cv::rectangle(image, recta, cv::Scalar(0, 0, 0), -1, 8, 0);

	//Images
	cv::Mat imgMask;
	cv::Mat imgHSV;

	//Changing the image
	cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);

	//Creating the mask
	cv::Scalar lower(hmin, smin, vmin);
	cv::Scalar upper(hmax, smax, vmax);
	cv::inRange(imgHSV, lower, upper, imgMask);
	

	//Get the contour points
	std::vector<std::vector<cv::Point>> contour;
	std::vector<cv::Vec4i> hiearchy;
	cv::findContours(imgMask, contour, hiearchy, 0, 2);
	printContours(contour);

	//Calculate the area of each contour in the "contour" vector of vector 
	calcAreaVectorofVector(contour);

	//Display the image
	cv::imshow("New image", image);

	cv::waitKey(0);
	

	//End of function
	return;
}

std::string Camera::type2str(const int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

void Camera::getMatInfo(const cv::Mat& image) {
	std::string result = type2str(image.type());
	std::cout << "Matrix: " << result.c_str() << " " << image.cols << " " << image.rows << "\n";

	return;
}

void Camera::getPhotoinfo(const std::string& photoPath) {
	cv::Mat inputtedPhoto = cv::imread(photoPath);

	if (inputtedPhoto.empty()) {
		std::cout << "\nThe inputted photo: " << photoPath << " is not real!\n";
		exit(1);
	}

	getMatInfo(inputtedPhoto);

	return;
}

void Camera::printContours(const std::vector<std::vector<cv::Point>>& contour) {
	for (size_t i = 0; i < contour.size(); i++) {
		std::cout << "Contour set: " << i + 1 << " has the following contour points\n";
		for (size_t j = 0; j < contour[i].size(); j++) {
			std::cout << contour[i][j] << "\n";
		}
		std::cout << "\n";
	}
	
	return;
}

void Camera::calcAreaVectorofVector(const std::vector<std::vector<cv::Point>>& contour) {
	for (size_t i = 0; i < contour.size(); i++) {
		std::cout << "The area for i = " << i + 1 << " is: " << cv::contourArea(contour[i]) << std::endl;
	}

	return;
}

void Camera::resetThresholdVals() {
	hmin = 0, smin = 0, vmin = 0;
	hmax = 0, smax = 0, vmax = 0;

	return;
}
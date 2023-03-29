#include "./CameraClass.h"

Camera::Camera() {
	//Threshhold for background of TOI after it was calibrated
	hmin = 73, smin = 71, vmin = 144;
	hmax = 111, smax = 255, vmax = 255;
	
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


void Camera::analyze_and_draw_video(const std::string& videoPath) {
	//These are the toi and critical targets
	int toi = 0;
	int criticalTargets = 0;

	//Create a video capture
	cv::VideoCapture vid_input(videoPath);

	if (!vid_input.isOpened()) {
		std::cout << "Error opening vidoe stream!\n\n";
		exit(1);
	}

	std::cout << "The Frame Rate: " << vid_input.get(5) << " frames per second\n";
	std::cout << "The Frame count: " << vid_input.get(7) << " \n\n";

	//Create an image
	cv::Mat image;
	cv::Mat resizeImage;
	
	//resize values
	int down_width = 500;
	int down_height = 500;


	//loop through the images
	while (vid_input.isOpened()) {

		//Check that the 
		if (!vid_input.read(image)) {
			std::cout << "Unsuccessful at bragging the frame\n\n";
			break;
		}

		//Change the image to display it
		analyse_and_draw(image, &toi, &criticalTargets);

		//Display the change
		//cv::resize(image, image.size() / 2);
		cv::imshow("New Updated Video: ", image);

		//Break the loop
		if (cv::waitKey(25) == 27) {
			break;
		}
	}

	//It will be an output message about what the algorithm found
	std::cout << "The algorithm found #toi: " << toi << " and #critical: " << criticalTargets << " in the video: " << videoPath << "\n\n";

	//Release the vidoe capture
	vid_input.release();
	cv::destroyAllWindows();

	return;
}

void Camera::analyze_and_draw_camera() {
	//Create a camera object
	cv::VideoCapture cameraCapture(0);

	//Check that the camera is working properly
	if (!cameraCapture.isOpened()) {
		std::cout << "The Camera is not working\n";
		exit(1);
	}

	//Create a matrix image
	cv::Mat image;


	while (true) {
		//Input what the camera is outputting onto the "image" variable
		cameraCapture >> image;

		//Call the analyze_and_draw function and will change the image
		analyse_and_draw(image);
	
		//It will show the modified image from "analyze_and_draw" function
		cv::imshow("Live Feed", image);

		if (cv::waitKey(25) == 27) {
			break;
		}
	
	}

	//Release the camera
	cameraCapture.release();

	//Closes all the windows
	cv::destroyAllWindows();

	return;
}

void Camera::analyse_and_draw(cv::Mat& image, int* toiNum, int* critNum, bool debug, bool showImage) {
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
	contourXandY(3, 1, imgMask, contour, hiearchy);

	////Print out the contour and hiearchy
	//printContours(contour);
	//printHiearchy(hiearchy);

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

	//Loop through the contour points
	for (size_t i = 0; i < contour.size(); i++) {	
		//If the current contour has no "First-Child" and "Parent" then it is just a target
		if ((hiearchy[i][2] == -1) && (hiearchy[i][3] == -1) && (cv::contourArea(contour[i]) > 1200)) {
			drawContourOnImage(image, contour, toiColor, 2, (int) i);
			
			//Changing the counter values when reading the video
			if (toiNum != nullptr) {
				*toiNum = *toiNum + 1;
			}
			else {
				toi++;
			}

		}else if((hiearchy[i][2] != -1) && (cv::contourArea(contour[i]) > 1200)) { //If the current contour has "First-child"
			//std::cout << "The area inside the outerContour is: " << cv::contourArea(contour[hiearchy[i][2]]);

			//Seeing that the inner area of the contour is small for a circle and that 
			if (cv::contourArea(contour[hiearchy[i][2]]) > 200 && hiearchy[hiearchy[i][2]][3] == i) {
				//Changing the counter values when reading the video
				if (toiNum != nullptr && critNum != nullptr) {
					*toiNum = *toiNum + 1;
					*critNum = *critNum + 1;
				}
				else {
					toi++;
					critical++;
				}

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

void Camera::changeImageSize(cv::Mat& image, int width, int length) {
	//This is the resized image
	cv::resize(image, image, cv::Size(width, length), 1);

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

	////The next part is to obtain the contours values, need to allocate space for contours and hiearchy
	//std::vector<std::vector<cv::Point>> contour;
	//std::vector<cv::Vec4i> hiearchy;
	////Print out the contour value from "contour0and1"
	//contour0and1(imgMask, contour, hiearchy);

	////Print out the contour value from "contour0and1"
	//contour1and2(imgMask, contour, hiearchy);

	////Print out the contour value from "contour0and1"
	//contour2and3(imgMask, contour, hiearchy);

	////Print out the contour value from "contour0and1"
	//contour3and4(imgMask, contour, hiearchy);

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

//void Camera::loop_and_draw_contour_and_image(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, const cv::Scalar& toi, const cv::Scalar& crit) {
//	//Loop to go through every contour list and identify what is a toi and which one is critical target
//	for (size_t i = 0; i < contour.size(); i++) {
//		//Getting the contour area 
//		double contourArea = cv::contourArea(contour[i]);
//
//		//This will print out the 
//		//std::cout << "The contour area for i: " <<  i + 1 << " is: " << contourArea << "\n";
//
//		if (contourArea < 2000) { //This is a random blob
//			continue;
//		}
//		else if (contourArea < 2300) { //This is a toi
//			drawContourOnImage(image, contour, toi, 2, (int) i);
//			
//		}
//		else if (contourArea < 2500) { //This is a critical
//			drawContourOnImage(image, contour, crit, 2, (int) i);
//			
//		}
//	}
//
//	return;
//}


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
		std::cout << "Contour set: " << i << " has the following contour points\n";
		for (size_t j = 0; j < contour[i].size(); j++) {
			std::cout << contour[i][j] << "\n";
		}
		std::cout << "\n";
	}
	
	return;
}

void Camera::printHiearchy(const std::vector<cv::Vec4i>& hiearchy) {
	for (size_t i = 0; i < hiearchy.size(); i++) {
		std::cout << "This is the hiearchy for set: " << i << " has the following\n";
		std::cout << "hiearchy[" << i << "][0] = " << hiearchy[i][0] << "\n";
		std::cout << "hiearchy[" << i << "][1] = " << hiearchy[i][1] << "\n";
		std::cout << "hiearchy[" << i << "][2] = " << hiearchy[i][2] << "\n";
		std::cout << "hiearchy[" << i << "][3] = " << hiearchy[i][3] << "\n\n";
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
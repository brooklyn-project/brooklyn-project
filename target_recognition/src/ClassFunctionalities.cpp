#include "ros/ros.h"
#include "CameraClass.h"


Camera::~Camera() {
	std::cout << "\nDeleting the CAMERA object\n";
	return;
}

void Camera::differentiateFaces(cv::Mat& image, char* faceColor) {
	//These are the templates that will be used to be compared with the image
	cv::Mat happyReference = cv::imread("../resources/SmileTemplate.jpg");
	cv::Mat sadReference = cv::imread("../resources/SadTemplate.jpg");
	cv::Mat happyResult, sadResult;

	//These are going to be values needed for the happy result
	double HminVal, HmaxVal;
	cv::Point HminLoc, HmaxLoc;

	//These are going to be values needed for the sad result
	double SminVal, SmaxVal;
	cv::Point SminLoc, SmaxLoc;

	//Getting the result of comparing it to happyReference
	cv::matchTemplate(image, happyReference, happyResult, cv::TM_CCOEFF_NORMED);

	//Getting the result of comparing it to sadReference
	cv::matchTemplate(image, sadReference, sadResult, cv::TM_CCOEFF_NORMED);

	//Obtain the correct values for happy result
	cv::minMaxLoc(happyResult, &HminVal, &HmaxVal, &HminLoc, &HmaxLoc);

	//Obtain the correct values for the sad result
	cv::minMaxLoc(sadResult, &SminVal, &SmaxVal, &SminLoc, &SmaxLoc);

	if (HmaxVal > SmaxVal) {
		*faceColor = 'g';
	}
	else {
		*faceColor = 'r';
	}

	return;
}

void Camera::drawContourOnImage(cv::Mat& image, const std::vector<std::vector<cv::Point>>& contour, cv::Scalar color, int thickness, int index) {
	
	//This will draw on the image that was inputted
	cv::drawContours(image, contour, index, color, thickness);

	return;
}



void Camera::contourXandY(const int RetrievalMode, const int ContourApprox, cv::Mat& imageMask, std::vector<std::vector<cv::Point>>& contour, std::vector<cv::Vec4i>& hiearchy) {
	
	cv::findContours(imageMask, contour, hiearchy, RetrievalMode, ContourApprox);

	return;
}

std::vector<TargetPixels> Camera::analyse_and_draw(cv::Mat& image) {

	std::vector<TargetPixels> res;
	//Create an HSV, and mask
	cv::Mat imgHSV, imgMask;
	cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
	cv::Scalar lower(hmin_, smin_, vmin_);
	cv::Scalar upper(hmax_, smax_, vmax_);
	cv::inRange(imgHSV, lower, upper, imgMask);

	//Obtain the contour points
	std::vector<std::vector<cv::Point>> contour;
	std::vector<cv::Vec4i> hiearchy;

	//This will get the contour value
	contourXandY(3, 1, imgMask, contour, hiearchy);

	//Checks if the contour array is empty
	if (contour.size() == 0) {
		return res;
	}

	//This will be the color that will be used to box in the toi or critical target
	cv::Scalar toiColor(255, 0, 0); //color blue
	cv::Scalar sadColor(0, 0, 255); //color red
	cv::Scalar happyColor(0, 255, 0); //color green

	//Loop through the contour point sets
	for (size_t i = 0; i < contour.size(); i++) {

		//If the current contour has no "First-Child" and "Parent". And the contourArea is greater than 1200, then it is a blue toi
		if ((hiearchy[i][2] == -1) && (hiearchy[i][3] == -1) && (cv::contourArea(contour[i]) > 1200)) {

			char color = 'b';

			drawContourOnImage(image, contour, toiColor, 2, (int)i); //draw the contour

			//if this is nullptr then we collect data
			// if (resultCollector != nullptr) {
				//Calculate the center point of the picture
				cv::Moments M = cv::moments(contour[i]);
				cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
				res.push_back(TargetPixels(center, color));

				// //Input the center of this contour and add it to a list that we already have
				// resultCollector->returnPixels()->addCenterToList(center);

				// //Add the color of the toi that was found here
				// resultCollector->returnPixels()->addColorToList(color);

			// }

		}
		else if ((hiearchy[i][2] != -1) && (cv::contourArea(contour[i]) > 1200) && cv::contourArea(contour[hiearchy[i][2]]) > 200 && hiearchy[hiearchy[i][2]][3] == i) {
			//If the current contour has a "First-Child" and area of the First-Child is greater than 200 and the parent of the child is the current contour then it is a critical target

			//This will store wether it is a happy or sad
			char faceColor;

			//Calculate the center point of the target of interest
			cv::Moments M = cv::moments(contour[i]);
			cv::Point center(M.m10 / M.m00, M.m01 / M.m00);

			//Obtain critical target that is enclosed by the rectangle
			cv::Rect rectangleOfInterest = cv::boundingRect(contour[i]);

			//Create ROI that will only hold the face
			cv::Mat croppedImage = image(rectangleOfInterest);

			//We have to differientiate between a happy and sad face
			differentiateFaces(croppedImage, &faceColor);
			res.push_back(TargetPixels(center, faceColor));


			if (faceColor == 'g') { //Happy Face and draw it on
				drawContourOnImage(image, contour, happyColor, 2, (int)i);
			}
			else { //Sad Face and draw it on
				drawContourOnImage(image, contour, sadColor, 2, (int)i);
			}

			// //If resultCollector is not equal to nullptr then we start to collect data to be displayed
			// if (resultCollector != nullptr) {
			// 	//Input the center of this contour and add it to a list that we already have
			// 	resultCollector->returnPixels()->addCenterToList(center);

			// 	//Add the color of the toi that was found here
			// 	resultCollector->returnPixels()->addColorToList(faceColor);

			// }
		}
	}

	// if (resultCollector != nullptr) {
	// 	cv::imshow("New image: ", image);
	// 	cv::waitKey(0);

	// }


	//Return a pointer of this data type
	return res;
}

// Define the ROS subscriber callback function
void Camera::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert the ROS image message to an OpenCV Mat object
  cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8)->image;
  ros::Time imgTimestamp = msg->header.stamp;

  // Call the analyse_and_draw function of the Camera object, passing in the OpenCV Mat object
  std::vector<TargetPixels> targetsPixels = this->analyse_and_draw(image);
  debug_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgba8", image).toImageMsg());

  // Do some processing with targetsPixels here @cameron and @luca

  // Then publish the latitude and longitude data so the map_generator_node can read it
}

Camera::Camera(ros::NodeHandle *nh) {
	//Threshhold for background of TOI after it was calibrated
	hmin_ = 73, smin_ = 71, vmin_ = 144;
	hmax_ = 111, smax_ = 255, vmax_ = 255;
	ROS_INFO("Subscribing to /camera/image");
	image_sub = nh->subscribe<sensor_msgs::Image>("/camera/image", 10, &Camera::cameraCallback, this);
	debug_image_pub = nh->advertise<sensor_msgs::Image>("/analyzed_image", 10);
	//End of function
	return;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "./GPSClass.h"

GPS::GPS() {

}

GPS::~GPS() {
	std::cout << "\nDeleting this GPS class object\n";
}

void GPS::setLatandLong(double* lat, double* log) {
	*latitude_ = *lat;
	*longitude_ = *log;

	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "./PixelsClass.h"
Pixels::Pixels() {

}

Pixels::~Pixels() {
	std::cout << "\nDeleting this PIXELS class object\n";
}


void Pixels::addCenterToList(const cv::Point& centerOfPoint) {
	//The inputted "centerOfPoint" will be added to the list
	centerCoordinatesPixels_.push_back(centerOfPoint);

	//End of function
	return;
}

void Pixels::addColorToList(const char& color) {
	//The inputted "color" will be added to the list
	colorCorresponding_.push_back(color);

	//End of function
	return;
}

void Pixels::deleteCenterData() {
	centerCoordinatesPixels_.clear();

	return;
}

void Pixels::deleteColorData() {
	colorCorresponding_.clear();

	return;
}

cv::Point* Pixels::pointCoordinateatI(int i) {
	return &centerCoordinatesPixels_[i];
}

char* Pixels::colorAtI(int i) {
	return &colorCorresponding_[i];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "./CollectDataClass.h"
ReturnData::ReturnData() {

}

ReturnData::~ReturnData() {
	std::cout << "\nDeleting this RETURNDATA class object\n";
}

void ReturnData::deletePrivateVariablesData() {
	//Delete the private variables data
	pi_.deleteCenterData();
	pi_.deleteColorData();


	return;
}

void ReturnData::printTheResultingData() {
	if (pi_.sizeOfCenterCoordinatesPixels_() != pi_.sizeOfcolorCorresponding_()) {
		std::cout << "The number of values of centroid and color are not the name!, time to debug\n";
		exit(1);
	}

	//Loop through the results
	for (int i = 0; i < pi_.sizeOfcolorCorresponding_(); i++) {
		std::cout << "Centroid# " << i + 1 << " is located at: " << *pi_.pointCoordinateatI(i) << " and a color of: " << *pi_.colorAtI(i) << "\n";
	}

	std::cout << "\n\n";

	return;
}
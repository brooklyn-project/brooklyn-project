#include "./CameraClass.h"

int main(int argc, char** argv) {

	std::vector<std::string> pictureArray{ "./resources/noTarget.jpg", "./resources/oneTarget.jpg", "./resources/twoTargets.jpg", \
		"./resources/targetFaceSad.jpg", "./resources/targetFaceSmile.jpg", \
		"./resources/critical_and_one_toi.jpg", "./resources/critical_and_two_toi.jpg"};
	std::string photoPath = "./resources/critical_and_one_toi.jpg";
	std::string videoPath = "./resources/search_phase.mp4";
	std::vector<std::string> videoArray {"./resources/search_phase.mp4"};
	Camera camera = Camera();
	bool debug = true;
	bool showImage = true;

	////Test the analyze_and_draw from an inputted video
	//for (std::string video : videoArray) {
	//	if (debug) {
	//		std::cout << "The video: " << video << " has the following toi and critical targets:\n";
	//	}

	//	//This will run the algorithm based on the inputted video
	//	camera.analyze_and_draw_video(video, debug);
	//}

	////Try to read in the video that was clipped
	//camera.analyze_and_draw_video(videoPath);

	////Play aroung with the camera input on my laptop
	//camera.analyze_and_draw_camera();

	////Looping through every image of the array
	//for (std::string image : pictureArray) {
	//	//Create a mat image
	//	cv::Mat matImage = camera.createImgMat(image);

	//	if (debug) {
	//		//Print out a prompt to the user
	//		std::cout << "The image: " << image << " has the following toi and critical targets:\n";
	//	}

	//	//Input that into teh "analyze_and_draw" function
	//	camera.analyse_and_draw(matImage, nullptr, nullptr, debug, showImage);

	//}

	////This to test analyze_and_draw() for one image
	//cv::Mat image = camera.createImgMat(photoPath);
	//camera.analyse_and_draw(image, nullptr, nullptr, debug, showImage);

	////This is to calibrate the camera
	//camera.calibrateFromPhoto(photoPath);


	////This is needed to test with my function
	//camera.resetThresholdVals();
	//camera.testFunction();
	return 0;
}
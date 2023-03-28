#include "./CameraClass.h"

int main(int argc, char** argv) {

	std::vector<std::string> pictureArray{ "./resources/noTarget.jpg", "./resources/oneTarget.jpg", "./resources/targetFace.jpg", "./resources/twoTargets.jpg", "./resources/targetFaceSmile.jpg" };
	std::string photoPath = "./resources/targetFace.jpg";
	std::string videoPath = "./resources/FlightTestFootage.mp4";
	std::vector<std::string> videoArray{"./resources/video1.mp4", "./resources/video2.mp4", "./resources/video3.mp4", "./resources/video4.mp4", "./resources/video5.mp4"};
	Camera camera = Camera();
	bool debug = true;
	bool showImage = false;

	////Test the analyze_and_draw from an inputted video
	//for (std::string video : videoArray) {
	//	if (debug) {
	//		std::cout << "The video: " << video << " has the following toi and critical targets:\n";
	//	}

	//	//This will run the algorithm based on the inputted video
	//	camera.analyze_and_draw_video(video, debug);
	//}

	////Play aroung with the camera input
	//camera.analyze_and_draw_camera(showImage);

	//Looping through every image of the array
	for (std::string image : pictureArray) {
		//Create a mat image
		cv::Mat matImage = camera.createImgMat(image);

		if (debug) {
			//Print out a prompt to the user
			std::cout << "The image: " << image << " has the following toi and critical targets:\n";
		}

		//Input that into teh "analyze_and_draw" function
		camera.analyse_and_draw(matImage, debug, showImage);

	}

	////This is needed to test with my function
	//camera.resetThresholdVals();
	//camera.testFunction();

	return 0;
}
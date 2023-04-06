#include "ros/ros.h"
#include "CameraClass.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	//Camera object
	Camera camera = Camera();
	ros::spin();


	////Try to read in the video that was clipped
	//camera.analyze_and_draw_video(videoPath);

	////Play aroung with the camera input on my laptop
	//camera.analyze_and_draw_camera();

	////Looping through every image of the array
	//for (std::string image : pictureArray) {
	//	//Create a mat image
	//	cv::Mat matImage = cv::imread(image);

	//	//Input that into the "analyze_and_draw" function
	//	camera.analyse_and_draw(matImage, &endResult);

	//	//We can print out data from endResult. We can also make it a parameter so the data type is created only once, depends on how to implement it
	//	endResult.printTheResultingData();

	//	//Delete all the private variables information
	//	endResult.deletePrivateVariablesData();

	//}

	////This to test analyze_and_draw() for one image
	//cv::Mat image = camera.createImgMat(photoPath);
	//camera.analyse_and_draw(image);




	return 0;
}
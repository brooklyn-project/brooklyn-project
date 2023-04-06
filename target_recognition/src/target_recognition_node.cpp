#include "ros/ros.h"
#include "CameraClass.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "image_processor_node");
	ros::NodeHandle n;
	//Camera object
	Camera camera = Camera(&n);
	ros::spin();
	return 0;
}
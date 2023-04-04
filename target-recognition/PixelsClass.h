#pragma once
#ifndef PIXELSCLASS_H
#define PIXELSCLASS_H


class Pixels {
private:
	//This is the list that will store the set of pixels
	std::vector<cv::Point> centerCoordinatesPixels_;

	//This is going to store the color
	std::vector<char> colorCorresponding_;

public:
	//Constructor
	Pixels();

	//Destructor
	~Pixels();

	//This function is going to keep track of the center points of each toi found
	void addCenterToList(const cv::Point& centerOfPoint);

	//This function is going to keep track of the color of each toi found
	void addColorToList(const char& color);

	//This fuction will delete the center data
	void deleteCenterData();

	//This funcion will delete the color data
	void deleteColorData();

	//This function will return the size that is stored in "centerCoordinatesPixels_"
	int sizeOfCenterCoordinatesPixels_() { return static_cast<int>(centerCoordinatesPixels_.size()); };

	//This function will return the size that is stored in "colorCorresponding_"
	int sizeOfcolorCorresponding_() { return static_cast<int>(colorCorresponding_.size()); };

	//This function will return the point at a i position
	cv::Point* pointCoordinateatI(int i);

	//This function will return the color of that toi
	char* colorAtI(int i);

};

#endif // !PIXELSCLASS_H

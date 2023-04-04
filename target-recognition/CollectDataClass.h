#pragma once
#ifndef COLLECTDATACLASS_H
#define COLLECTDATACLASS_H

#include "./CameraClass.h"
#include "./GPSClass.h"
#include "./PixelsClass.h"

class ReturnData {
private:
	Pixels pi_;
	GPS gps_;
public:

	ReturnData();

	~ReturnData();

	//This function will return the address of the Pixel object
	Pixels* returnPixels() { return &pi_; };

	//This function will return the address of the GPS object when it is needed later
	GPS* returnGPS() { return &gps_; };

	//This function will make it private variables to delete its own data for another frame
	void deletePrivateVariablesData();

	//This function will print out the data that was found inside the private variables
	void printTheResultingData();
};

#endif
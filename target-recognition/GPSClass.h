#pragma once
#ifndef GPSCLASS_H
#define GPSCLASS_H


class GPS {
private:
	//Coordiantes
	double* latitude_;
	double* longitude_;

public:
	GPS();

	~GPS();

	//This will set the values of latitude_ and longitude_ private variables
	void setLatandLong(double* lat, double* log);

};

#endif
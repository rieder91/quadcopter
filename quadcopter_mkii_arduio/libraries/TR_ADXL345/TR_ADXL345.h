#ifndef TR_ADXL345_H
#define TR_ADXL345_H

#include "Arduino.h"

class TR_ADXL345 {
public:
	float gains[3];
	int offset[3];
	
	TR_ADXL345();
	
	void init();
	void calibrate();
	void readAccel(int* xyz);
	void readAccel(int* x, int* y, int* z);
	void readAccelCalibrated(int* x, int* y, int* z);
	void get_xyz(float *xyz);
	
	// I2C Stuff
	int readReverseShortI2C();
	void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue);
	void sendByteI2C(int deviceAddress, byte dataValue);
};

#endif
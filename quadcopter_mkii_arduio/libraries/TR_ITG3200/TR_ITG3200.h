#pragma once

#include "Arduino.h"

class TR_ITG3200 {
public:
	int offset[3];
	
	float gyroTemperature;
	float gain[3];
	
	TR_ITG3200();
	
	void init();
	void calibrate();
	void readGyro(int* x, int* y, int* z);
	void readGyroRaw(int* x, int* y, int* z);
	void readGyroTemp();
	
	// I2C Stuff
	int readShortI2C();
	int readWordI2C();
	void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue);
	void sendByteI2C(int deviceAddress, byte dataValue);
};
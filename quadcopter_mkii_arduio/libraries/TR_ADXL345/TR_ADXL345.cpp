#include "TR_ADXL345.h"
#include <Wire.h>

TR_ADXL345::TR_ADXL345() {
	gains[0] = 0.00376390;
	gains[1] = 0.00376009;
	gains[2] = 0.00349265;
}

void TR_ADXL345::init() {
	updateRegisterI2C(0x53, 0x2D, 1<<3);	// set device to *measure*
	updateRegisterI2C(0x53, 0x31, 0x09);	// set full range and +/- 4G
	updateRegisterI2C(0x53, 0x2C, 8+2+1);	// 200hz sampling
	delay(10);
}

void TR_ADXL345::calibrate() {
	int i, x, y, z, count = 100;
	float xSum = 0.0, ySum = 0.0, zSum = 0.0;
	
	for(i = 0; i < count; i++) {
		readAccel(&x, &y, &z);
		xSum += x;
		ySum += y;
		zSum += z;
		delay(10);
	}
	
	offset[0] = xSum / count;
	offset[1] = ySum / count;
	offset[2] = zSum / count;
}

void TR_ADXL345::readAccel(int* xyz) {
	readAccel(xyz, xyz + 1, xyz + 2);
}

void TR_ADXL345::readAccel(int *x, int *y, int *z) {
	sendByteI2C(0x53, 0x32);
	Wire.requestFrom(0x53, 6);
	
	*x = readReverseShortI2C();
	*y = readReverseShortI2C();
	*z = readReverseShortI2C();
}


void TR_ADXL345::readAccelCalibrated(int *x, int *y, int *z) {
	sendByteI2C(0x53, 0x32);
	Wire.requestFrom(0x53, 6);
	
	*x = readReverseShortI2C();
	*y = readReverseShortI2C();
	*z = readReverseShortI2C();
	
	*x += offset[0];
	*y += offset[1];
	*z += offset[2];
}


void TR_ADXL345::get_xyz(float* xyz) {
	int readings[3];
	readAccel(readings);
	
	for(int i = 0; i < 3; i++) {
		xyz[i] = readings[i] * gains[i];
	}
}


// I2C Stuff (inspiried by ArduQuad)
int TR_ADXL345::readReverseShortI2C() {
	return (signed short)( Wire.read() | (Wire.read() << 8));
}

void TR_ADXL345::updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue) {
	Wire.beginTransmission(deviceAddress);
	Wire.write(dataAddress);
	Wire.write(dataValue);
	Wire.endTransmission();
} 

void TR_ADXL345::sendByteI2C(int deviceAddress, byte dataValue) {
	Wire.beginTransmission(deviceAddress);
	Wire.write(dataValue);
	Wire.endTransmission();
}
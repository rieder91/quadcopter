#include "TR_ITG3200.h"
#include <Wire.h>


/**
  * default constructor. gain is not set at the moment
  *
  */
TR_ITG3200::TR_ITG3200() {
	gain[0] = 1.0;
	gain[1] = 1.0;
	gain[2] = 1.0;
}


/**
  * Initalises the gyro using i2c with the following configuration:
  *  - reset
  *  - 20 hz low pass filtering (0x1D is 10Hz, 0x1B is 42Hz and so forth)
  *  - using the internal oscillator
  *
  * Duration: ~50ms
  */
void TR_ITG3200::init() {
	updateRegisterI2C(0x69, 0x3E, 0x80); // reset
	updateRegisterI2C(0x69, 0x16, 0x1C); // 20Hz low pass filter; 
	updateRegisterI2C(0x69, 0x3E, 0x01); // use internal oscillator 
	delay(50);
}


/**
  * Reads the temperature of the gyro into gyroTemperature
  *
  */
void TR_ITG3200::readGyroTemp() {
  sendByteI2C(0x69, 0x1B);
  Wire.requestFrom(0x69, 2);
  gyroTemperature = (readWordI2C() + 13200) / 280 + 35;
}


/**
  *
  * Takes 128 samples and determines and offset to use for the gyro.
  * Duration: ~640ms
  *
  */
void TR_ITG3200::calibrate() {
	int x, y, z, i;
	int xSum = 0, ySum = 0, zSum = 0;
	
	for(i = 0; i < 128; i++) {
		delay(5);
		readGyroRaw(&x, &y, &z);
		xSum += x;
		ySum += y;
		zSum += z;
	}
	
	offset[0] = -xSum / 128;
	offset[1] = -ySum / 128;
	offset[2] = -zSum / 128;
}


/**
  * Reads the gyro data using i2c. no post-processing
  *
  */
void TR_ITG3200::readGyroRaw(int *x, int *y, int *z) {
	sendByteI2C(0x69, 0x1D);
	Wire.requestFrom(0x69, 6);
	
	*x = readShortI2C();
	*y = readShortI2C();
	*z = readShortI2C();
}


/**
  * Reads the gyro data and does post-processing using the offset and the
  * constant conversion factor of 1/14.375
  *
  */
void TR_ITG3200::readGyro(int *x, int *y, int *z) {
	readGyroRaw(x, y, z);
	
	// Calculated Offset
	*x += offset[0];
	*y += offset[1];
	*z += offset[2];
	
	*x /= 14.375;
	*y /= 14.375;
	*z /= 14.375;
}



// I2C Stuff (inspiried by ArduQuad)
int TR_ITG3200::readShortI2C() {
	  return (signed short)readWordI2C();
}


int TR_ITG3200::readWordI2C() {
	return (Wire.read() << 8) | Wire.read();
}


void TR_ITG3200::sendByteI2C(int deviceAddress, byte dataValue) {
	Wire.beginTransmission(deviceAddress);
	Wire.write(dataValue);
	Wire.endTransmission();
}


void TR_ITG3200::updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue) {
	Wire.beginTransmission(deviceAddress);
	Wire.write(dataAddress);
	Wire.write(dataValue);
	Wire.endTransmission();
}
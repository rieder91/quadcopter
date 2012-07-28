#ifndef TR_IMUFILTER_H
#define TR_IMUFILTER_H

#include "Arduino.h"

#include <TR_ITG3200.h>
#include <TR_ADXL345.h>

class TR_IMUFilter {
	TR_ITG3200 gyro;
	TR_ADXL345 accel;
	
	int gyroX, gyroY, gyroZ;
	int accelX, accelY, accelZ;
	
	// Stuff from FreeIMU DCM
	float iq0, iq1, iq2, iq3;
	float exInt, eyInt, ezInt;		// scaled integral error
	volatile float twoKp;			// 2 * proportional gain (Kp)
	volatile float twoKi;			// 2 * integral gain (Ki)
	volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	volatile float integralFBx, integralFBy, integralFBz;
	unsigned long lastUpdate, now;	// sample period expressed in milliseconds
	float sampleFreq;				// half the sample period expressed in seconds
	int startLoopTime;
	
public:
	TR_IMUFilter();
	
	void init();
	void getReadings();
	void print(boolean accel_debug, boolean gyro_debug);
	void getQuaternion(float* q);
	void updateAHRS(float gx, float gy, float gz, float ax, float ay, float az);
	void getEuler(float* angles);
	void getRPY(float* angles);
};

// inverted sqrt taken from quake3
float invSqrt(float number);

#endif
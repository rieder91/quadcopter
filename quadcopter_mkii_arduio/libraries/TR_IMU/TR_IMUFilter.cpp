#include "TR_IMUFilter.h"

/**
  * Straight DCM implementation based on FreeIMU
  *
  */

/**
  * Initializes the quaternion
  */
TR_IMUFilter::TR_IMUFilter() {
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	
	twoKp = 2.0f * 0.5f;
	twoKi = 2.0f * 0.1f;
	integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
	
	lastUpdate = 0;
	now = 0;
}


/**
  * Inits the accel and gyro
  */
void TR_IMUFilter::init() {
	accel.init();
	delay(250);
	accel.calibrate();
	
	gyro.init();
	delay(250);
	gyro.calibrate();
}


/**
  * Reads the gyro and accel value into the member variables
  */
void TR_IMUFilter::getReadings() {
	gyro.readGyro(&gyroX, &gyroY, &gyroZ);
	accel.readAccel(&accelX, &accelY, &accelZ);
}


/**
  * Computes the euler angles derived from the quaternion
  */
void TR_IMUFilter::getEuler(float* angles) {
	float q[4];
	getQuaternion(q);
	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}


/**
  * Updates and outputs the current quaternion into q
  */
void TR_IMUFilter::getQuaternion(float* q) {
	getReadings();
	
	now = micros();
	sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
	lastUpdate = now;
	updateAHRS(gyroX * M_PI / 180, gyroY * M_PI/180, gyroZ * M_PI/180, accelX, accelY, accelZ);
	
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}


/**
  * Returns the roll pitch and yaw angles
  *
  */
void TR_IMUFilter::getRPY(float* angles) {
	float q[4];
	float gx, gy, gz;
	
	getQuaternion(q);
	
	gx = 2 * (q[1]*q[3] - q[0]*q[2]);
	gy = 2 * (q[0]*q[1] + q[2]*q[3]);
	gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
	
	angles[0] = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
	angles[1] = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
	angles[2] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
}


/**
  * Taken from FreeIMU
  */
void TR_IMUFilter::updateAHRS(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float qa, qb, qc;
	
	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
	float halfvx, halfvy, halfvz;
	
	// Normalise accelerometer measurement
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
	// Estimated direction of gravity
	halfvx = q1q3 - q0q2;
	halfvy = q0q1 + q2q3;
	halfvz = q0q0 - 0.5f + q3q3;
	
	// Error is sum of cross product between estimated direction and measured direction of field vectors
	halfex += (ay * halfvz - az * halfvy);
	halfey += (az * halfvx - ax * halfvz);
	halfez += (ax * halfvy - ay * halfvx);
	}
	
	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
	// Compute and apply integral feedback if enabled
	if(twoKi > 0.0f) {
		integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
		integralFBy += twoKi * halfey * (1.0f / sampleFreq);
		integralFBz += twoKi * halfez * (1.0f / sampleFreq);
		gx += integralFBx;  // apply integral feedback
		gy += integralFBy;
		gz += integralFBz;
	}
	else {
		integralFBx = 0.0f; // prevent integral windup
		integralFBy = 0.0f;
		integralFBz = 0.0f;
	}
	
	// Apply proportional feedback
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/**
  * Debugging for accel and gyro
  */
void TR_IMUFilter::print(boolean accel_debug, boolean gyro_debug) { 
	getReadings();
	if(gyro_debug) {
		Serial.print("Gyro: ");
		Serial.print(gyroX);
		Serial.print('\t');
		Serial.print(gyroY);
		Serial.print('\t');
		Serial.print(gyroZ);
		Serial.print('\n');
	}
	if(accel_debug) {
		Serial.print("Accel: ");
		Serial.print(accelX);
		Serial.print('\t');
		Serial.print(accelY);
		Serial.print('\t');
		Serial.print(accelZ);
		Serial.print('\n');
	}
}


/**
  * Inverted sqrt first seen in quake3
  */
float invSqrt(float number) {
	volatile long i;
	volatile float x, y;
	volatile const float f = 1.5F;
	
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f375a86 - ( i >> 1 );
	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	return y;
}
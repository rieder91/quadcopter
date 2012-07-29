#include <Wire.h>

// IMU
#include <TR_ADXL345.h>
#include <TR_ITG3200.h>
#include <TR_IMUFilter.h>

// Motors
#include <Servo.h>
#include <PID_v1.h>

boolean debug_imu = true;
boolean debug_motors = false;

boolean use_imu = true;
boolean use_motors = true;

int i, j;

TR_IMUFilter imu;
float angles[3];
double xAngle, yAngle;

int xOffsetIMU = -6;
int yOffsetIMU = 0;

Servo esc_x1;
Servo esc_x2;
Servo esc_y1;
Servo esc_y2;

int esc_x1_pin = 3; // passt
int esc_x2_pin = 11; // passt
int esc_y1_pin = 10; // passt
int esc_y2_pin = 9; // passt

double xSpeed;
double ySpeed;
double xPIDSpeed;
double yPIDSpeed;

double targetAngle = 180.0;

PID xPID(&xAngle, &xPIDSpeed, &targetAngle, 1, 0, 0.2, DIRECT);
PID yPID(&yAngle, &yPIDSpeed, &targetAngle, 1, 0, 0.2, DIRECT);

void setup() {
  // Init serial
  Serial.begin(115200);
  
  if(use_imu) {
    Wire.begin();
    
    // Takes ~500ms
    imu.init();
    i = 0;
  }
  
  if(use_motors) {
    esc_x1.attach(esc_x1_pin);
    esc_x2.attach(esc_x2_pin);
    esc_y1.attach(esc_y1_pin);
    esc_y2.attach(esc_y2_pin);
    
    // set speed to zero
    setSpeedX(1000);
    esc_y1.writeMicroseconds(1000);
    esc_y2.writeMicroseconds(1000);
    
    xSpeed = 1300;
    ySpeed = 1300;
    
    xPID.SetOutputLimits(-25, 25);
    xPID.SetMode(AUTOMATIC);
    xPID.SetSampleTime(5);
    
    yPID.SetOutputLimits(-25, 25);
    yPID.SetMode(AUTOMATIC);
    yPID.SetSampleTime(5);
    
    
    delay(1000);
    setSpeedX(1200);
    setSpeedY(1200);
    delay(1000);
    setSpeedX(1300);
    setSpeedY(1300);
    
    delay(1000);
    //esc_x1.writeMicroseconds(1400);
    //esc_x2.writeMicroseconds(1400);
    //esc_y1.writeMicroseconds(1400);
    //esc_y2.writeMicroseconds(1400);
    
    delay(1000);

    // set speed to zero
    setSpeedX(1000);
    setSpeedY(1000);
    
    j = 0;
  }
}



void loop() {
  if(use_imu) {
    imu.getRPY(angles);
    xAngle = angles[1] + 180 + xOffsetIMU;
    yAngle = angles[0] + 180 + yOffsetIMU;
  }
  
  if(use_motors) {
    xPID.Compute();
    yPID.Compute();
    setSpeedESC(esc_x1, 1300 - xPIDSpeed/2);
    setSpeedESC(esc_x2, 1300 + xPIDSpeed/2);
    //esc_x1.writeMicroseconds(1300 - xPIDSpeed/2);
    //esc_x2.writeMicroseconds(1300 + xPIDSpeed/2);
  }
  
  if(debug_imu) {
    // Anti-Spam
    if(i == 75) {
      Serial.print("X: ");
      Serial.print(xAngle);
      Serial.print('\t Y:');
      Serial.print(yAngle);
      Serial.print('\n');
      i = 0;  
    }
    i++;
  }
  
  if(debug_motors) {
    if(j == 75) {
      j = 0;
      Serial.print("x-PID: ");
      Serial.print(xPIDSpeed);
      Serial.print("\t y-PID: ");
      Serial.print(yPIDSpeed);
      Serial.print('\n');
    }
    j++;
  }
}


void setSpeedX(int ms) {
  esc_x1.writeMicroseconds(ms);
  esc_x2.writeMicroseconds(ms);
}

void setSpeedY(int ms) {
  esc_y1.writeMicroseconds(ms);
  esc_y2.writeMicroseconds(ms);
}

void setSpeedESC(Servo s, int ms) {
  s.writeMicroseconds(ms);
}

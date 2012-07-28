#include <Wire.h>

// IMU
#include <TR_ADXL345.h>
#include <TR_ITG3200.h>
#include <TR_IMUFilter.h>

boolean debug_imu = true;
boolean use_imu = true;


TR_IMUFilter imu;
int i;

void setup() {
  // Init serial
  Serial.begin(115200);
  
  if(use_imu) {
    Wire.begin();
    
    // Takes ~500ms
    imu.init();
    i = 0;
  }
}

void loop() {
  float angles[3];
  imu.getRPY(angles);
  
  if(debug_imu) {
    // Anti-Spam
    if(i == 20) {
      imu.print(false, true);
      Serial.print(angles[0]);
      Serial.print('\t');
      Serial.print(angles[1]);
      Serial.print('\t');
      Serial.print(angles[2]);
      Serial.print('\n');
      i = 0;  
    }
    i++;
  }
}

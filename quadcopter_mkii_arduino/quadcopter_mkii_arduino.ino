#include <Wire.h>

// IMU
#include <TR_ADXL345.h>
#include <TR_ITG3200.h>
#include <TR_IMUFilter.h>

// Motors
#include <Servo.h>
#include <PID_v1.h>

boolean debug_imu = false;
boolean debug_motors = false;
boolean debug_serial = false;

boolean use_imu = true;
boolean use_motors = true;
boolean en_motors = false;

int i, j, k;

TR_IMUFilter imu;
float angles[3];
double xAngle, yAngle;

int xOffsetIMU = -4;
int yOffsetIMU = -3;

Servo esc_x1;
Servo esc_x2;
Servo esc_y1;
Servo esc_y2;

int esc_x1_pin = 3;    // verified
int esc_x2_pin = 11;   // verified
int esc_y1_pin = 10;   // verified
int esc_y2_pin = 9;    // verified

double xSpeed = 1000;
double ySpeed = 1000;

double xPIDSpeed;
double yPIDSpeed;

double targetAngleX = 180.0;
double targetAngleY = 180.0;

//PID xPID(&xAngle, &xPIDSpeed, &targetAngleX, 0.4, 0.05, 0.2, DIRECT); // .7, .1, .2
//PID yPID(&yAngle, &yPIDSpeed, &targetAngleY, 0.4, 0.05, 0.2, DIRECT);

PID xPID(&xAngle, &xPIDSpeed, &targetAngleX, 0.7, 0.0, 0.0, DIRECT); // .7, .1, .2
PID yPID(&yAngle, &yPIDSpeed, &targetAngleY, 0.7, 0.0, 0.0, DIRECT);



//PID yPID(&yAngle, &yPIDSpeed, &targetAngleY, 1.0, 0.05, 0.2, DIRECT);

// Previous Settings
//PID xPID(&xAngle, &xPIDSpeed, &targetAngle, 0.25, 0.05, 0.1, DIRECT); // .7, .1, .2
//PID xPID(&xAngle, &xPIDSpeed, &targetAngle, 0.55, 0.1, 0.3, DIRECT); // .7, .1, .2


// Remote Control Stuff

// Contains the issued command
char command[6];

// Contains the command-parameter
char value[6];

char tempChar;
int number;

boolean flush_needed = false;



void setup() {
  // Init serial
  Serial.begin(19200);
  
  if(use_imu) {
    Wire.begin();
    
    // Takes ~500ms
    imu.init();
    
    // IMU debug variable
    i = 0;
  }
  
  if(use_motors) {
    esc_x1.attach(esc_x1_pin);
    esc_x2.attach(esc_x2_pin);
    esc_y1.attach(esc_y1_pin);
    esc_y2.attach(esc_y2_pin);
    
    // set speed to zero
    setSpeedX(xSpeed);
    setSpeedY(ySpeed);
    
    xPID.SetOutputLimits(-40, 40);
    xPID.SetMode(AUTOMATIC);
    xPID.SetSampleTime(5);
    
    yPID.SetOutputLimits(-20, 20);
    yPID.SetMode(AUTOMATIC);
    yPID.SetSampleTime(5);
    
    delay(500);
    
    // Motor debug variable
    j = 0;
  }
  
  Serial.print("Setup complete\n");
}



void loop() {
  if(use_imu) {
    imu.getRPY(angles);
    xAngle = angles[1] + 180 + xOffsetIMU;
    yAngle = angles[0] + 180 + yOffsetIMU;
  }
  
  // Emergency Shutdown
  /*if(xAngle <= 110 || xAngle >= 250 || yAngle <= 110 || yAngle >= 250) {
    Serial.print("failsafe_triggered\n");
    en_motors = false;
    setSpeedX(1000);
    setSpeedY(1000);
  }*/
  
  if(use_motors && en_motors) {
    xPID.Compute();
    yPID.Compute();
    if(xPIDSpeed < 0) {
      setSpeedESC(esc_x1, xSpeed - xPIDSpeed/2);
      setSpeedESC(esc_x2, xSpeed + xPIDSpeed);
    } else {
      setSpeedESC(esc_x1, xSpeed - xPIDSpeed);
      setSpeedESC(esc_x2, xSpeed + xPIDSpeed/2);
    }
    setSpeedESC(esc_y1, ySpeed + yPIDSpeed/2);
    setSpeedESC(esc_y2, ySpeed - yPIDSpeed/2);
    
    /*
    if(xSpeed < 1250) {
      xPID.SetOutputLimits(-20, 20);
    } else if(xSpeed >= 1250 && xSpeed < 1350) {
      xPID.SetOutputLimits(-12, 12);
    } else if(xSpeed >= 1350 && xSpeed < 1400) {
      xPID.SetOutputLimits(-8, 8);
    } else if(xSpeed >= 1400 && xSpeed < 2000) {
      xPID.SetOutputLimits(-5, 5);
    }
    
    if(ySpeed < 1250) {
      yPID.SetOutputLimits(-20, 20);
    } else if(ySpeed >= 1250 && ySpeed < 1350) {
      yPID.SetOutputLimits(-12, 12);
    } else if(ySpeed >= 1350 && ySpeed < 1400) {
      yPID.SetOutputLimits(-8, 8);
    } else if(ySpeed >= 1400 && ySpeed < 2000) {
      yPID.SetOutputLimits(-5, 5);
    }
    */
    /*if(ySpeed < 1250) {
      yPID.SetOutputLimits(-50, 50);
    } else if(ySpeed >= 1250 && ySpeed < 1350) {
      yPID.SetOutputLimits(-30, 30);
    } else if(ySpeed >= 1350 && ySpeed < 1400) {
      yPID.SetOutputLimits(-15, 15);
    } else if(ySpeed >= 1400 && ySpeed < 2000) {
      yPID.SetOutputLimits(-10, 10);
    }*/
  } else {
    setSpeedX(1000);
    setSpeedY(1000);  
  }
  
  
  // Read the serial buffer
  if(Serial.available() >= 10) {
    // 10 bytes available
    
    k = 0;
    while(k < 5) {
      tempChar = Serial.read(); 
      command[k] = tempChar;
      k++;
      command[k] = '\0';
      if(tempChar == '-') {
        flush_needed = true;
      }
    }
    
    k = 0;
    while(k < 5) {
      tempChar = Serial.read();
      value[k] = tempChar;
      k++;
      value[k] = '\0';
    }
    
    // convert the second parameter to int
    number = atoi(&value[0]);
  }
  
  if(flush_needed) {
    Serial.print("flushing_ser_cache\n");
    
    // Clear Buffer
    while(Serial.available()>0) {
      Serial.read();
    }
    
    flush_needed = false;
    number = 0;
  }
  
  
  if(number != 0) {
    // X-Axis
    if(strcmp(command, "setXB") == 0) {
      Serial.print("setXB.Ack;");
      if(number >= 1000 && number <= 2000) {
        xSpeed = number;
      }
      Serial.print(xSpeed);
      Serial.print("\n");
    } 
    
    // Y-Axis
    else if(strcmp(command, "setYB") == 0) {
      Serial.print("setYB.Ack;");
      if(number >= 1000 && number <= 2000) {
        ySpeed = number;
      }
      Serial.print(ySpeed);
      Serial.print("\n");
    } 
    
    // Both Axis
    else if(strcmp(command, "setBB") == 0) {
      Serial.print("setBB.Ack;");
      if(number >= 1000 && number <= 2000) {
        xSpeed = number;
        ySpeed = number;
      }
      Serial.print(xSpeed);
      Serial.print("\n");
    } 
    
    // Set Angles
    else if(strcmp(command, "setXA") == 0) {
      Serial.print("setXA.Ack;");
      if(number >= 140 && number <= 220) {
        targetAngleX = number;
      }
      Serial.print(targetAngleX);
      Serial.print("\n");
    } else if(strcmp(command, "setYA") == 0) {
      Serial.print("setYA.Ack;");
      if(number >= 140 && number <= 220) {
        targetAngleY = number;
      }
      Serial.print(targetAngleY);
      Serial.print("\n");
    }
    
    // Individual Servos - testing only
    else if(strcmp(command, "setX1") == 0) {
      Serial.print("setX1.Ack\n");
      if(number >= 1000 && number <= 2000) {
        setSpeedESC(esc_x1, number);
      }
    } else if(strcmp(command, "setX2") == 0) {
      Serial.print("setX2.Ack\n");
      if(number >= 1000 && number <= 2000) {
        setSpeedESC(esc_x2, number);
      }
    } else if(strcmp(command, "setY1") == 0) {
      Serial.print("setY1.Ack\n");
      if(number >= 1000 && number <= 2000) {
        setSpeedESC(esc_y1, number);
      }
    } else if(strcmp(command, "setY2") == 0) {
      Serial.print("setY2.Ack\n");
      if(number >= 1000 && number <= 2000) {
        setSpeedESC(esc_y2, number);
      }
    }
   
   
   // IMU Offsets
  else if(strcmp(command, "setXO") == 0) {
      Serial.print("setXO.Ack;");
      if(number >= -20 && number <= 20) {
        xOffsetIMU = number;
      }
      Serial.print(xOffsetIMU);
      Serial.print("\n");
    } else if(strcmp(command, "setYO") == 0) {
      Serial.print("setYO.Ack;");
      if(number >= -20 && number <= 20) {
        yOffsetIMU = number;
      }
      Serial.print(yOffsetIMU);
      Serial.print("\n");
    } 
    
    // Enable Motors
    else if(strcmp(command, "enMTR") == 0) {
      Serial.print("enMTR.Ack\n");
      if(number == 1) {
        en_motors = false;
      } else if (number == 2) {
        en_motors = true;
      }
    } 
    
    // Debug 
    else if(strcmp(command, "dbIMU") == 0) {
      Serial.print("dbIMU.Ack\n");
      if(number == 1) {
        debug_imu = false;
      } else if(number == 2) {
        debug_imu = true;
      }
    } else if(strcmp(command, "dbMTR") == 0) {
      Serial.print("dbMTR.Ack\n");
      if(number == 1) {
        debug_motors = false;
      } else if(number == 2) {
        debug_motors = true;
      }
    } else if(strcmp(command, "dbSER") == 0) {
      Serial.print("dbSER.Ack\n");
      if(number == 1) {
        debug_serial = false;
      } else if(number == 2) {
        debug_serial = true;
      }
    }
  }
  
  debug();
    
  command[0] = '\0';
  value[0] = '\0';
  number = 0;
}

void debug() {
  if(debug_imu) {
    // Anti-Spam
    if(i == 100) {
      //imu.print(true, true);
      Serial.print("X:");
      Serial.print(xAngle);
      Serial.print(";Y:");
      Serial.print(yAngle);
      Serial.print('\n');
      i = 0;  
    }
    i++;
  }
  
  if(debug_motors) {
    if(j == 100) {
      j = 0;
      Serial.print("x-PID:");
      Serial.print(xPIDSpeed);
      Serial.print(";y-PID:");
      Serial.print(yPIDSpeed);
      Serial.print('\n');
    }
    j++;
  }
  
  if(debug_serial) {
    if(strcmp(command, "") != 0) {
      Serial.print("Command:");
      Serial.print(command);
      Serial.print(";Value:");
      Serial.print(value);
      Serial.print('\n');
    }
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

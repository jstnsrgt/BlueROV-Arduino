#include <Servo.h>
#include "Conversions.h"
#include "MotorSpeedDefinitions.h"


#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>

//Map the PWM pins on the board a corresponding propeller pin name
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;


RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;


//Choose 4 analogue pins for battery cell voltage input
#define BATTERY_INPUT_1 A4 //Cell 1
#define BATTERY_INPUT_2 A5 //Cell 2
#define BATTERY_INPUT_3 A6 //Cell 3
#define BATTERY_INPUT_4 A7 //Cell 4



//Each output will be treated as a servo controller
//as the Blue Robitics Basic ESC accepts PWM as input
Servo propAll[6];

/* ESC INPUT
 * PWM duty cycle
 * ESC translates microseconds of pulse width into motor output
 * Stop Motor:    1500 Microsecond PW
 * Forward:       ~ 1525(1%) to 1900(100%) Microsecond PW
 * Reverse:       ~ 1475(1%) down to 1100(100%) Mircosecond PW
 */
 int accelY, accelX;

void setup() {
  // Telling each servo object which port to use
  int errcode;
  
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
 
  
  propAll[0].attach(propPin1);
  propAll[1].attach(propPin2);
  propAll[2].attach(propPin3);
  propAll[3].attach(propPin4);
  propAll[4].attach(propPin5);
  propAll[5].attach(propPin6);

  


  propAll[0].writeMicroseconds(MOTOR_STOP);
  propAll[1].writeMicroseconds(MOTOR_STOP);
  propAll[2].writeMicroseconds(MOTOR_STOP);
  propAll[3].writeMicroseconds(MOTOR_STOP);
  propAll[4].writeMicroseconds(MOTOR_STOP);
  propAll[5].writeMicroseconds(MOTOR_STOP);

  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.5);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(false);
  
  delay(5000);
  
  // Wait for speed controllers to initialise
}


void loop() {
  unsigned long now = millis();
    unsigned long delta;
    int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        sampleCount++;
        if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.println(", gyro bias valid");
            else
                Serial.println(", calculating gyro bias");
        
            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
            RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
           Serial.println();
        }
    }
}


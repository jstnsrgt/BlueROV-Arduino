#include <Servo.h>
#include <Wire.h>
#include "Conversions.h"
#include "MotorSpeedDefinitions.h"
#include <math.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

RTVector3 rpy; //object containing a 3 element array of floats for roll, pitch and yaw

int setRoll, setPitch, setYaw; //target
int roll, pitch, yaw;
int cRollErr,cPitchErr,cYawErr; //current
int pRollErr,pPitchErr,pYawErr; //previous

//stores control system output for each thruster
int pid1Adj;
int pid2Adj;

//time keeping variables, rollover protection not required - takes > 50 days
int currentMilli, previousMilli;

//gains may be implemented again
float pGainRoll = 0.25; //set the gain for the speed based on the level correction changes
float pGainPitch = 0.25; //set the gain for the speed based on the level correction changes
float pGainYaw = 1; //set the gain for the speed based on the level correction changes
float dGainRoll = 0.12; //set the gain for the speed based on the level correction changes
float dGainPitch = 0.12; //set the gain for the speed based on the level correction changes
float dGainYaw = 1; //set the gain for the speed based on the level correction changes

void setup() {
  Serial.begin(115200);
  Wire.begin(); //for I2C communication with pressure and IMU sensors

  // create the imu object
  imu = RTIMU::createIMU(&settings);
  
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.5);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  setRoll = rpy.y();
  setPitch = rpy.x();
  
  pinMode(13,OUTPUT); //Ready LED

  //ten second buffer to stabilise speed controllers and also in case of program upload
   
  currentMilli = millis(); //begin tracking time
}


void loop() {
  imu->IMURead();
  delay(50);
  //Read sensor inputs
  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  roll = rpy.y();    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  pitch = rpy.x();   //value decreses pitching down, increases pitching up

  //roll over current error values into previous values
  pRollErr = cRollErr;
  pPitchErr = cPitchErr;
  
  //update current error values
  cRollErr = roll - setRoll;
  cPitchErr = pitch - setPitch;

//NEITHER PITCH NOR ROLL WORKS CORRECTLY
    pid1Adj = -(cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll) + cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch
    pid2Adj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll + cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch
  
  Serial.print("Motor 1 adjustment: ");
  Serial.println(pid1Adj);
  
  Serial.print("Motor 2 adjustment: ");
  Serial.println(pid2Adj);
  Serial.println();
  
  Serial.print("Current Roll Error: ");
  Serial.println(cRollErr);
  
  Serial.print("Current Pitch Error: ");
  Serial.println(cPitchErr);
  Serial.println();

  Serial.print("Previous Roll Error: ");
  Serial.println(pRollErr);
  
  Serial.print("Previous Pitch Error: ");
  Serial.println(pPitchErr);
  Serial.println();

  Serial.print("(Roll, Pitch) : (");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.println(")");
  
  delay(1000);
}

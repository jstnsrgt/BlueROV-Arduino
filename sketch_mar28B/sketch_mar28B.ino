#include <MS5837.h>
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

MS5837 pSensor;
//probably unnecessary unless tested in seawater
#define FRESHWATER 997
#define SEAWATER 1029


//pin terminal masks
byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;


int setRoll, setPitch, setYaw;
int roll, pitch, yaw;
int cRollErr,cPitchErr,cYawErr;
int pRollErr,pPitchErr,pYawErr;

int cErr5;
int pErr5;
int pid5Adj;

int pid1Adj;
int pid2Adj;
int pid3Adj;
int pid4Adj;

float currentDepth, previousDepth;
int currentMilli, previousMilli;


float targetDepth;
float startDepth;


int prop1Output = MOTOR_HOVER_START_1;
int prop2Output = MOTOR_HOVER_START_2;
int prop5Output = MOTOR_HOVER_START_5;

//gains may be implemented again
float pGainRoll = 1; //set the gain for the speed based on the level correction changes
float pGainPitch = 1; //set the gain for the speed based on the level correction changes
float pGainYaw = 1; //set the gain for the speed based on the level correction changes
float dGainRoll = 1; //set the gain for the speed based on the level correction changes
float dGainPitch = 1; //set the gain for the speed based on the level correction changes
float dGainYaw = 1; //set the gain for the speed based on the level correction changes
float pGain5 = 1; //set the gain for the PID speed based on the vertical equilibrium changes
float dGain5 = 2; //

Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

void setup() {
  Wire.begin();

  
  // create the imu object
  imu = RTIMU::createIMU(&settings);
  
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  setRoll = rpy.x();
  setPitch = rpy.y();
  setYaw = rpy.z();
  
  pSensor.init();
  pSensor.setFluidDensity(FRESHWATER);
  
  pinMode(13,OUTPUT); //Ready LED

  
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);
  prop5.attach(propPin5);
  prop6.attach(propPin6);

  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);
  prop5.writeMicroseconds(MOTOR_STOP);
  prop6.writeMicroseconds(MOTOR_STOP);

  delay(1000);
   
  pSensor.read(); //read sensor
  currentDepth = startDepth = pSensor.depth(); //set initial depth  
  targetDepth = startDepth - 0.15; //set target depth
  
  currentMilli = millis(); //begin tracking time
  
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);
 
  delay(40); //delay for good measure
  cErr5 = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target
  
  
}
void loop() {
  imu->IMURead();
  delay(50);
  //Read sensor inputs
  pSensor.read(); //Pressure
  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  roll = rpy.x()*100;    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  pitch = rpy.y()*100;   //value decreses pitching down, increases pitching up
  yaw = rpy.z()*100;

  pRollErr = cRollErr;
  pPitchErr = cPitchErr;
  pYawErr = cYawErr;
  
  cRollErr = roll - setRoll;
  cPitchErr = pitch - setPitch;
  cYawErr = yaw - setYaw;

  //an example of code layout in a future version
  //Hover(&propSpeed, pSensor.depth(), xInput, yInput);
  
  //Save information from last cycle and assign info for current cycle
  previousDepth = currentDepth;
  currentDepth = pSensor.depth();

  pErr5 = cErr5;

  //Now in cm/s

  //meters to cm
  cErr5 = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target

  pid1Adj = -(cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll) + cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch;
  pid2Adj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll + cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch;
  pid3Adj = -(cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw);
  pid4Adj = cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw;
  pid5Adj = cErr5*pGain5 + (cErr5 - pErr5)*dGain5;

  
  //speed is in centimeters per second, directly based on how many cemtimeters away the robot is
  // targetVelocity = pDiff/2 cm/s;
  
 
  //Depth Control
  //Gauge how far away robot needs to go
  //Gauge speed of robot
  
  //for every cm away robot is, target velocity is that distance per second
  prop1Output = prop1Output + pid1Adj;
  
  prop2Output = prop2Output + pid2Adj;
  
  prop5Output = prop5Output + pid5Adj;

  //set the system update rate
  //while(millis() - currentMilli < 200);
  
  //update all motors after adjustment
  prop1.writeMicroseconds(prop1Output);
  prop2.writeMicroseconds(prop2Output);
  //prop3.writeMicroseconds(prop3Hover);
  //prop4.writeMicroseconds(prop4Hover);
  prop5.writeMicroseconds(prop5Output);
  //prop6.writeMicroseconds(prop6Hover);
}

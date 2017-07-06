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

int cDepthErr;
int pDepthErr;
int pid5Adj;

int pid1Adj;
int pid2Adj;
int pid3Adj;
int pid4Adj;

float currentDepth, previousDepth;
int currentMilli, previousMilli;

int cycleCount = 0;

float targetDepth;
float startDepth;


int prop1Output = MOTOR_HOVER_START_1;
int prop2Output = MOTOR_HOVER_START_2;
int prop5Output = MOTOR_HOVER_START_5;

//gains may be implemented again
float pGainRoll = 0.25; //set the gain for the speed based on the level correction changes
float pGainPitch = 0.25; //set the gain for the speed based on the level correction changes
float pGainYaw = 1; //set the gain for the speed based on the level correction changes
float dGainRoll = 0.12; //set the gain for the speed based on the level correction changes
float dGainPitch = 0.12; //set the gain for the speed based on the level correction changes
float dGainYaw = 1; //set the gain for the speed based on the level correction changes
float pGainDepth = 1; //set the gain for the PID speed based on the vertical equilibrium changes
float dGainDepth = 2; //

Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

void setup() {
  Wire.begin();

  Serial.begin(115200);
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
  setRoll = rpy.y();
  setPitch = rpy.x();
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

  delay(10000);
   
  pSensor.read(); //read sensor
  currentDepth = startDepth = pSensor.depth(); //set initial depth  
  targetDepth = startDepth - 0.10; //set target depth
  
  currentMilli = millis(); //begin tracking time
  
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);
 
  delay(40); //delay for good measure
  cDepthErr = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target
  
  
}
void loop() {
  imu->IMURead();
  delay(50);
  //Read sensor inputs
  pSensor.read(); //Pressure
  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  roll = rpy.y();    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  pitch = rpy.x();   //value decreses pitching down, increases pitching up
  yaw = rpy.z();

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

  pDepthErr = cDepthErr;

  //Now in cm/s

  //meters to cm
  cDepthErr = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target

  
//  pid3Adj = -(cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw); //yaw
//  pid4Adj = cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw; //yaw
    pid5Adj = cDepthErr*pGainDepth + (cDepthErr - pDepthErr)*dGainDepth; // depth

    pid1Adj = -(cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll) + cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch
    pid2Adj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll + cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch

  //speed is in centimeters per second, directly based on how many cemtimeters away the robot is
  // targetVelocity = pDiff/2 cm/s;

  
  //Depth Control
  //Gauge how far away robot needs to go
  //Gauge speed of robot
  
  //for every cm away robot is, target velocity is that distance per second
  prop1Output += pid1Adj;
  
  prop2Output += pid2Adj;
  
  prop5Output += pid5Adj;

  if(prop5Output < 1500)
  {
    prop5Output = 1500;
  }

  
  //set the system update rate, comment out for fastest possible
  //while(millis() - currentMilli < 200);
  
  //update all motors after adjustment
  prop1.writeMicroseconds(prop1Output);
  prop2.writeMicroseconds(prop2Output);
  //prop3.writeMicroseconds(prop3Hover);
  //prop4.writeMicroseconds(prop4Hover);
  prop5.writeMicroseconds(prop5Output);
  //prop6.writeMicroseconds(prop6Hover);
}

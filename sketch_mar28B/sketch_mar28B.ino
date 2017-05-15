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

RTVector3 xyz;

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


int xInput, yInput, zInput;

int xDiff, yDiff, pDiff;

float currentDepth, previousDepth;
int currentMilli, previousMilli;

float velocity;
float targetVelocity;

float targetDepth;
float startDepth;

int accelX;
int accelY;




int prop1Hover = MOTOR_HOVER_START_1;
int prop2Hover = MOTOR_HOVER_START_2;
int prop5Hover = MOTOR_HOVER_START_5;

//gains may be implemented again
float xyGain = 1; //set the gain for the speed based on the level correction changes
float pGain = 1; //set the gain for the speed based on the vertical equilibrium changes
float dGain = 1;
float speedGain = 1;
float Kp = 1;
float Kd = 1;
float Ki = 1;

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
  targetDepth = startDepth - 0.10; //set target depth
  
  currentMilli = millis(); //begin tracking time
  
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);
 
  delay(40); //delay for good measure
  
}
void loop() {
  imu->IMURead();
  delay(400);
  //Read sensor inputs
  pSensor.read(); //Pressure
  xyz = fusion.getFusionPose();

  
  yInput = analogRead(A1); //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  xInput = analogRead(A2); //value decreses pitching down, increases pitching up

  //an example of code layout in a future version
  //Hover(&propSpeed, pSensor.depth(), xInput, yInput);
  
  //Save information from last cycle and assign info for current cycle
  previousMilli = currentMilli;
  currentMilli = millis();
  previousDepth = currentDepth;
  currentDepth = pSensor.depth();

  //Now in cm/s
  velocity = ((currentDepth*100 - previousDepth*100)*1000)/(currentMilli - previousMilli); // +ve = moving down, -ve = moving up

  //meters to cm
  pDiff = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target

  //speed is in centimeters per second, directly based on how many cemtimeters away the robot is
  targetVelocity = pDiff*speedGain; // +ve = want to move up, -ve = want to move down
  
  //need the absolute value to test largest change easily, not change direction
  xDiff = fabs(accelX - xInput); 
  yDiff = fabs(accelY - yInput);

  //X and Y stability
  if(xDiff > yDiff) 
  {
    if(xInput < accelX)
    {
      if(prop1Hover < 1899)
        prop1Hover++;
      if(prop2Hover < 1899)
        prop2Hover++;
      if(prop5Hover > 1526)
        prop5Hover--;
    }
    else if(xInput > accelX)
    {
      if(prop1Hover > 1526)
        prop1Hover--;
      if(prop2Hover > 1526)
        prop2Hover--;
      if(prop5Hover < 1899)
        prop5Hover++;
    }
  }
  else
  {
    if(yInput < accelY)
    {
      if(prop1Hover < 1899)
        prop1Hover++;
      if(prop2Hover > 1526)
        prop2Hover--;
    }
    else if(yInput > accelY)
    {
      if(prop1Hover > 1526)
        prop1Hover--;
      if(prop2Hover < 1899)
        prop2Hover++;
    }  
  }

  //Depth Control
  //Gauge how far away robot needs to go
  //Gauge speed of robot
  
  //for every cm away robot is, target velocity is that distance per second
  
  
  if(targetVelocity > 0) //if robot wants to move up
  {
    if(-velocity < targetVelocity) //if moving down or moving up too slowly, increase thrust
    {
      if(prop1Hover < 1899)
        prop1Hover++;
      if(prop2Hover < 1899)
        prop2Hover++;
      if(prop1Hover < 1899)
        prop5Hover = prop5Hover+2;
    }
    else if(-velocity > targetVelocity) //if moving up too fast, decrease thrust
    {
      if(prop1Hover > 1526)
        prop1Hover--;
      if(prop2Hover > 1526)
        prop2Hover--;
      if(prop5Hover > 1528)
        prop5Hover = prop5Hover-2;
    }
  }
  else if(targetVelocity < 0) //wants to move down
  {
    if(velocity < targetVelocity) //if moving down too fast, increase thrust
    {
      if(prop1Hover < 1899)
        prop1Hover++;
      if(prop2Hover < 1899)
        prop2Hover++;
      if(prop1Hover < 1899)
        prop5Hover = prop5Hover+2;
    }
    else if(velocity > targetVelocity) //if moving up or moving down too slowly, decrease thrust
    {
      if(prop1Hover > 1526)
        prop1Hover--;
      if(prop2Hover > 1526)
        prop2Hover--;
      if(prop5Hover > 1528)
        prop5Hover = prop5Hover-2;
    }
  }
  
  //update all motors after adjustment
  prop1.writeMicroseconds(prop1Hover);
  prop2.writeMicroseconds(prop2Hover);
  //prop3.writeMicroseconds(prop3Hover);
  //prop4.writeMicroseconds(prop4Hover);
  prop5.writeMicroseconds(prop5Hover);
  //prop6.writeMicroseconds(prop6Hover);
}

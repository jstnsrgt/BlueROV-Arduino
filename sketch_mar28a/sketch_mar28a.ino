#include <RTFusionRTQF.h>
#include <RTIMU.h>
#include <RTIMUBNO055.h>
#include <RTIMUGD20HM303D.h>
#include <RTIMUGD20HM303DLHC.h>
#include <RTIMUGD20M303DLHC.h>
#include <RTIMULibDefs.h>
#include <RTIMULSM9DS0.h>
#include <RTIMUMPU9150.h>
#include <RTIMUMPU9250.h>
#include <RTIMUSettings.h>
#include <RTMath.h>
#include <RTPressure.h>
#include <RTPressureBMP180.h>
#include <RTPressureDefs.h>
#include <RTPressureLPS25H.h>
#include <RTPressureMS5611.h>
#include <RTArduLink.h>
#include <RTArduLinkDefs.h>
#include <RTArduLinkDemoDefs.h>
#include <RTArduLinkHAL.h>
#include <RTArduLinkUtils.h>
#include <I2Cdev.h>
#include <CalLib.h>

#include <MS5837.h>
#include <Servo.h>
#include <Wire.h>
#include "Conversions.h"
#include "MotorSpeedDefinitions.h"
#include <math.h>
#include <EEPROM.h>

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

int accelX;
int accelY;

int prop1Hover = MOTOR_HOVER_START_1;
int prop2Hover = MOTOR_HOVER_START_2;
int prop5Hover = MOTOR_HOVER_START_5;

//gains may be implemented again
float xyGain = 0.2; //set the gain for the speed based on the level correction changes
float pGain = 1; //set the gain for the speed based on the vertical equilibrium changes
float dGain = 1;
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

void setup() {
  Wire.begin();
  pSensor.init();
  pSensor.setFluidDensity(FRESHWATER);
  delay(3000);



  pinMode(A0, INPUT); // Z
  pinMode(A1, INPUT); // Y
  pinMode(A2, INPUT); // X
  pinMode(groundPin, OUTPUT);
  digitalWrite(groundPin, LOW);
  
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

  pSensor.read();
  while(pSensor.depth() < 0.02)
  {
    pSensor.read();
    delay(200);  
  }
  
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);
  
  //pCurrent = analogRead()
  targetDepth = 0.20; //set target depth
  //pSensor.read(); //read sensor
  //currentDepth = pSensor.depth(); //set initial depth 
  //currentMilli = millis(); //begin tracking time
  delay(40); //delay for good measure
  
}

void loop() {

  //Read sensor inputs
  pSensor.read(); //Pressure
  //yInput = analogRead(A1); //Rotational Position
  //xInput = analogRead(A2);

  //Save information from last cycle and assign info for current cycle
  //previousDepth = currentDepth;
  currentDepth = pSensor.depth();

  //calculate velocity data
  //pDiff = currentDepth - targetDepth; //pdif: +ve = lower than target, -ve = higher than target
  /*
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
      if(prop1Hover > 1526)
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
  }*/

  if(currentDepth < targetDepth - 0.01)
  {
    prop1.writeMicroseconds(getMicrosecondsForward(5));
    prop2.writeMicroseconds(getMicrosecondsForward(5));
    prop5.writeMicroseconds(getMicrosecondsForward(10));
  }
  else if(currentDepth > targetDepth + 0.01)
  {
    prop1.writeMicroseconds(getMicrosecondsForward(7.5));
    prop2.writeMicroseconds(getMicrosecondsForward(7.5));
    prop5.writeMicroseconds(getMicrosecondsForward(15));
  }
  else
  {
    prop1.writeMicroseconds(getMicrosecondsForward(6.5));
    prop2.writeMicroseconds(getMicrosecondsForward(6.5));
    prop5.writeMicroseconds(getMicrosecondsForward(13));
  }
}

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


/* DONT CARE ABOUT Z AT THIS POINT IN TIME
int accelZMin = ;
int accelZ = ;
int accelZMax = ;
*/

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
  delay(10000);



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

  delay(5000);
  accelX = analogRead(A2);
  accelY = analogRead(A1);
  prop1.writeMicroseconds(MOTOR_FORWARD_5P);
  prop2.writeMicroseconds(MOTOR_FORWARD_5P);
  prop3.writeMicroseconds(MOTOR_FORWARD_5P);
  prop4.writeMicroseconds(MOTOR_FORWARD_5P);
  prop5.writeMicroseconds(MOTOR_FORWARD_5P);
  prop6.writeMicroseconds(MOTOR_FORWARD_5P);
  delay(1000);
  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);
  prop5.writeMicroseconds(MOTOR_STOP);
  prop6.writeMicroseconds(MOTOR_STOP);
  delay(20000);
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);

  
  //pCurrent = analogRead()
  targetDepth = 0.20; //set target depth
  pSensor.read(); //read sensor
  currentDepth = pSensor.depth(); //set initial depth 
  currentMilli = millis(); //begin tracking time
  delay(40); //delay for good measure
  
}

void loop() {
  // put your main code here, to run repeatedly:
  previousMilli = currentMilli;
  currentMilli = millis();
  previousDepth = currentDepth;
  currentDepth = pSensor.depth();
  pDiff = currentDepth - targetDepth; //pdif: +ve = lower than target, -ve = higher than target
  targetVelocity = pDiff/(currentMilli - previousMilli);
  velocity = (currentDepth - previousDepth)/(currentMilli - previousMilli); //float value in meters, +ve = dropping, -ve = lifting

  yInput = analogRead(A1);
  xInput = analogRead(A2);

  //need the absolute value to test largest change easily, not change direction
  xDiff = fabs(accelX - xInput); 
  yDiff = fabs(accelY - yInput);
  //pDiff = pCurrent - pPrevious;

  //X and Y stability
  if(xDiff > yDiff) //determine
  {
    if(xInput < accelX)
    {
      if(prop1Hover < 1900)
        prop1Hover = prop1Hover++;
      if(prop2Hover < 1900)
        prop2Hover = prop2Hover++;
      if(prop5Hover > 1525)
        prop5Hover = prop5Hover--;
    }
    else if(xInput > accelX)
    {
      if(prop1Hover > 1525)
        prop1Hover = prop1Hover--;
      if(prop1Hover > 1525)
        prop2Hover = prop2Hover--;
      if(prop5Hover < 1900)
        prop5Hover = prop5Hover++;
    }
  }
  else
  {
    if(yInput < accelY)
    {
      if(prop1Hover < 1900)
        prop1Hover = prop1Hover++;
      if(prop2Hover > 1525)
        prop2Hover = prop2Hover--;
    }
    else if(yInput > accelY)
    {
      if(prop1Hover > 1525)
        prop1Hover = prop1Hover--;
      if(prop2Hover < 1900)
        prop2Hover = prop2Hover++;
    }  
  }

/*
  //Height Alteration, Velocity: +ve = dropping, -ve = lifting
  if(targetVelocity > velocity)
  {
    if(prop1Hover > 1540)
      prop1Hover = prop1Hover-3;
    if(prop2Hover > 1540)
      prop2Hover = prop2Hover-3;
    if(prop5Hover > 1540)
      prop5Hover = prop5Hover-3;
  }
  else
  {
    if(prop1Hover < 1885)
      prop1Hover = prop1Hover+3;
    if(prop2Hover < 1885)
      prop2Hover = prop2Hover+3;
    if(prop2Hover < 1885)
      prop5Hover = prop5Hover+3;
  }
  */

  prop1.writeMicroseconds(prop1Hover);
  prop2.writeMicroseconds(prop2Hover);
  prop5.writeMicroseconds(prop5Hover);
}




/*
  //pSensor.readTestCase();
  pSensor.read();

  Serial.print("Pressure: "); 
  Serial.print(pSensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(pSensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(pSensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(pSensor.altitude()); 
  Serial.println(" m above mean sea level");

  delay(2000);
  */

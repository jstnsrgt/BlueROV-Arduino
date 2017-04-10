#include <Servo.h>
#include "Conversions.h"
#include "MotorSpeedDefinitions.h"
#include <math.h>

byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;

int accelXMin = 260;  //front down
int accelX = 324;     //stable
int accelXMax = 390;  //front up

int accelYMin = 260;  //roll left
int accelY = 319;     //stable
int accelYMax = 390;  //roll right

/* DONT CARE ABOUT Z AT THIS POINT IN TIME
int accelZMin = ;
int accelZ = ;
int accelZMax = ;
*/

int xInput, yInput, zInput;

int xDiff, yDiff, pDiff;

int pCurrent, pPrevious;

int xyGain = 1; //set the gain for the speed based on the level correction changes
int pGain = 1; //set the gain for the speed based on the vertical equilibrium changes

int prop1Hover = MOTOR_HOVER_START_1;
int prop2Hover = MOTOR_HOVER_START_2;
int prop5Hover = MOTOR_HOVER_START_5;

Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

void setup() {
  // put your setup code here, to run once:
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
  
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);

  zInput = analogRead(A0);
  yInput = analogRead(A1);
  xInput = analogRead(A2);
  //pCurrent = analogRead()
}

void loop() {
  // put your main code here, to run repeatedly:

  //pPrevious = pCurrent; 
  
  zInput = analogRead(A0);
  yInput = analogRead(A1);
  xInput = analogRead(A2);

  xDiff = fabs(accelX - xInput);
  yDiff = fabs(accelY - yInput);
  //pDiff = pCurrent - pPrevious;

  //X and Y stability
  if(xDiff > yDiff)
  {
    if(xInput < accelX - 3)
    {
      prop1Hover = prop1Hover + (xDiff*xyGain);
      prop2Hover = prop2Hover + (xDiff*xyGain);
      prop5Hover = prop5Hover - (xDiff*xyGain);
    }
    else if(xInput > accelX + 3)
    {
      prop1Hover = prop1Hover - (xDiff*xyGain);
      prop2Hover = prop2Hover - (xDiff*xyGain);
      prop5Hover = prop5Hover + (xDiff*xyGain);
    }
  }
  else
  {
    if(yInput < accelY - 3)
    {
      prop1Hover = prop1Hover + (xDiff*xyGain);
      prop2Hover = prop2Hover - (xDiff*xyGain);
    }
    else if(yInput > accelY + 3)
    {
      prop1Hover = prop1Hover - (xDiff*xyGain);
      prop2Hover = prop2Hover + (xDiff*xyGain);
    }  
  }

  //Height Alteration
  /*
  if( change in pressure is positive ) //if the change in pressure is positive
  {
    prop1Hover = prop1Hover + (pDiff*diffGain);
    prop2Hover = prop2Hover + (pDiff*diffGain);
    prop5Hover = prop5Hover - (pDiff*diffGain);
  }
  else if( change in pressure is negative ) //if the change in pressure is positive
  {
    prop1Hover = prop1Hover + (pDiff*diffGain);
    prop2Hover = prop2Hover + (pDiff*diffGain);
    prop5Hover = prop5Hover + (pDiff*diffGain);
  }
  */
  
  
  prop1.writeMicroseconds(prop1Hover);
  prop2.writeMicroseconds(prop2Hover);
  prop5.writeMicroseconds(prop5Hover);
  delay(20);
}

#include <MS5837.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

MS5837 pSensor;
//probably unnecessary unless tested in seawater
#define FRESHWATER 997
#define SEAWATER 1029

#define MOTOR_HOVER_START_5 1581
#define MOTOR_HOVER_START_1 1553
#define MOTOR_HOVER_START_2 1553

#define MOTOR_STOP 1500

//pin terminal masks
byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;


float prevError;
float currentError;
int Pgain;

float errorRate;
int Dgain;

float currentDepth, previousDepth;
float targetDepth, startDepth;

int currentMilli, previousMilli;
float currentSec, previousSec;

//int xInput, yInput, zInput;
//int xDiff, yDiff;
//int accelX;
//int accelY;

int prop1Hover = MOTOR_HOVER_START_1;
int prop2Hover = MOTOR_HOVER_START_2;
int prop5Hover = MOTOR_HOVER_START_5;

/*//gains may be implemented again
float xyGain = 1; //set the gain for the speed based on the level correction changes
float pGain = 1; //set the gain for the speed based on the vertical equilibrium changes
float dGain = 1;
float speedGain = 1;
float Kp = 1;
float Kd = 1;
float Ki = 1;*/

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
  
  //pinMode(A1, INPUT); // Y
  //pinMode(A2, INPUT); // X

  pinMode(13,OUTPUT); //Ready LED

  //accelY = analogRead(A1);
  //accelX = analogRead(A2);
  
  
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
  delay(3000);
  pSensor.read(); //read sensor
  startDepth = currentDepth = pSensor.depth();
  targetDepth = startDepth - 0.10; //set target depth
  currentMilli = millis(); //begin tracking time
  currentError = targetDepth - currentDepth;
  
  prop1.writeMicroseconds(MOTOR_HOVER_START_1);
  prop2.writeMicroseconds(MOTOR_HOVER_START_2);
  prop5.writeMicroseconds(MOTOR_HOVER_START_5);

  Pgain = 100;
  Dgain = 0;
  
  
  delay(40); // delay for good measure
}
void loop() {
  //Read sensor inputs
  pSensor.read(); //Pressure
  //yInput = analogRead(A1); //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  //xInput = analogRead(A2); //value decreses pitching down, increases pitching up
  
  //Save information from last cycle and assign info for current cycle
  previousMilli = currentMilli;
    previousSec = previousMilli/1000;
  currentMilli = millis();
    currentSec = currentMilli/1000;
  previousDepth = currentDepth;
  currentDepth = pSensor.depth();
  
  prevError = currentError;
  currentError = targetDepth - currentDepth;

  // rate of change in error in metres per second
  errorRate = (currentError - prevError)/(currentSec - previousSec); // +ve = moving down, -ve = moving up

  // Depth Control
  // Gauge how far away robot needs to go
  // Gauge speed of robot

  // calculating prop adjustments for depth control
  float prop1DepthAdj = -(currentError * Pgain + errorRate * Dgain);
  float prop2DepthAdj = -(currentError * Pgain + errorRate * Dgain);
  float prop5DepthAdj = -(currentError * Pgain + errorRate * Dgain) * 2;  // set prop5 to double power of prop1/prop2

  // setting PWM dependent on depth and accelerometer control
  prop1Hover = MOTOR_HOVER_START_1 + prop1DepthAdj;
  prop2Hover = MOTOR_HOVER_START_2 + prop2DepthAdj;
  prop5Hover = MOTOR_HOVER_START_5 + prop5DepthAdj;

  // update all motors after adjustment
  prop1.writeMicroseconds(prop1Hover);
  prop2.writeMicroseconds(prop2Hover);
  prop5.writeMicroseconds(prop5Hover);
  
  /*//need the absolute value to test largest change easily, not change direction
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
  }*/
}

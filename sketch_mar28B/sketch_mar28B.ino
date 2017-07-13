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


float setRoll, setPitch, setYaw; //target
float roll, pitch, yaw;
int cRollErr,cPitchErr,cYawErr; //current
int pRollErr,pPitchErr,pYawErr; //previous

//depth error values
int cDepthErr; //current
int pDepthErr; //previous

//stores control system output for each thruster
int pid1Adj;
int pid2Adj;
int pid3Adj;
int pid4Adj;
int pid5Adj; 

//raw depth values
float currentDepth, previousDepth;

//time keeping variables, rollover protection not required - takes > 50 days
int currentMilli, previousMilli;


int cycleCount = 0;

// depth tracking values
float targetDepth;
float startDepth;

// close approximation stable start values
int prop1Output = T1_BASE;
int prop2Output = T2_BASE;
int prop5Output = T5_BASE;

//gains may be implemented again
float pGainRoll = 0.8; //set the gain for the speed based on the level correction changes
float pGainPitch = 0.8; //set the gain for the speed based on the level correction changes
float pGainYaw = 1; //set the gain for the speed based on the level correction changes
float dGainRoll = 5; //set the gain for the speed based on the level correction changes
float dGainPitch = 5; //set the gain for the speed based on the level correction change
float dGainYaw = 1; //set the gain for the speed based on the level correction changes
float pGainDepth = 1; //set the gain for the PID speed based on the vertical equilibrium changes
float dGainDepth = 2; //

//thruster output represented as Servo objects, PWM signal managed by functions
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

void setup() {
  Wire.begin(); //for I2C communication with pressure and IMU sensors

  // create the imu object
  imu = RTIMU::createIMU(&settings);
  imu->IMUInit();
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.5);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  setRoll = rpy.y()*100;
  setPitch = rpy.x()*100;
  setYaw = rpy.z()*100;
  
  //initialise pressure sensor
  pSensor.init();
  pSensor.setFluidDensity(FRESHWATER);
  
  pinMode(13,OUTPUT); //Ready LED

  //assign servo objects to their corresponding output pins
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);
  prop5.attach(propPin5);
  prop6.attach(propPin6);

  //begin a constant PWM output with duty cycle corresponding to the dead zone of the thrusters
  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);
  prop5.writeMicroseconds(MOTOR_STOP);
  prop6.writeMicroseconds(MOTOR_STOP);

  delay(10000);
  //ten second buffer to stabilise speed controllers and also in case of program upload
   
  pSensor.read(); //read pressure sensor
  currentDepth = startDepth = pSensor.depth(); //set initial depth  
  targetDepth = startDepth - 0.10; //set target depth
  
  currentMilli = millis(); //begin tracking time
  
  //output thruster power values close to bouyancy and stability
  prop1.writeMicroseconds(T1_BASE);
  prop2.writeMicroseconds(T2_BASE);
  prop5.writeMicroseconds(T5_BASE);
 
  delay(40); //delay for to allow some motion to occur
  
  //calculate first depth error value
  cDepthErr = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target
  
  
}
void loop() {
  while(!imu->IMURead());
  //Read sensor inputs
  pSensor.read(); //Pressure
  
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());

  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  roll = rpy.y()*100;    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  pitch = rpy.x()*100;   //value decreses pitching down, increases pitching up
  yaw = rpy.z()*100;

  //roll over current error values into previous values
  pRollErr = cRollErr;
  pPitchErr = cPitchErr;
  pYawErr = cYawErr;
  
  //update current error values
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

//APPEARS TO WORK CORRECTLY
    pid5Adj = cDepthErr*pGainDepth + (cDepthErr - pDepthErr)*dGainDepth; // depth

//NEITHER PITCH NOR ROLL WORKS CORRECTLY
    pid1Adj = /*-(cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll) +*/ cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch
    pid2Adj =  /*cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll +*/ cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch

  //speed is in centimeters per second, directly based on how many cemtimeters away the robot is
  // targetVelocity = pDiff/2 cm/s;

  
  //Depth Control
  //Gauge how far away robot needs to go
  //Gauge speed of robot
  
  //for every cm away robot is, target velocity is that distance per second

  prop5Output = T5_BASE + pid5Adj;

  
  if(prop5Output > 1525)
  {
    prop1Output = 1525 + 0.5*(prop5Output - 1525) + pid1Adj;
    prop2Output = 1525 + 0.5*(prop5Output - 1525) + pid2Adj;
  }
  else
  {
    prop1Output = 1525 + pid1Adj;
    prop2Output = 1525 + pid2Adj;
  }

  //if output falls into dead zone, change instead to corresponding reverse thrust value
  if(prop1Output < 1525)
  {
    prop1Output -= 50;
  }
  if(prop2Output < 1525)
  {
    prop2Output -= 50;
  }
  if(prop5Output < 1525)
  {
    prop5Output -= 50;
  }
  

// Reverse 40% max power limit
  if(prop5Output < 1320)
  {
    prop5Output = 1320;
  }
  if(prop1Output < 1320)
  {
    prop1Output = 1320;
  }
  if(prop2Output < 1320)
  {
    prop2Output = 1320;
  }
  
  
  // 50 % max power limit
  if(prop5Output > 1712)
  {
    prop5Output = 1712;
  }
  if(prop1Output > 1712)
  {
    prop1Output = 1712;
  }
  if(prop2Output > 1712)
  {
    prop2Output = 1712;
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

#include <MS5837.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"


#define TIME_STOP 1000
#define TIME_REV 3000
#define TIME_CW 5000

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

int bandwidth = 5; // used for adding a dead band to bi-directional thruster code

float setRoll, setPitch, setYaw; //target
float roll, pitch, yaw;
int cRollErr,cPitchErr,cYawErr; //current
int pRollErr,pPitchErr,pYawErr; //previous

//depth error values
int cDepthErr, pDepthErr; //current + previous

//stores control system output for each thruster
int pid1Adj, pid2Adj, pid3Adj, pid4Adj, pid5Adj; 

int rollAdj, pitchAdj, yawAdj;

volatile bool frontLeftSwitch = false;
volatile bool frontRightSwitch = false;


//values in milliseconds
unsigned long timePassed = 0;


volatile int interruptTime;


//raw depth values
float currentDepth, previousDepth;

//time keeping variables, rollover protection not required - takes > 50 days
int currentMilli, previousMilli;

int cycleCount = 0;
int addr = 0;
bool eso = false;
// depth tracking values
float targetDepth, startDepth;


int prop3Output, prop4Output;

//gains may be implemented again
float pGainRoll = 0.3; //set the gain for the speed based on the level correction changes
float pGainPitch = 0.3; //set the gain for the speed based on the level correction changes
float pGainYaw = 0.7; //set the gain for the speed based on the level correction changes
float dGainRoll = 1; //set the gain for the speed based on the level correction changes
float dGainPitch = 1; //set the gain for the speed based on the level correction change
float dGainYaw = 2; //set the gain for the speed based on the level correction changes
float pGainDepth = 1; //set the gain for the PID speed based on the vertical equilibrium changes
float dGainDepth = 2; //

//thruster output represented as Servo objects, PWM signal managed by functions
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(); //for I2C communication with pressure and IMU sensors

  // create the imu object
  imu = RTIMU::createIMU(&settings);
  imu->IMUInit();
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.1);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  while(!imu->IMURead());
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  rpy = fusion.getFusionPose();
  
  //initialise pressure sensor
  /*
  for (int i = 0 ; i < EEPROM.length() ; i++) 
  {
    EEPROM.write(i, 0);
  }
  */
  pinMode(13,OUTPUT); //Ready LED
  
  //assign servo objects to their corresponding output pins
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);

  //begin a constant PWM output with duty cycle corresponding to the dead zone of the thrusters
  //delay(5000);
  //ten second buffer to stabilise speed controllers and also in case of program upload

  pitch = 0.0;
  
  // wait for pitch values to settle (longest settling time)
  while((abs(pitch) > (abs(setPitch) + 0.1)) || (abs(pitch) < (abs(setPitch) - 0.1)))
  {
      while(!imu->IMURead());
      fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
      rpy = fusion.getFusionPose();
      pitch = rpy.x() * 100;
      delay(100);
  }
  
  
  pSensor.init();
  pSensor.setFluidDensity(FRESHWATER);
  
  /*for(addr = 0; addr < 200; addr += sizeof(float))
  {
    pSensor.read();
    EEPROM.put(addr,pSensor.depth());
    delay(200);
    
  }*/

  pSensor.read();      
}


void loop() {
  while(!imu->IMURead());
  //Read sensor inputs
  pSensor.read();
  
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());

  rpy = fusion.getFusionPose();

  Serial.println(pSensor.depth());
  Serial.println(rpy.x());
  Serial.println(rpy.y());
  Serial.println(rpy.z());
  delay(1000);
}


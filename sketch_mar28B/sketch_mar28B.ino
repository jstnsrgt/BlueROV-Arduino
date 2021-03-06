#include <Arduino.h>
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

#define ROLL_RANGE 50
#define PITCH_RANGE 50
#define YAW_RANGE 50
#define DEPTH_RANGE 30


#define TEST_ROLL 0 //POSITIVE RIGHT, NEGATIVE LEFT
#define TEST_PITCH 120 //POSITIVE FORWARD, NEGATIVE REVERSE
#define YAW_TEST 0 //POSITIVE LEFT ROTATION, NEGATIVE RIGHT ROTATION

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



//left test
//int yawTest = 30;

//right test
//int yawTest = -30;
int yawTest = YAW_TEST;


int setRoll, setPitch, setYaw;
int targetRoll, targetPitch, targetYaw;
int roll, pitch, yaw;
int cRollErr,cPitchErr,cYawErr; //current
int pRollErr,pPitchErr,pYawErr; //previous

int currentPitch;

//depth error values
int cDepthErr, pDepthErr; //current + previous

//stores control system output for each thruster
int pid1Adj, pid2Adj, pid3Adj, pid4Adj; 

int rollAdj, pitchAdj, yawAdj, depthAdj;

int t1_base = T1_BASE;
int t2_base = T1_BASE;
int t3_base = T1_BASE;
int t4_base = T1_BASE;

int t1_output = (int)t1_base;
int t2_output = (int)t2_base;
int t3_output = (int)t3_base;
int t4_output = (int)t4_base;


//raw depth values
int currentDepth, previousDepth;

//time keeping variables, rollover protection not required - takes > 50 days
int currentMilli, previousMilli;

// depth tracking values
int targetDepth, setDepth;

// close approximation stable start values

//gains may be implemented again
float pGainRoll = 0.2; //set the gain for the speed based on the level correction changes
float pGainPitch = 0.2; //set the gain for the speed based on the level correction changes
float pGainYaw = 1; //set the gain for the speed based on the level correction changes
float dGainRoll = 1*pGainRoll; //set the gain for the speed based on the level correction changes
float dGainPitch = 1*pGainPitch; //set the gain for the speed based on the level correction change
float dGainYaw = 2; //set the gain for the speed based on the level correction changes
float pGainDepth = 1; //set the gain for the PID speed based on the vertical equilibrium changes
float dGainDepth = 2*pGainDepth;

//thruster output represented as Servo objects, PWM signal managed by functions
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;


bool serialError = false;
volatile bool readSerial = false;



void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  attachInterrupt(digitalPinToInterrupt(18) , s1Read , RISING);
  pinMode(13,OUTPUT); //Ready LED
  
  //assign servo objects to their corresponding output pins
  
  Wire.begin(); //for I2C communication with pressure and IMU sensors
  // create the imu object
  imu = RTIMU::createIMU(&settings);
  imu->IMUInit();
  
  fusion.setSlerpPower(0.1);
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  while(!imu->IMURead());
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  rpy = fusion.getFusionPose();

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  targetRoll = setRoll = rpy.x()*100;
  setRoll += TEST_ROLL;
  targetPitch = setPitch = rpy.y()*100;
  setPitch -= TEST_PITCH;
  targetYaw = rpy.z()*100;

  cRollErr = 0;
  cPitchErr = 0;
  cYawErr = 0;
  
  pSensor.init();
  pSensor.setFluidDensity(FRESHWATER);

  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);
  delay(5000);
  
  delay(100);
  
  currentMilli = millis(); //begin tracking time
  
  //output thruster power values close to bouyancy and stability
 
  pSensor.read();
  currentDepth = pSensor.depth()*100;      //set initial depth in mm
  setDepth = targetDepth = currentDepth - 30;      //set target depth
  cDepthErr = currentDepth - targetDepth; //pdif: +ve = lower than target, -ve = higher than target
  //currentPitch = 0;
  /*
  while((abs(currentPitch) > (abs(setPitch) + 1)) || (abs(currentPitch) < (abs(setPitch) - 1)))
  
    while(!imu->IMURead());
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    rpy = fusion.getFusionPose();
    currentPitch = rpy.x();
    delay(100);
  }
  */
  delay(40); //delay for to allow some motion to occur
}


void loop() {
  
  if(readSerial == true)
  {
    Serial.println("Reading Serial");
    Serial.println(Serial2.available());
    
    if(Serial2.available() > 0)
    {
      byte header = Serial2.read();
      Serial.println(header);
      Serial.println(header, BIN);
      switch((int)header)
      {
        case 0:
          Serial.println("Case 0");
          if(Serial2.available() == 3)
          {
            uint8_t angleDeg = (Serial2.read()*360)/255;
            uint8_t angleRad = angleDeg*DEG_TO_RAD;
            uint8_t intensity = Serial2.read();
            targetRoll = setRoll + (cos(angleRad)*intensity*ROLL_RANGE)/100;
            targetDepth = setDepth + (sin(angleRad)*intensity*DEPTH_RANGE)/100;
            Serial.println("Target Depth : ");
            Serial.println(targetDepth);

            int8_t fwd = Serial2.read();
            targetPitch = setPitch - (fwd*PITCH_RANGE)/100;
            Serial.println("Target Pitch : ");
            Serial.println(targetPitch);
          }
          else
            serialError = true;
        break;
        case 1:
          Serial.println("Case 1");
          if(Serial2.available() == 1)
          {
            int yaw = Serial2.read();
            yawTest = (yaw*YAW_RANGE)/100;
            Serial.println("Yaw : ");
            Serial.println(yawTest);
            Serial2.write(1);
        break;
          }
          else
            serialError = true;
        break;
        case 2:
          Serial.println("Case 2");
          targetRoll = setRoll;
          targetPitch = setPitch;
          targetYaw = setYaw;
        break;
        break;
        case 3:
          Serial.println("Case 3");
          targetDepth = setDepth;
          targetRoll = setRoll;
          targetPitch = setPitch;
          targetYaw = setYaw;
        break;
      }
      Serial2.flush();
    }
    else
      serialError = true;
    if(serialError == true)
    {
      Serial2.write(0);
      serialError = false;
    }
    else 
      Serial2.write(1);
    readSerial = false;
  }
  
  // routine to control obstacle avoidance for front two bumpers 
  
  previousMilli = currentMilli;
  currentMilli = millis();
  while(!imu->IMURead());
  //Read sensor inputs
  pSensor.read();
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  rpy = fusion.getFusionPose();
  
  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  roll = rpy.x()*100;    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  pitch = rpy.y()*100;   //value decreses pitching down, increases pitching up
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
  currentDepth = pSensor.depth()*100;
  pDepthErr = cDepthErr;

  //meters to cm
  cDepthErr = currentDepth - targetDepth; //pdif: +ve = lower than target, -ve = higher than target

//APPEARS TO WORK CORRECTLY
    depthAdj = cDepthErr*pGainDepth + (cDepthErr - pDepthErr)*dGainDepth; // depth, +ve = lower than target, -ve = higher than target
//Roll having problems
    rollAdj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll; // +ve, rolling left
    pitchAdj = cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; // +ve pitching up
    yawAdj = cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw; //
    pid1Adj = depthAdj -rollAdj -pitchAdj + yawTest;
    pid2Adj = depthAdj +rollAdj -pitchAdj - yawTest;
    pid3Adj = depthAdj -rollAdj +pitchAdj - yawTest;
    pid4Adj = depthAdj +rollAdj +pitchAdj + yawTest;

    if(cDepthErr == 0 && depthAdj > 0)
    {
      t1_base++;
      t2_base++;
      t3_base++;
      t4_base++;
    }
    else if(cDepthErr == 0 && depthAdj < 0)
    {
      t1_base--;
      t2_base--;
      t3_base--;
      t4_base--;
    }

    if(cRollErr == 0 && rollAdj > 0)
    {
      t1_base--;
      t2_base++;
      t3_base--;
      t4_base++;
    }
    else if(cRollErr == 0 && rollAdj < 0)
    {
      t1_base++;
      t2_base--;
      t3_base++;
      t4_base--;
    }

    if(cPitchErr == 0 && pitchAdj > 0)
    {
      t1_base--;
      t2_base--;
      t3_base++;
      t4_base++;
    }
    else if(cPitchErr == 0 && pitchAdj < 0)
    {
      t1_base++;
      t2_base++;
      t3_base--;
      t4_base--;
    }

    t1_output = t1_base - pid1Adj;
    t2_output = t2_base - pid2Adj;
    t3_output = t3_base - pid3Adj;
    t4_output = t4_base - pid4Adj;

    if(t1_output > 1472)
      t1_output = 1472;
    if(t2_output > 1472)
      t2_output = 1472;
    if(t3_output > 1472)
      t3_output = 1472;
    if(t4_output > 1472)
      t4_output = 1472;

    if(t1_output < 1300)
      t1_output = 1300;
    if(t2_output < 1300)
      t2_output = 1300;
    if(t3_output < 1300)
      t3_output = 1300;
    if(t4_output < 1300)
      t4_output = 1300;

    

    //prop1.writeMicroseconds(t1_output);
    //prop2.writeMicroseconds(t2_output);
    //prop3.writeMicroseconds(t3_output);
    //prop4.writeMicroseconds(t4_output);

/*
    Serial.println(rpy.z());
    */

    /*
    Serial.println(cDepthErr);
    Serial.println(cRollErr);
    Serial.println(cPitchErr);
    Serial.println(cYawErr);

    Serial.println();
*/
    
    /*
    Serial.println(t1_output);
    Serial.println(t2_output);
    Serial.println(t3_output);
    Serial.println(t4_output);
    */
    //Serial.println();
    //Serial.println(currentMilli - previousMilli);
}



void s1Read() {
  readSerial = true;
}


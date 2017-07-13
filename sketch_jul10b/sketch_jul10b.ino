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

byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;

Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;


float setRoll, setPitch, setYaw; //target
float roll, pitch, yaw;
int cRollErr,cPitchErr,cYawErr; //current
int pRollErr,pPitchErr,pYawErr; //previous

//stores control system output for each thruster
int pid1Adj;
int pid2Adj;

//time keeping variables, rollover protection not required - takes > 50 days
int currentMilli, previousMilli;

//gains may be implemented again
float pGainRoll = 0.25; //set the gain for the speed based on the level correction changes
float pGainPitch = 0.25; //set the gain for the speed based on the level correction changes
float pGainYaw = 1; //set the gain for the speed based on the level correction changes
float dGainRoll = 0.12; //set the gain for the speed based on the level correction changes
float dGainPitch = 0.12; //set the gain for the speed based on the level correction changes
float dGainYaw = 1; //set the gain for the speed based on the level correction changes

void setup() {
  Serial.begin(115200);
  Wire.begin(); //for I2C communication with pressure and IMU sensors


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
  
  
  for(int i = 0; i < 10000; i++)
  {
  while(!imu->IMURead());
  currentMilli = millis();
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  rpy = fusion.getFusionPose();
  Serial.print(currentMilli);
  Serial.print("\t");
  Serial.print(rpy.x());
  Serial.print("\t");
  Serial.print(rpy.y());
  Serial.print("\t");
  Serial.println(rpy.z());
  }
  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  setRoll = rpy.y()*100;
  setPitch = rpy.x()*100;
  
  pinMode(13,OUTPUT); //Ready LED

  //ten second buffer to stabilise speed controllers and also in case of program upload
   
  currentMilli = millis(); //begin tracking time
}


void loop() {
  if(imu->IMURead())
  {
  //Read sensor inputs
  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());

  rpy = fusion.getFusionPose();
  /*
  Serial.println(rpy.x());
  Serial.println(rpy.y());
  Serial.println(rpy.z());
    Serial.println();
*/
  

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  roll = rpy.y()*100;    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  pitch = rpy.x()*100;   //value decreses pitching down, increases pitching up

  //roll over current error values into previous values
  pRollErr = cRollErr;
  pPitchErr = cPitchErr;
  
  //update current error values
  cRollErr = roll - setRoll;
  cPitchErr = pitch - setPitch;

//NEITHER PITCH NOR ROLL WORKS CORRECTLY
    pid1Adj = /*-(cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll) +*/ cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch
    pid2Adj = /*cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll +*/ cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch; //roll+pitch
/*  
  Serial.print("Motor 1 adjustment: ");
  Serial.println(pid1Adj);
  
  Serial.print("Motor 2 adjustment: ");
  Serial.println(pid2Adj);
  Serial.println();
  
  Serial.print("Current Roll Error: ");
  Serial.println(cRollErr);
  
  Serial.print("Current Pitch Error: ");
  Serial.println(cPitchErr);
  Serial.println();

  Serial.print("Previous Roll Error: ");
  Serial.println(pRollErr);
  
  Serial.print("Previous Pitch Error: ");
  Serial.println(pPitchErr);
  Serial.println();

  Serial.print("(Roll, Pitch) : (");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.println(")");
  //RTMath::displayRollPitchYaw("Pose: ", (RTVector3&)fusion.getFusionPose());  
  delay(1000);
  */
  }
}

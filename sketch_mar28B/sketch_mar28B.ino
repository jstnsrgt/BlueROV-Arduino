b#include <MS5837.h>
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


#define TIME_FWD 8000
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
byte propPin5 = 6;
byte propPin6 = 7;

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

// close approximation stable start values
int t5_base = T5_BASE;
int t1_base = T1_BASE;
int t2_base = T2_BASE;
int prop1Output = T1_BASE;
int prop2Output = T2_BASE;
int prop5Output = T5_BASE;
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
Servo prop5;
Servo prop6;

void setup() {
  //Serial.begin(115200);
  
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

  //X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
  setRoll = rpy.y()*100;
  setPitch = rpy.x()*100;
  setYaw = rpy.z()*100;
  
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
  prop5.attach(propPin5);
  prop6.attach(propPin6);

  //begin a constant PWM output with duty cycle corresponding to the dead zone of the thrusters
  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);
  prop5.writeMicroseconds(MOTOR_STOP);
  prop6.writeMicroseconds(MOTOR_STOP);
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

  pSensor.read();                               //read pressure sensor
  currentDepth = startDepth = pSensor.depth();  //set initial depth  
  targetDepth = startDepth - 0.35;              //set target depth
  
  currentMilli = millis(); //begin tracking time
  
  //output thruster power values close to bouyancy and stability
  //prop1.writeMicroseconds(T1_BASE);
  //prop2.writeMicroseconds(T2_BASE);
  //prop5.writeMicroseconds(T5_BASE);
 
  delay(40); //delay for to allow some motion to occur
  
  //calculate first depth error value
  cDepthErr = (currentDepth*100) - (targetDepth*100); //pdif: +ve = lower than target, -ve = higher than target


  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(19),frontRight, FALLING);
  attachInterrupt(digitalPinToInterrupt(18),frontLeft, FALLING);
  timePassed = 0;
}
void loop() {
  /*
  Serial.print(digitalRead(18));
  Serial.print("\t");
  Serial.println(digitalRead(19));
  delay(500);
  */
//values in milliseconds
  /////////////////////////////////////////////////////////////////////////////////////////////////////

  // routine to control obstacle avoidance for front two bumpers 
  
  previousMilli = currentMilli;
  currentMilli = millis();
  
    // time passed since activating microswitch                                                     ///  
  timePassed += (currentMilli - previousMilli);

  if(frontLeftSwitch || frontRightSwitch)
  {                                                                                            ///
  // FORWARD
    if (timePassed < TIME_FWD)                                                                          ///
    {
      prop3.writeMicroseconds(T3_FORWARD);   // FWD                                           ///
      prop4.writeMicroseconds(T4_FORWARD);   // FWD
    }                                                                                               ///
    else if (timePassed >= TIME_FWD && timePassed < TIME_FWD + TIME_REV)                                               ///
    {
      prop3.writeMicroseconds(T3_BACKWARD);                                                              ///
      prop4.writeMicroseconds(T4_BACKWARD);
      if(frontLeftSwitch)
        prop6.writeMicroseconds(T6_POWER_FORWARD);
      else if(frontRightSwitch)
        prop6.writeMicroseconds(T6_POWER_BACK);///
    }
    else if(timePassed >= TIME_FWD + TIME_REV && timePassed < TIME_FWD + TIME_REV + TIME_CW)
    {
      prop6.writeMicroseconds(MOTOR_STOP);
      prop3.writeMicroseconds(T3_TURN_CW);                                                              ///
      prop4.writeMicroseconds(T4_TURN_CW);
    }
    // obstacle avoidance complete: reset variables to enable interrupts for next encounter
    else                                                                                            ///
    {
      frontLeftSwitch = false;
      frontRightSwitch = false;
      prop6.writeMicroseconds(MOTOR_STOP);
      prop3.writeMicroseconds(MOTOR_STOP);
      prop4.writeMicroseconds(MOTOR_STOP);
      timePassed = 0;
      eso = !eso;///
    }
    
}                                                                                             
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  
  while(!imu->IMURead());
  //Read sensor inputs
  pSensor.read();
  
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

 

//APPEARS TO WORK CORRECTLY
    pid5Adj = cDepthErr*pGainDepth + (cDepthErr - pDepthErr)*dGainDepth; // depth

//NEITHER PITCH NOR ROLL WORKS CORRECTLY
    rollAdj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll;
    pitchAdj = cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch;
    yawAdj = cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw;

    pid1Adj = -rollAdj + pitchAdj; //roll+pitch
    pid2Adj =  rollAdj + pitchAdj; //roll+pitch

    pid3Adj = -yawAdj;
    pid4Adj = yawAdj;

  //speed is in centimeters per second, directly based on how many cemtimeters away the robot is
  // targetVelocity = pDiff/2 cm/s;

  
  //Depth Control
  //Gauge how far away robot needs to go
  //Gauge speed of robot
  
  //for every cm away robot is, target velocity is that distance per second
  

  
  prop5Output = t5_base + pid5Adj;
  prop1Output = 1525 + (prop5Output - 1525)*0.3 + pid1Adj;
  prop2Output = 1525 + (prop5Output - 1525)*0.3 + pid2Adj;
  //prop3Output = 1475 - pid3Adj;
  //prop4Output = 1475 - pid4Adj;
  //if output falls intgo dead zone, change instead to corresponding reverse thrust value

/*
  if(prop1Output < 1525)
    prop1Output -= 50 - bandwidth;
  if(prop2Output < 1525)
    prop2Output -= 50 - bandwidth;
  if(prop5Output < 1535)
    prop5Output -= 50 - bandwidth;

  if(prop1Output > 1525)
    prop1Output -= bandwidth;
  if(prop2Output > 1525)
    prop2Output -= bandwidth;
  if(prop5Output > 1535)
    prop5Output -= bandwidth;
  */
 
  if(prop5Output > 1600)
    prop5Output = 1600;
  if(prop1Output > 1600)
    prop1Output = 1600;
  if(prop2Output > 1600)
    prop2Output = 1600;
   
  if(prop5Output < 1525)
    prop5Output = 1525;
  if(prop1Output < 1525)
    prop1Output = 1525;
  if(prop2Output < 1525)
    prop2Output = 1525;
    
  
  //set the system update rate, comment out for fastest possible
  //while(millis() - currentMilli < 200);
  
  //update all motors after adjustment
  
  prop1.writeMicroseconds(prop1Output);
  prop2.writeMicroseconds(prop2Output);
  //prop3.writeMicroseconds(prop3Output);
  //prop4.writeMicroseconds(prop4Output);
  prop5.writeMicroseconds(prop5Output);

/*
  if (frontLeftSwitch == false && frontRightSwitch == false)
  {
    prop3.writeMicroseconds(1456);
    prop4.writeMicroseconds(1456);
  }
 */


  /*
  if(cycleCount % 10 == 0)
  {
    int dep = currentDepth*100;
    if(addr <  EEPROM.length() - 8*sizeof(int))
    {
      EEPROM.put(addr,rpy.y());
      EEPROM.put(addr+sizeof(float),cRollErr);
      EEPROM.put(addr+sizeof(float) + sizeof(int),rollAdj);
      addr += 2*sizeof(int) + sizeof(float);
      EEPROM.put(addr,rpy.x());
      EEPROM.put(addr+sizeof(float),cPitchErr);
      EEPROM.put(addr+sizeof(float) + sizeof(int),pitchAdj);
      addr += 2*sizeof(int) + sizeof(float);
    }
  }*/
  cycleCount++;
}


void frontLeft()                                                                      //
{
  // only activate obstacle avoidance if not already avoiding obstacle
  if (frontLeftSwitch == false && frontRightSwitch == false)
  {                                                                                   //
    prop3.writeMicroseconds(MOTOR_STOP);
    prop4.writeMicroseconds(MOTOR_STOP);
    frontLeftSwitch = true;                                                           
  }                                                                                   //
}

// right bumper interrupt function
void frontRight()                                                                     //
{
  // only activate obstacle avoidance if not already avoiding obstacle
  if (frontLeftSwitch == false && frontRightSwitch == false)  
  {                                                                                   //
    prop3.writeMicroseconds(MOTOR_STOP);
    prop4.writeMicroseconds(MOTOR_STOP);
    frontRightSwitch = true;                                                              
  }                                                                                   //
}


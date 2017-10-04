#include <stm32f10x_gpio.h>
#include <Wire.h>

#include <ServoM3.h>
#include <ServoTimer.h>
#include <stm32f10x.h>
#include <system_stm32f10x.h>

//sketch uses ~= 25kB program storage, <900B dynamic, 31/8/17


#include "Data.h"
#include "Sensors.h"
#include "Control.h"
#include "PID.h"
#include "Commands.h"

#include <Servo.h>
#include <EEPROM.h>

#define MOTOR_STOP 1500
#define CMD_INIT      0x00
#define CMD_SUCCESS   0x01
#define CMD_NULL      0x02


//pin terminal masks
byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;

//volatile bool frontLeftSwitch = false;
//volatile bool frontRightSwitch = false;
//volatile int interruptTime;

uint8_t currentCmd;
unsigned long timePassed = 0; //values in milliseconds
int pwm[4];

int prop1Output;
int prop2Output;
int prop3Output;
int prop4Output;

//thruster output represented as Servo objects, PWM signal managed by functions
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;


Data d;
Sensors sensorReader(&d);
PID pid(&d);

void setup() {
  Serial.begin(115200);
  pinMode(13,OUTPUT); //Ready LED
  
  //assign servo objects to their corresponding output pins
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);

  //begin a constant PWM output with duty cycle corresponding to the dead zone of the thrusters
  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);
  
  Serial.print(CMD_INIT);
}


void loop() {
  if(Serial.available() > 0) //check for availability of a byte in buffer
  {
    currentCmd = Serial.read(); //reads and stores the available byte
    Serial.print(CMD_SUCCESS); //send successfully recieved command
  }
  else
    Serial.print(CMD_NULL);
  // time passed since activating microswitch                                                     ///  
  sensorReader.Update();
  pid.Update(pwm);

  //set the system update rate, comment out for fastest possible
  //while(millis() - currentMilli < 200);
  //update all motors after adjustment
  prop1.writeMicroseconds(pwm[0]);
  prop2.writeMicroseconds(pwm[1]);
  prop3.writeMicroseconds(pwm[2]);
  prop4.writeMicroseconds(pwm[3]);
}

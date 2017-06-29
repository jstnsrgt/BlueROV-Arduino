
#include <Servo.h>
#include "PID.h"
#include "SensorReader.h"
#include "SensorData.h"

SensorReader sensors;
PID pid;


Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;


float gains[9];
gains[0] = 1; //P Depth
gains[1] = 1; //P RPY
gains[2] = 1; //P Forward
gains[3] = 1; //I Depth
gains[4] = 1; //I RPY
gains[5] = 1; //I Forward
gains[6] = 2; //D Depth
gains[7] = 1; //D RPY
gains[8] = 1; //D Forward

int prop1PWM = 1500;
int prop2PWM = 1500;
int prop3PWM = 1500;
int prop4PWM = 1500;
int prop5PWM = 1500;
int prop6PWM = 1500;

void setup() {
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);
  prop5.attach(propPin5);
  prop6.attach(propPin6);
  prop1.writeMicroseconds(prop1PWM);
  prop2.writeMicroseconds(prop2PWM);
  prop3.writeMicroseconds(prop3PWM);
  prop4.writeMicroseconds(prop4PWM);
  prop5.writeMicroseconds(prop5PWM);
  prop6.writeMicroseconds(prop6PWM);
  delay(1000);
  
  // put your setup code here, to run once:
  sensors = new SensorReader();
  pid = new PID(sensors.Read(), gains);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  
  int comp[] = pid.Cycle(sensors.Read());

  if(comp.count() == 5)
  {
    prop1PWM += comp[0];
    prop2PWM += comp[1];
    prop3PWM += comp[2];
    prop4PWM += comp[3];
    prop5PWM += comp[4];
  }

  prop1.writeMicroseconds(prop1PWM);
  prop2.writeMicroseconds(prop2PWM);
  prop3.writeMicroseconds(prop3PWM);
  prop4.writeMicroseconds(prop4PWM);
  prop5.writeMicroseconds(prop5PWM);
  prop6.writeMicroseconds(prop6PWM);
}

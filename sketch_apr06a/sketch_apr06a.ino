#include <Servo.h>
#include "Conversions.h"
#define MOTOR_STOP 1500

byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;

Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;

void setup() {
  // put your setup code here, to run once:
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);

  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);

  
  delay(1000);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  prop1.writeMicroseconds(getMicrosecondsBackward(5));
  delay(500);
  prop1.writeMicroseconds(MOTOR_STOP);
  delay(2000);
  
  prop2.writeMicroseconds(getMicrosecondsBackward(5));
  delay(500);
  prop2.writeMicroseconds(MOTOR_STOP);
  delay(2000);

  prop3.writeMicroseconds(getMicrosecondsBackward(5));
  delay(500);
  prop3.writeMicroseconds(MOTOR_STOP);
  delay(2000);
  
  prop4.writeMicroseconds(getMicrosecondsBackward(5));
  delay(500);
  prop4.writeMicroseconds(MOTOR_STOP);
  delay(2000);

  
}

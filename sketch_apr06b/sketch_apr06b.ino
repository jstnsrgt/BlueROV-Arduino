#include <Servo.h>
#include "Conversions.h"
#define MOTOR_STOP 1500

#define MOTOR_HOVER_START_5 1562
#define MOTOR_HOVER_START_1 1543
#define MOTOR_HOVER_START_2 1543

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

void setup() {
  // put your setup code here, to run once:
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

  
  delay(1000);
}

void loop() {
  for(int i = 0; i < 10; i++)
  {
    prop1.writeMicroseconds(getMicrosecondsForward(5));
    prop2.writeMicroseconds(getMicrosecondsForward(5));
    prop5.writeMicroseconds(getMicrosecondsForward(10));

    prop3.writeMicroseconds(getMicrosecondsBackward(5 + i*5));
    prop4.writeMicroseconds(getMicrosecondsBackward(i*5));
  
    delay(8000);

    prop3.writeMicroseconds(getMicrosecondsForward(MOTOR_STOP));
    prop4.writeMicroseconds(getMicrosecondsForward(MOTOR_STOP));
  
    delay(2000);
  }

}

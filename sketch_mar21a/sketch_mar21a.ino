#include <MS5837.h>

#include <Servo.h>
#include "Conversions.h"
#include "MotorSpeedDefinitions.h"
#include <Wire.h>

//Map the PWM pins on the board a corresponding propeller pin name
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;
byte propPin5 = 6;
byte propPin6 = 7;


//Choose 4 analogue pins for battery cell voltage input
#define BATTERY_INPUT_1 A4 //Cell 1
#define BATTERY_INPUT_2 A5 //Cell 2
#define BATTERY_INPUT_3 A6 //Cell 3
#define BATTERY_INPUT_4 A7 //Cell 4



//Each output will be treated as a servo controller
//as the Blue Robitics Basic ESC accepts PWM as input
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;
Servo prop5;
Servo prop6;

/* ESC INPUT
 * PWM duty cycle
 * ESC translates microseconds of pulse width into motor output
 * Stop Motor:    1500 Microsecond PW
 * Forward:       ~ 1525(1%) to 1900(100%) Microsecond PW
 * Reverse:       ~ 1475(1%) down to 1100(100%) Mircosecond PW
 */
 int accelY, accelX;

void setup() {
  // Telling each servo object which port to use

 
  
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);
  prop5.attach(propPin5);
  prop6.attach(propPin6);

  pinMode(A0, INPUT); // Z
  pinMode(A1, INPUT); // Y
  pinMode(A2, INPUT); // X
  
  Serial.begin(9600);


  prop1.writeMicroseconds(MOTOR_STOP);
  prop2.writeMicroseconds(MOTOR_STOP);
  prop3.writeMicroseconds(MOTOR_STOP);
  prop4.writeMicroseconds(MOTOR_STOP);
  prop5.writeMicroseconds(MOTOR_STOP);
  prop6.writeMicroseconds(MOTOR_STOP);
  
  delay(5000);
  
  // Wait for speed controllers to initialise
}


void loop() {
  accelY = analogRead(A1); //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
  accelX = analogRead(A2); //value decreses pitching down, increases pitching up
  Serial.println(accelY);
  Serial.println(accelX);
  delay(1000);
}


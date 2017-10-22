#include <ServoM3.h>
#include <stm32f10x.h>
#include <system_stm32f10x.h>

#include <ServoM3.h>

#define pwm_1 6
#define pwm_2 3
#define pwm_3 4
#define pwm_4 5

Servo p1;
Servo p2;
Servo p3;
Servo p4;

void setup() {
  //Serial.begin(115200);
  p1.attach(pwm_1);
  //p2.attach(pwm_2);
  //p3.attach(pwm_3);
  //p4.attach(pwm_4);
  p1.writeMicroseconds(1500);
  delay(10000);
}

void loop() {
  /*
  //Serial1.println("Full Loop");
  delay(1000);
  p1.writeMicroseconds(1550);
  delay(1000);
  p1.writeMicroseconds(1500);
  delay(1000);
  
  p2.writeMicroseconds(1550);
  delay(1000);
  p2.writeMicroseconds(1500);
  delay(1000);
  p3.writeMicroseconds(1550);
  delay(1000);
  p3.writeMicroseconds(1500);
  delay(1000);
  p4.writeMicroseconds(1550);
  delay(1000);
  p4.writeMicroseconds(1500);
  */

}

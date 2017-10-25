#include <Servo.h>

Servo p1;
Servo p2;
Servo p3;
Servo p4;

byte toRead=0;


void setup() {
  p1.attach(2);
  p2.attach(3);
  p3.attach(4);
  p4.attach(5);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial3.available() > 0)
  {
    //delay(1000);
    byte toRead = Serial3.read(); //&& B01111111;
    /*byte toPrint = 0;
    if(toRead && B01000000 == 64)
      toPrint |= B00000001;
    if(toRead && B00100000 == 32)
      toPrint |= B00000010;
    if(toRead && B00010000 == 16)
      toPrint |= B00000100;
    if(toRead && B00001000 == 8)
      toPrint |= B00001000;
    if(toRead && B00000100 == 4)
      toPrint |= B00010000;
    if(toRead && B00000010 == 2)
      toPrint |= B00100000;
    if(toRead && B00000001 == 1)
      toPrint |= B01000000;*/
     
    Serial.println(toRead, DEC);
    //Serial.println(toRead, OCT);
    //Serial.println(toRead, HEX);
    Serial.println(toRead, BIN);
  }
}

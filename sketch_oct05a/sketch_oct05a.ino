#include <ServoM3.h>
#include <Arduino.h>

Servo test1;

int prevMicro = 0;
int avSum = 0;

void setup() {
  Serial1.begin(115200);
  test1.attach(6);
  
  pinMode(4,INPUT);
  
  //test1.writeMicroseconds(1500);
  //test2.writeMicroseconds(1500);
  //test3.writeMicroseconds(1500);
  //test4.writeMicroseconds(1500);

  for(int j = 1500; j <= 1900; j++)
  {
    test1.writeMicroseconds(j);
    for(int i = 0; i < 4; i++)
    {
      prevMicro = micros();
      while(digitalRead(4)==HIGH);
      avSum += micros() - prevMicro;
      while(digitalRead(4)==LOW);
      while(digitalRead(4)==HIGH);
      while(digitalRead(4)==LOW);
    }
    Serial1.println((int)avSum/4);
    avSum = 0;
  }
}

void loop() {
  
}

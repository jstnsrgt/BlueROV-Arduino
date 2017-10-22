#include <ServoM3.h>
//I should be shot for making this

Servo test;

#define x(value) (value - 1499)
#define currentTestPWM 1500

int y = 1.1248*x(currentTestPWM) + 1580.6;
int z = x(currentTestPWM) + 1499;
int diff = y-z;
int output = currentTestPWM - diff;

int prevMicro = 0;


void setup() {
  Serial1.begin(115200);
  // put your setup code here, to run once:
  test.attach(6);
  test.writeMicroseconds(output);
  pinMode(4,INPUT);
  Serial1.println(output);
}

void loop() {
  
}

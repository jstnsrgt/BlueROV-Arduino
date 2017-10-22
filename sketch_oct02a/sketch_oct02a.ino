#include <Arduino.h>

#define BIT_RANGE 4096

#define FREQ 100 //Hz
#define UPDATE_MICRO 10000
#define STOP (int)(1500/UPDATE_MICRO)*BIT_RANGE
#define GO (int)(1550/UPDATE_MICRO)*BIT_RANGE


int pwm_0 = 0;
int pwm_1 = 1;
int pwm_6 = 6;
int pwm_7 = 7;

void setup() {
  //pwmMode(pwm_0, FREQ, PWM_12_BIT);
  //pwmMode(pwm_1, FREQ, PWM_12_BIT);
  pwmMode(pwm_6, FREQ, PWM_12_BIT);
  pwmMode(pwm_7, FREQ, PWM_12_BIT);
  //pwmWrite(0, STOP);
  //pwmWrite(1, STOP);
  pwmWrite(6, STOP);
  pwmWrite(7, STOP);
  delay(5000);
}

void loop() {
  /*
  //pwmWrite(0, GO);
  //pwmWrite(1, GO);
  pwmWrite(6, GO);
  //pwmWrite(7, GO);
  delay(1000);
  //pwmWrite(0, STOP);
  //pwmWrite(1, STOP);
  pwmWrite(6, STOP);
  //pwmWrite(7, STOP);
  delay(5000);
  */
}

//WRITE EEPROM
#include <MS5837.h>
#include <Wire.h>
#include <EEPROM.h>

MS5837 pSensor;

#define FRESHWATER 997
#define SEAWATER 1029


byte snapShot = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  Wire.begin();
  pSensor.init();
  pSensor.setFluidDensity(FRESHWATER);
  delay(10000);
  
  digitalWrite(13,HIGH);
  for(int addr = 0; addr < EEPROM.length(); addr += sizeof(float))
  {
    pSensor.read();
    EEPROM.put(addr,pSensor.pressure());
  delay(200);
  }
  digitalWrite(13,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}

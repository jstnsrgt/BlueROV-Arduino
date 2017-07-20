//EEPROM float read

#include <EEPROM.h>



void setup() {
  int intPrint;
  float floatPrint;
  Serial.begin(115200);
  delay(10000);
  for(int addr = 0; addr < EEPROM.length(); addr += sizeof(int)+ sizeof(float))
  {
     EEPROM.get(addr,floatPrint);
     Serial.print(floatPrint);
     Serial.print("\t");
     //EEPROM.get(addr+sizeof(int),toPrint);
     //Serial.print(toPrint);
     //Serial.print("\t");
     EEPROM.get(addr+sizeof(float),intPrint);
     Serial.println(intPrint);
  }
  digitalWrite(13,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50000);
}

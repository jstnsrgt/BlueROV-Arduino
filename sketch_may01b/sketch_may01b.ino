//EEPROM float read

#include <EEPROM.h>



void setup() {
  int cycleCount = 0;
  int intPrint;
  float floatPrint;
  Serial.begin(115200);
  delay(10000);
  for(int addr = 0; addr < EEPROM.length(); addr += 2*sizeof(int)+ sizeof(float))
  {
     EEPROM.get(addr,floatPrint);
     Serial.print(floatPrint);
     Serial.print("\t");
     //EEPROM.get(addr+sizeof(float),intPrint);
     //Serial.print(intPrint);
     //Serial.print("\t");
     //EEPROM.get(addr+sizeof(float) + sizeof(int),intPrint);
     Serial.print(intPrint);
     if(cycleCount == 1)
     {
      Serial.println();
      cycleCount = 0;
     }
     else
     {
      cycleCount++;
     Serial.print("\t");
     }
     
     
  }
  digitalWrite(13,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50000);
}

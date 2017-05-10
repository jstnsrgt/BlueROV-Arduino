//EEPROM float read

#include <EEPROM.h>



void setup() {
  float toPrint;
  Serial.begin(115200);
  delay(5000);
  for(int addr = 0; addr < EEPROM.length(); addr += sizeof(float))
  {
     EEPROM.get(addr,toPrint);
     Serial.println(toPrint);
  }
  digitalWrite(13,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50000);
}

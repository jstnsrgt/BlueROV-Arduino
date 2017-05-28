//#include <SoftwareSerial.h>

//SoftwareSerial BT_Serial(2,3); // RX | TX
//change for arduino mega if needed, currently set for UNO

char c = ' ';

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200); 
  Serial.println("Arduino is ready");
  Serial.println("Remember to select NL & CR in the serial monitor");

  //initialise BT at AT mode baud rate
  //BT_Serial.begin(38400);

  
  
}

void loop() {
  // put your main code here, to run repeatedly:
    /*
    if (BT_Serial.available())
    {  
        c = BT_Serial.read();
        Serial.write(c);
    }
 */
    // Keep reading from Arduino Serial Monitor and send to HC-05
    if (Serial.available())
    {
        c =  Serial.read();
        Serial.write(c);  
    }
}

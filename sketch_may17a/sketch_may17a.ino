
//change for arduino mega if needed, currently set for UNO

char c = ' ';

void setup() {
  // put your setup code here, to run once:

  Serial.begin(38400);
  Serial.println("Arduino is ready");
  Serial.println("Remember to select NL & CR in the serial monitor");

  //initialise BT at AT mode baud rate
  Serial1.begin(38400);

  
  
}

void loop() {
  // put your main code here, to run repeatedly:
    
    if (Serial1.available())
    {  
        c = Serial1.read();
        Serial.write(c);
    }
 
    // Keep reading from Arduino Serial Monitor and send to HC-05
    if (Serial.available())
    {
        c =  Serial.read();
        Serial1.write(c);  
    }
}

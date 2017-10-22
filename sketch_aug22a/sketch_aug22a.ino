//sketch uses ~= 25kB program storage, <900B dynamic, 31/8/17


#include "Data.h"
#include "Sensors.h"
#include "Control.h"
#include "PID.h"
#include "Commands.h"

#include <Servo.h>
#include <EEPROM.h>

//#define SERIAL_DEBUG 1

#define MOTOR_STOP 1500
#define PI_BAUD 115200
#define BT_BAUD 115200


//pin terminal masks
byte groundPin = 1;
byte propPin1 = 2;
byte propPin2 = 3;
byte propPin3 = 4;
byte propPin4 = 5;

//array for thruster control values
int pwm[4];

//thruster output represented as Servo objects, PWM signal managed by functions
Servo prop1;
Servo prop2;
Servo prop3;
Servo prop4;

//Centralised data struct
Data d;

//Object pointers assigned here for reference in both setup() and loop()
Sensors* sensorReader;
PID* pid;

void setup() {
  Serial.begin(BT_BAUD);
  //Serial1.begin(PI_BAUD);
  prop1.attach(propPin1);
  prop2.attach(propPin2);
  prop3.attach(propPin3);
  prop4.attach(propPin4);
  delay(5000); //after this delay, it is now acceptable to alter thruster values. I think the speed controllers like it
  //initialises the sensor object with a reference for access to data struct
  sensorReader = new Sensors(&d);
  //initialises control system with access to data struct
  pid = new PID(&d);
  
  //assign servo objects to their corresponding output pins
  //Note, attach function just happens to initialise a PWM at 1500 us pulse width, which corresponds to the default stop position of the t100 thrusters
  
}

//Main program loops indefinately
void loop() {
	// time passed since activating microswitch                                                     ///  
	sensorReader->Update(); //tells sensor object to read the sensors and update the values in the data struct accordingly
	pid->Update(pwm); //tells control system to crunch new numbers based on data struct and send back pwm values for all thrusters

	//set the system update rate, comment out for fastest possible
	//while(millis() - currentMilli < 200);
	//update all motors after adjustment

  /*
  Serial.println(pwm[0]);
  Serial.println(pwm[1]);
  Serial.println(pwm[2]);
  Serial.println(pwm[3]);
*/
  Serial.println(d.setPitch);
  Serial.println(d.setRoll);
  Serial.println(d.setYaw);
  Serial.println(d.setDepth);
  
  Serial.println();
	//prop1.writeMicroseconds(pwm[0]);
	//prop2.writeMicroseconds(pwm[1]);
	//prop3.writeMicroseconds(pwm[2]);
	//prop4.writeMicroseconds(pwm[3]);

  
}

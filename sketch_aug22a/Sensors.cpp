#include "Sensors.h"
#include <math.h>

//constructor for sensor reading object
Sensors::Sensors(Data* dPtr) { //takes in a reference to the one and only data structure instance
	*d = *dPtr; //gives sensor object a reference to the data struct

  //initialise sensors and save initial values
	Wire.begin();
	imu = RTIMU::createIMU(&settings);
	imu->IMUInit();
	fusion.setSlerpPower(0.1);
	fusion.setGyroEnable(true);
	fusion.setAccelEnable(true);
	fusion.setCompassEnable(true);
	while(!imu->IMURead());
		fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
	rpy = fusion.getFusionPose();
	d->setRoll = rpy.y()*100;
	d->setPitch = rpy.x()*100;
	d->setYaw = rpy.z()*100;
	d->currentPitch = 0.0;

  //standby while rpy sensor stabilises 
	while((abs(d->currentPitch) > (abs(d->setPitch) + 0.1)) || (abs(d->currentPitch) < (abs(d->setPitch) - 0.1)))
	{
		while(!imu->IMURead());
		fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
		rpy = fusion.getFusionPose();
		d->currentPitch = rpy.x() * 100;
		delay(100);
	}
	
	pSensor.init();
	pSensor.setFluidDensity(FRESHWATER);
	
	pSensor.read();                         //read pressure sensor
    d->currentDepth = pSensor.depth();  		//set initial depth  
    d->targetDepth = d->currentDepth - 0.35;      //set target depth
	d->currentMilli = millis();
	delay(40);
}

//updates data struct with new sensor values for currenct cycle
void Sensors::Update() {
	d->previousMilli = d->currentMilli;
	d->currentMilli = millis();
	
	while(!imu->IMURead()); //Read sensor inputs
	pSensor.read();
	fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
	rpy = fusion.getFusionPose();
	//X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
	d->currentRoll = rpy.y()*100;    //value decreases when side with motors 1 and 3 drops lower, value increases when side with motors 2 and 4 drops lower
	d->currentPitch = rpy.x()*100;   //value decreses pitching down, increases pitching up
	d->currentYaw = rpy.z()*100;

	d->previousDepth = d->currentDepth;
	d->currentDepth = pSensor.depth();
}

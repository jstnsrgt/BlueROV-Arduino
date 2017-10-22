#include "Sensors.h"
#include <math.h>

//constructor for sensor reading object
Sensors::Sensors(Data* dPtr) { //takes in a reference to the one and only data structure instance
	*d = *dPtr; //gives sensor object a reference to the data struct
  Wire.begin();
  //initialise sensors and save initial values
  fusion = new RTFusionRTQF();
  settings = new RTIMUSettings();
	
 //IMU sensor init
  
  
	imu = RTIMU::createIMU(settings);
	imu->IMUInit();
 
	fusion->setSlerpPower(0.1);
	fusion->setGyroEnable(true);
	fusion->setAccelEnable(true);
	fusion->setCompassEnable(true);

 
	while(!imu->IMURead());
	  fusion->newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
	rpy = fusion->getFusionPose();

  
	d->setRoll = d->targetRoll = d->currentRoll = rpy.y()*100;
	d->setPitch = d->targetPitch = d->currentPitch =  rpy.x()*100;
	d->setYaw = d->targetYaw = d->currentYaw = rpy.z()*100;
	
  //standby while rpy sensor stabilises
  /*
   * d->currentPitch = 0.0;
	while((abs(d->currentPitch) > (abs(d->setPitch) + 0.1)) || (abs(d->currentPitch) < (abs(d->setPitch) - 0.1)))
	{
		while(!imu->IMURead());
		fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
		rpy = fusion.getFusionPose();
		d->currentPitch = rpy.x() * 100;
		delay(100);
	}*/
  //Pressure sensor init
  pSensor = new MS5837();
	pSensor->init();
	pSensor->setFluidDensity(FRESHWATER);

	
	pSensor->read();                         //read pressure sensor

  d->currentDepth = pSensor->depth()*100;  		//set initial depth in mm
  d->setDepth = d->targetDepth = d->currentDepth - 35;      //set target depth

	d->currentMilli = millis();
	delay(40);
}

//updates data struct with new sensor values for currenct cycle
void Sensors::Update() {
  //update time reference
  
	d->previousMilli = d->currentMilli;
	d->currentMilli = millis();

  //read PRESSURE
  pSensor->read();
  d->previousDepth = d->currentDepth;
  d->currentDepth = pSensor->depth()*100;


  
	while(!imu->IMURead()); //Read sensor inputs
	fusion->newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
	rpy = fusion->getFusionPose();
  //shift current data to previous data variables
  d->previousRoll = d->currentRoll;
  d->previousPitch = d->currentPitch;
  d->previousYaw = d->currentYaw; //may change implementation of yaw
 //update data values
	//X Y Z IS INCORRECTLY ALIGNED, MUST TEST AUV FOR REAL CONFIGURATION
	d->currentRoll = rpy.x()*100;    //x() increases when rolling right, decreases when rolling left
	d->currentPitch = rpy.y()*100;   //y() increases when pitching up, decreases when pitching down
	d->currentYaw = rpy.z()*100; //may change implementation of yaw, z() has strange characteristics which make the value unpredictable

}

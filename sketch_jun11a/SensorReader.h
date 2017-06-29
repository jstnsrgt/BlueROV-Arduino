#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <MS5837.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include "SensorData.h"


#define FRESHWATER 997
#define SEAWATER 1029

class SensorReader
{
	private:
	RTIMU *imu;
	RTFusionRTQF fusion;
	RTIMUSettings settings;
	RTVector3 rpy;
	MS5837 pSensor;
	
	public:
	
	SensorReader()
	{
		Wire.begin();
		imu = RTIMU::createIMU(&settings);
		
		fusion.setSlerpPower(0.02);
		
		fusion.setGyroEnable(true);
		fusion.setAccelEnable(true);
		fusion.setCompassEnable(true);
		
		pSensor.init();
		pSensor.setFluidDensity(FRESHWATER);
	}
	
	SensorData Read()
	{
		imu->IMURead();
		delay(50);
		pSensor.read(); //Pressure
		rpy = fusion.getFusionPose();

		return (new SensorData( rpy , pSensor.depth() , 0 ));
		// ALT new SensorData( rpy , pSensor.pressure() , calc'd fwd speed )
	}
	
};



#endif

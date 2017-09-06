#ifndef SENSORS_H
#define SENSORS_H


#include <MS5837.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"

#include "Data.h"

#define FRESHWATER 997
#define SEAWATER 1029

class Sensors {
	private:
		Data* d;
		RTIMU *imu;                                           // the IMU object
		RTFusionRTQF fusion;                                  // the fusion object
		RTIMUSettings settings;                               // the settings object
		RTVector3 rpy;
		MS5837 pSensor;
	public:
	
		Sensors(Data*);
		void Update();
};

#endif
